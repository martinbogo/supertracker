#!/usr/bin/env python3
"""Live AIS ship map for a HackRF.

Needs: hackrf_transfer on PATH (brew install hackrf), numpy and scipy.
Run:   python3 ais_map.py --lat 45.5152 --lon -122.6784   then open http://127.0.0.1:8080
Test:  python3 ais_map.py --file capture.bin   (replays a 2.4 Msps int8 IQ file)

Antenna: a vertical whip about 45 cm long (quarter-wave at 162 MHz).
AIS is two 9600 baud GMSK channels, 161.975 and 162.025 MHz, in plain text
(unencrypted by design, it is a ship safety broadcast).
The HackRF can only do one job at a time, so stop adsb_map.py first.
"""
import argparse
import collections
import json
import math
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from scipy.signal import firwin, lfilter, resample_poly

FS = 2_400_000                        # HackRF sample rate
DEC = 50                              # 2.4 MHz / 50 = 48 kHz, 5 samples per bit at 9600 baud
CENTER = 162_000_000                  # both AIS channels are 25 kHz either side of this
CHANNELS = (-25_000, 25_000)          # 161.975 MHz and 162.025 MHz
CHUNK = 1_200_000                     # samples per step (0.5 s)
OVERLAP = 240_000                     # raw samples carried over so bursts are never cut (0.1 s)
EXPIRE_S = 600

vessels = {}
stats = {"msgs": 0, "peak_adc": 0, "load": 0.0, "status": "starting", "error": "", "bursts": 0, "bad_crc": 0}
msg_times = collections.deque()
lock = threading.Lock()
receiver = None
recent = {}


# ---------------------------------------------------------------- AIS message decoding
def crc_ok(bits):
    crc = 0xFFFF
    for b in bits:
        if (crc ^ b) & 1:
            crc = (crc >> 1) ^ 0x8408
        else:
            crc >>= 1
    return crc == 0xF0B8


def ship_type(t):
    if t is None:
        return "Unknown", "other"
    names = {30: "Fishing", 31: "Towing", 32: "Towing (large)", 33: "Dredging", 34: "Diving ops",
             35: "Military", 36: "Sailing", 37: "Pleasure craft", 50: "Pilot", 51: "Search and rescue",
             52: "Tug", 53: "Port tender", 55: "Law enforcement"}
    if t in names:
        n = names[t]
    elif 60 <= t < 70:
        n = "Passenger"
    elif 70 <= t < 80:
        n = "Cargo"
    elif 80 <= t < 90:
        n = "Tanker"
    elif 40 <= t < 50:
        n = "High speed craft"
    elif 20 <= t < 30:
        n = "Wing in ground"
    else:
        n = "Other"
    if t == 30:
        cat = "fishing"
    elif t in (36, 37):
        cat = "pleasure"
    elif t in (31, 32, 52, 53):
        cat = "tug"
    elif 60 <= t < 70:
        cat = "passenger"
    elif 70 <= t < 80:
        cat = "cargo"
    elif 80 <= t < 90:
        cat = "tanker"
    elif t in (35, 55, 51):
        cat = "military"
    else:
        cat = "other"
    return n, cat


def parse_message(bits, rssi, now):
    n = len(bits)
    if n < 38:
        return
    v = int("".join("1" if b else "0" for b in bits), 2)

    def f(a, k):
        return (v >> (n - a - k)) & ((1 << k) - 1) if a + k <= n else None

    def sf(a, k):
        x = f(a, k)
        if x is None:
            return None
        return x - (1 << k) if x >= 1 << (k - 1) else x

    def text(a, chars):
        s = ""
        for i in range(chars):
            c = f(a + 6 * i, 6)
            if c is None:
                break
            s += chr(c + 64) if c < 32 else chr(c)
        return s.replace("@", " ").strip()

    mtype, mmsi = f(0, 6), f(8, 30)
    upd = {}
    kind = "ship"
    if mtype in (1, 2, 3) and n >= 137:
        upd.update(sog=f(50, 10), lon=sf(61, 28), lat=sf(89, 27), cog=f(116, 12), hdg=f(128, 9), status=f(38, 4))
        div = 600000.0
    elif mtype == 18 and n >= 133:
        upd.update(sog=f(46, 10), lon=sf(57, 28), lat=sf(85, 27), cog=f(112, 12), hdg=f(124, 9))
        div = 600000.0
    elif mtype == 19 and n >= 263:
        upd.update(sog=f(46, 10), lon=sf(57, 28), lat=sf(85, 27), cog=f(112, 12), hdg=f(124, 9),
                   name=text(143, 20), type=f(263, 8))
        div = 600000.0
    elif mtype == 5 and n >= 300:
        upd.update(callsign=text(70, 7), name=text(112, 20), type=f(232, 8), dest=text(302, 20) if n >= 422 else None)
        div = None
    elif mtype == 24 and n >= 160:
        part = f(38, 2)
        if part == 0:
            upd.update(name=text(40, 20))
        elif part == 1:
            upd.update(type=f(40, 8))
        div = None
    elif mtype == 4 and n >= 134:
        upd.update(lon=sf(79, 28), lat=sf(107, 27))
        div, kind = 600000.0, "base"
    elif mtype == 21 and n >= 219:
        upd.update(name=text(43, 20), lon=sf(164, 28), lat=sf(192, 27))
        div, kind = 600000.0, "aton"
    else:
        return
    with lock:
        s = vessels.get(mmsi)
        if s is None:
            s = vessels[mmsi] = {"mmsi": mmsi, "name": None, "type": None, "kind": kind, "lat": None, "lon": None,
                                 "sog": None, "cog": None, "hdg": None, "dest": None, "callsign": None,
                                 "trail": [], "msgs": 0, "last": now, "rssi": 0.0}
        s["msgs"] += 1
        s["last"] = now
        s["rssi"] = rssi
        if kind != "ship":
            s["kind"] = kind
        for k in ("name", "type", "dest", "callsign"):
            if upd.get(k) not in (None, "", 0) or (k == "type" and upd.get(k)):
                s[k] = upd[k]
        if div and upd.get("lat") is not None and upd.get("lon") is not None:
            lat, lon = upd["lat"] / div, upd["lon"] / div
            if abs(lat) <= 90 and abs(lon) <= 180 and not (lat == 0 and lon == 0):
                s["lat"], s["lon"] = round(lat, 5), round(lon, 5)
                if not s["trail"] or s["trail"][-1] != [s["lat"], s["lon"]]:
                    s["trail"].append([s["lat"], s["lon"]])
                    del s["trail"][:-100]
        sog = upd.get("sog")
        if sog is not None and sog < 1023:
            s["sog"] = round(sog / 10.0, 1)
        cog = upd.get("cog")
        if cog is not None and cog < 3600:
            s["cog"] = round(cog / 10.0)
        hdg = upd.get("hdg")
        if hdg is not None:
            s["hdg"] = hdg if hdg < 360 else None
        stats["msgs"] += 1
        msg_times.append(now)


# ---------------------------------------------------------------- radio decoding
def hdlc_frames(bits):
    """bits: NRZI decoded 0/1 list. Yields unstuffed frames (payload plus 16 bit FCS)."""
    buf, ones, inframe = [], 0, False
    for b in bits:
        if b:
            ones += 1
            if inframe:
                buf.append(1)
                if ones >= 7:
                    inframe, buf = False, []
        else:
            if ones == 6:
                if inframe and len(buf) >= 7:
                    yield buf[:-7]                          # drop the closing flag (0 and six 1s)
                buf, inframe = [], True
            elif ones == 5:
                pass
            elif inframe:
                buf.append(0)
            ones = 0


def fix_order(bits):
    """AIS sends every byte least significant bit first. Reversing each group of 8 gives the field order of the standard."""
    out = []
    for i in range(0, len(bits), 8):
        out += bits[i:i + 8][::-1]
    return out


# Coherent MSK detector, following the approach of AIS-Catcher (GPL-3): square-law carrier offset correction, a narrow
# Gaussian filter, then a carrier phase search per symbol. It is about 3 dB better than a one bit differential detector,
# and the offset correction and narrow filter matter even more on weak signals.
GAUSS = np.array([2.06995719e-06, 3.18610148e-05, 3.40605309e-04, 2.52892989e-03, 1.30411453e-02,
                  4.67076746e-02, 1.16186141e-01, 2.00730781e-01, 2.40861391e-01, 2.00730781e-01,
                  1.16186141e-01, 4.67076746e-02, 1.30411453e-02, 2.52892989e-03, 3.40605309e-04,
                  3.18610148e-05, 2.06995719e-06])          # 17 tap Gaussian low-pass at 48 kHz, about 4.5 kHz wide
_TH = (2 * np.arange(8) + 1) * np.pi / 32
_ANG = np.concatenate([_TH, -_TH[::-1]])                    # 16 candidate carrier phase axes, 11.25 degrees apart
_COS, _SIN = np.cos(_ANG)[:, None], np.sin(_ANG)[:, None]
_ROT = np.array([1, 1j, -1, -1j])


def correct_offset(seg, n_fft=1024, delta=205, search=220):
    """Squaring an MSK signal gives two lines 9600 Hz apart. Their midpoint is twice the carrier offset.
    Uses the first n_fft samples of the burst, the rest is rotated with the same estimate."""
    z = np.abs(np.fft.fftshift(np.fft.fft(seg * seg, n_fft)))
    c = np.arange(-search, search + 1)
    score = z[n_fft // 2 + c - delta // 2] + z[n_fft // 2 + c + delta // 2]
    f0 = c[np.argmax(score)] / (2.0 * n_fft)                # cycles per sample
    return seg * np.exp(-2j * np.pi * f0 * np.arange(len(seg)))


def coherent_bits(u, delay=3, weight=0.85):
    """u: one complex sample per symbol. Returns the raw frequency bit stream, before NRZI decoding."""
    n = len(u)
    if n < 20:
        return None
    v = u * _ROT[np.arange(n) % 4]                          # derotate 90 degrees per symbol, symbols now lie on a line
    t = v.real[None, :] * _COS + v.imag[None, :] * _SIN     # projection on each candidate axis, shape (16, n)
    ma = lfilter([1 - weight], [1, -weight], np.abs(t), axis=1)
    pos = t > 0
    sel = np.empty(n, dtype=np.int64)
    cur = 0
    for i in range(n):                                      # follow the best axis, moving at most one step per symbol
        a, b, c = (cur - 1) % 16, cur, (cur + 1) % 16
        best, bv = a, ma[a, i]
        if ma[b, i] > bv:
            best, bv = b, ma[b, i]
        if ma[c, i] > bv:
            best = c
        cur = best
        sel[i] = cur
    idx = np.arange(delay + 1, n)
    return pos[sel[idx], idx - delay] ^ pos[sel[idx], idx - delay - 1]


def decode_burst(seg):
    """seg: complex samples at 48 kHz covering one burst. Returns the payload bit lists of every valid frame (bits as
    transmitted, use fix_order before reading fields)."""
    s = np.convolve(correct_offset(seg), GAUSS, mode="same")
    found = {}
    for p in range(5):                                      # the five possible symbol timings
        raw = coherent_bits(s[p::5])
        if raw is None:
            continue
        nrzi = (raw[1:] == raw[:-1]).astype(np.uint8).tolist()
        for fr in hdlc_frames(nrzi):
            if 56 <= len(fr) <= 1100 and crc_ok(fr):
                payload = fr[:-16]
                found[bytes(payload)] = payload
    return list(found.values())


def find_bursts(s):
    env = np.convolve(np.abs(s) ** 2, np.ones(8) / 8, mode="same")
    thr = np.percentile(env, 30) * 3.0
    on = (env > thr).astype(np.int8)
    edges = np.flatnonzero(np.diff(np.concatenate(([0], on, [0]))))
    segs = []
    for a, b in zip(edges[0::2], edges[1::2]):
        if segs and a - segs[-1][1] <= 20:
            segs[-1][1] = b
        else:
            segs.append([a, b])
    return [(max(a - 30, 0), min(b + 30, len(s)), float(env[a:b].max())) for a, b in segs if b - a >= 400]


_lo = {}
# Narrow channel filter (about 12 kHz wide) so the HackRF's centre DC spike, 25 kHz away, is rejected
_taps = firwin(1001, 12500, window=("kaiser", 7.0), fs=FS)


def process(buf, now):
    """buf: bytes of interleaved int8 IQ. Decodes every complete AIS burst in it."""
    x = np.frombuffer(buf, dtype=np.int8).astype(np.float32).view(np.complex64)
    x = x - x.mean()
    for off in CHANNELS:
        key = (off, len(x))
        if key not in _lo:
            _lo.clear()
            _lo[key] = np.exp(-2j * np.pi * off / FS * np.arange(len(x))).astype(np.complex64)
        s = resample_poly(x * _lo[key], 1, DEC, window=_taps)
        for a, b, peak in find_bursts(s):
            frames = decode_burst(s[a:b])
            stats["bursts"] += 1
            if not frames:
                stats["bad_crc"] += 1
            for payload in frames:
                key2 = bytes(payload)
                if key2 in recent and now - recent[key2] < 2.0:
                    continue
                recent[key2] = now
                parse_message(fix_order(payload), round(10 * math.log10(max(peak, 1e-6)), 1), now)
    for k in [k for k, t in recent.items() if now - t > 10]:
        del recent[k]


def decoder_loop(source, is_file):
    tail = b""
    paced = time.time()
    while True:
        raw = source.read(CHUNK * 2)
        if not raw or len(raw) < CHUNK * 2:
            stats["status"] = "replay finished" if is_file else "HackRF stopped"
            return
        t0 = time.time()
        d = np.frombuffer(raw, dtype=np.int8)
        stats["peak_adc"] = max(int(d.max()), -int(d.min()))
        buf = tail + raw
        if len(tail) < OVERLAP * 2:                        # first chunk: pad so the length is constant
            buf = bytes(OVERLAP * 2 - len(tail)) + buf
        process(buf, t0)
        tail = buf[-OVERLAP * 2:]
        stats["load"] = round(100 * (time.time() - t0) / (CHUNK / FS), 1)
        stats["status"] = "replaying" if is_file else "receiving"
        if is_file:
            paced += CHUNK / FS
            time.sleep(max(0.0, paced - time.time()))


# ---------------------------------------------------------------- web page and server
PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>HackRF AIS</title>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.css">
<style>
:root{--bg:#fff;--fg:#1c1e21;--mut:#6b7280;--line:#e5e7eb;--panel:#f7f8fa;--warn:#b45309}
@media(prefers-color-scheme:dark){:root{--bg:#15181c;--fg:#e6e8eb;--mut:#9aa3ad;--line:#2a2f36;--panel:#1b1f24;--warn:#fbbf24}}
*{box-sizing:border-box}body{margin:0;font:14px system-ui,sans-serif;background:var(--bg);color:var(--fg);display:flex;flex-direction:column;height:100vh}
header{padding:8px 14px;border-bottom:1px solid var(--line);display:flex;gap:16px;flex-wrap:wrap;align-items:center;background:var(--panel)}
header b{font-size:15px}header span{color:var(--mut)}header .warn{color:var(--warn);font-weight:600}
.key i{display:inline-block;width:10px;height:10px;border-radius:50%;margin:0 4px 0 10px;vertical-align:-1px}
main{flex:1;display:flex;min-height:0}#map{flex:1;min-width:0}
aside{width:420px;max-width:45%;overflow:auto;border-left:1px solid var(--line)}
table{width:100%;border-collapse:collapse}th,td{padding:5px 8px;text-align:right;white-space:nowrap}
th{position:sticky;top:0;background:var(--panel);color:var(--mut);font-weight:600;border-bottom:1px solid var(--line)}
td:first-child,th:first-child{text-align:left;max-width:170px;overflow:hidden;text-overflow:ellipsis}tr{cursor:pointer}tr:hover{background:var(--panel)}tr.stale{opacity:.5}
.trail{filter:drop-shadow(0 0 2px #fff) drop-shadow(0 0 1px #fff)}
.lbl{background:transparent;border:0;box-shadow:none;font-weight:600;font-size:11px;color:#111;text-shadow:0 0 3px #fff,0 0 3px #fff}
button{font:inherit;padding:3px 10px;border:1px solid var(--line);background:var(--bg);color:var(--fg);border-radius:6px;cursor:pointer}
@media(max-width:700px){main{flex-direction:column}aside{width:100%;max-width:none;height:40%;border-left:0;border-top:1px solid var(--line)}}
</style></head><body>
<header><b>HackRF AIS</b><span id="s1"></span><span id="s2"></span><span id="s3"></span><button id="fit">Fit all</button>
<span class="key" id="key"></span></header>
<main><div id="map"></div><aside><table><thead><tr><th>Vessel</th><th>Type</th><th>kn</th><th>Course</th><th>Msgs</th></tr></thead><tbody id="rows"></tbody></table></aside></main>
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.js"></script>
<script>
const map=L.map('map').setView([45.6,-122.7],9);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:18,attribution:'&copy; OpenStreetMap contributors'}).addTo(map);
const CAT={cargo:'#16a34a',tanker:'#dc2626',passenger:'#2563eb',tug:'#f59e0b',fishing:'#ea580c',pleasure:'#06b6d4',military:'#7c3aed',other:'#6b7280',base:'#111827',aton:'#a16207'};
const NAMES={cargo:'Cargo',tanker:'Tanker',passenger:'Passenger',tug:'Tug or tow',fishing:'Fishing',pleasure:'Pleasure',other:'Other'};
document.getElementById('key').innerHTML=Object.keys(NAMES).map(k=>`<i style="background:${CAT[k]}"></i>${NAMES[k]}`).join('');
const marks={},trails={};let fitted=false,home=null;
function icon(v){const c=CAT[v.cat]||CAT.other;
 if(v.kind!=='ship')return L.divIcon({className:'',iconSize:[12,12],iconAnchor:[6,6],html:`<div style="width:10px;height:10px;background:${c};transform:rotate(45deg);border:1px solid #fff"></div>`});
 if((v.sog||0)<0.5)return L.divIcon({className:'',iconSize:[12,12],iconAnchor:[6,6],html:`<div style="width:12px;height:12px;background:${c};border-radius:50%;border:2px solid #fff"></div>`});
 const r=v.hdg!=null?v.hdg:(v.cog||0);
 return L.divIcon({className:'',iconSize:[16,20],iconAnchor:[8,10],html:`<div style="width:0;height:0;border-left:8px solid transparent;border-right:8px solid transparent;border-bottom:20px solid ${c};transform:rotate(${r}deg);filter:drop-shadow(0 0 1px #000)"></div>`})}
function fitAll(){const p=Object.values(marks).map(m=>m.getLatLng());if(home)p.push(home);if(p.length)map.fitBounds(L.latLngBounds(p).pad(.2),{maxZoom:13})}
document.getElementById('fit').onclick=fitAll;
async function tick(){
 let j;try{j=await (await fetch('vessels.json',{cache:'no-store'})).json()}catch(e){return}
 if(j.receiver&&!home){home=L.latLng(j.receiver);L.circleMarker(home,{radius:6,color:'#2563eb',fillOpacity:.8}).addTo(map).bindTooltip('Receiver');map.setView(home,10)}
 const seen=new Set(),rows=document.getElementById('rows');rows.textContent='';
 const list=j.vessels.sort((x,y)=>x.age-y.age);let withPos=0;
 for(const a of list){
  const name=a.name||('MMSI '+a.mmsi);seen.add(a.mmsi);
  if(a.lat!=null){withPos++;const ll=[a.lat,a.lon];
   if(!marks[a.mmsi]){marks[a.mmsi]=L.marker(ll,{icon:icon(a)}).addTo(map).bindTooltip(name,{permanent:true,direction:'right',offset:[8,0],className:'lbl'});
    trails[a.mmsi]=L.polyline(a.trail,{color:'#ff00ff',weight:3,opacity:1,className:'trail'}).addTo(map);if(!fitted){fitted=true;fitAll()}}
   else{marks[a.mmsi].setLatLng(ll).setIcon(icon(a)).setTooltipContent(name);trails[a.mmsi].setLatLngs(a.trail)}
   marks[a.mmsi].setOpacity(a.age>120?.4:1)}
  const tr=document.createElement('tr');if(a.age>120)tr.className='stale';tr.title=(a.dest?'To '+a.dest+'. ':'')+'MMSI '+a.mmsi;
  [name,a.type_name,a.sog==null?'-':a.sog,a.cog==null?'-':a.cog,a.msgs].forEach(v=>{const td=document.createElement('td');td.textContent=v;tr.appendChild(td)});
  tr.onclick=()=>{if(marks[a.mmsi])map.setView(marks[a.mmsi].getLatLng(),Math.max(map.getZoom(),12))};rows.appendChild(tr)}
 for(const k of Object.keys(marks))if(!seen.has(+k)){map.removeLayer(marks[k]);map.removeLayer(trails[k]);delete marks[k];delete trails[k]}
 document.getElementById('s1').textContent=`${list.length} vessels, ${withPos} on map`;
 document.getElementById('s2').textContent=`${j.msgs} messages decoded, ${j.bursts} radio bursts heard (${j.bad_crc} not decodable), status: ${j.status}`;
 const s3=document.getElementById('s3');s3.textContent=`ADC peak ${j.peak_adc}/127, CPU load ${j.load}%`+(j.peak_adc>110?' (near clipping, lower the gain)':'');
 s3.className=j.peak_adc>110||j.load>90?'warn':'';
 if(j.error)s3.textContent+=' | '+j.error}
setInterval(tick,1000);tick();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            body, ctype = PAGE.encode(), "text/html; charset=utf-8"
        elif path == "/vessels.json":
            now = time.time()
            with lock:
                for k in [k for k, a in vessels.items() if now - a["last"] > EXPIRE_S]:
                    del vessels[k]
                while msg_times and now - msg_times[0] > 30:
                    msg_times.popleft()
                out = []
                for a in vessels.values():
                    tn, cat = ship_type(a["type"])
                    if a["kind"] == "base":
                        tn, cat = "Base station", "base"
                    elif a["kind"] == "aton":
                        tn, cat = "Navigation aid", "aton"
                    d = {k: v for k, v in a.items() if k not in ("last", "type")}
                    d.update(type_name=tn, cat=cat, age=round(now - a["last"], 1))
                    out.append(d)
                data = {"vessels": out, "msg_rate": len(msg_times) / 30, "receiver": receiver, **stats}
            body, ctype = json.dumps(data).encode(), "application/json"
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


def main():
    global receiver
    ap = argparse.ArgumentParser(description="Live AIS ship map for a HackRF")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--lna", type=int, default=32, help="LNA gain 0-40 dB, steps of 8 (default 32)")
    ap.add_argument("--vga", type=int, default=30, help="VGA gain 0-62 dB, steps of 2 (default 30)")
    ap.add_argument("--amp", type=int, default=0, choices=(0, 1),
                    help="HackRF RF amplifier (default 0, strong VHF signals overload it)")
    ap.add_argument("--lat", type=float, help="your latitude, to mark the receiver on the map")
    ap.add_argument("--lon", type=float, help="your longitude")
    ap.add_argument("--file", help="replay a 2.4 Msps int8 IQ capture instead of using the HackRF")
    args = ap.parse_args()
    if args.lat is not None and args.lon is not None:
        receiver = [args.lat, args.lon]

    proc = None
    if args.file:
        source = open(args.file, "rb")
    else:
        cmd = ["hackrf_transfer", "-r", "-", "-f", str(CENTER), "-s", str(FS), "-b", "1750000",
               "-l", str(args.lna), "-g", str(args.vga), "-a", str(args.amp)]
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except FileNotFoundError:
            sys.exit("hackrf_transfer not found. Install it with: brew install hackrf")
        errs = collections.deque(maxlen=5)

        def drain():
            for line in proc.stderr:
                errs.append(line.decode(errors="replace").strip())
                if "error" in errs[-1].lower() or "not found" in errs[-1].lower():
                    stats["error"] = errs[-1]
        threading.Thread(target=drain, daemon=True).start()
        source = proc.stdout

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"Open http://127.0.0.1:{args.port}   (Ctrl-C to stop)")
    try:
        decoder_loop(source, bool(args.file))
        if args.file:
            while True:
                time.sleep(1)
        elif proc:
            sys.exit("hackrf_transfer stopped: " + (stats["error"] or "is the HackRF connected and not in use?"))
    except KeyboardInterrupt:
        pass
    finally:
        if proc:
            proc.terminate()


if __name__ == "__main__":
    main()

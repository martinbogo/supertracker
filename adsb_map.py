#!/usr/bin/env python3
"""Live ADS-B map for a HackRF.

Needs: hackrf_transfer on PATH (brew install hackrf) and numpy.
Run:   python3 adsb_map.py            then open http://127.0.0.1:8080
Test:  python3 adsb_map.py --file capture.bin   (replays a 4 Msps int8 IQ file)

The map uses Leaflet and OpenStreetMap tiles, so the browser needs internet
for the background map. Decoding itself is fully local.
"""
import argparse
import collections
import json
import math
import os
import re
import socket
import sqlite3
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

from adsb_config import Config, ConfigError          # settings in a private file, secrets in the macOS Keychain

try:                                  # optional: filed flight plans from the FAA SWIM feed, see swim_routes.py
    from swim_routes import Nav, Plans, route_fits
except Exception:                     # solace library or navigation data not available, the map just shows no routes
    Nav = Plans = route_fits = None

FS = 4_000_000                       # samples per second (4 samples per microsecond)
CHUNK = 1_000_000                    # samples per processing step (0.25 s)
OVERLAP = 32 + 4 * 112 + 8           # preamble + longest message, carried between chunks
CHARSET = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######"
PULSE = np.array([0, 1, 4, 5, 14, 15, 18, 19])               # preamble pulse samples
GAP = np.array([2, 3] + list(range(6, 14)) + list(range(20, 28)))  # preamble quiet samples
EXPIRE_S = 120

aircraft = {}
stats = {"msgs": 0, "peak_adc": 0, "noise": 0.0, "load": 0.0, "status": "starting", "error": ""}
msg_times = collections.deque()
lock = threading.Lock()
receiver = None
plans = None                                  # set in main() when the SWIM feed is enabled
RETRY_MSG = "HackRF not connected, retrying every few seconds"
OFF_ROUTE_KM = 250                           # a plan whose line passes farther than this from the aircraft is not drawn
HERE = os.path.dirname(os.path.abspath(__file__))
tracks = None                                 # set in main(), the position history
flightlog = None                              # set in main(), the permanent log of flights
sbs = None                                    # set in main(), the local SBS output
cfg = None                                    # set in main(), the settings (see adsb_config.py)
uplink = None                                 # set in main(), the optional upload to ADSBHub
hub = None                                    # set in main(), the optional download from ADSBHub
radio = {"proc": None}                        # the running hackrf_transfer, so a settings change can restart it


class Tracks:
    """Position history of every aircraft, kept for 30 days by default (--keep-days). It is stored in SQLite, so a restart
    does not lose it and an aircraft that comes back (a daily flight, say) still has its earlier positions."""
    KEEP_S = 30 * 24 * 3600
    GAP_S = 600                               # a longer silence starts a new segment, so a flight that left and came back is
    MIN_STEP_S = 4                            # not joined by a straight line. One stored position per aircraft every 4 s.

    def __init__(self, path):
        self.path = path
        os.makedirs(os.path.dirname(path), exist_ok=True)
        db = sqlite3.connect(path)
        db.execute("PRAGMA journal_mode=WAL")
        db.execute("CREATE TABLE IF NOT EXISTS pos (icao TEXT, ts INTEGER, lat REAL, lon REAL, alt INTEGER)")
        db.execute("CREATE INDEX IF NOT EXISTS pos_icao_ts ON pos (icao, ts)")
        db.commit()
        db.close()
        self.queue = collections.deque()
        self.last = {}
        threading.Thread(target=self._writer, daemon=True).start()

    def add(self, icao, ts, lat, lon, alt):
        if ts - self.last.get(icao, 0) < self.MIN_STEP_S:
            return
        self.last[icao] = ts
        self.queue.append((icao, int(ts), lat, lon, alt))

    def _writer(self):
        db = sqlite3.connect(self.path)
        purged = 0.0
        while True:
            time.sleep(3)
            rows = []
            while self.queue:
                rows.append(self.queue.popleft())
            try:
                if rows:
                    db.executemany("INSERT INTO pos VALUES (?,?,?,?,?)", rows)
                if time.time() - purged > 3600:
                    db.execute("DELETE FROM pos WHERE ts < ?", (int(time.time()) - self.KEEP_S,))
                    purged = time.time()
                db.commit()
            except Exception as e:
                print("position history write failed:", type(e).__name__, e)

    def read(self, icao, max_points=3000, hours=24):
        """Positions of the last `hours` hours (24 by default, at most what is kept) as a list of segments, each a list
        of [lat, lon, alt, time]."""
        window = min(hours * 3600, self.KEEP_S)
        db = sqlite3.connect(self.path)
        try:
            rows = db.execute("SELECT ts, lat, lon, alt FROM pos WHERE icao=? AND ts>? ORDER BY ts",
                              (icao, int(time.time()) - window)).fetchall()
        finally:
            db.close()
        rows += [r[1:] for r in list(self.queue) if r[0] == icao]           # not yet written to disk
        if len(rows) > max_points:
            step = len(rows) / max_points
            rows = [rows[int(i * step)] for i in range(max_points)] + [rows[-1]]
        segs, cur = [], []
        for ts, lat, lon, alt in rows:
            if cur and ts - cur[-1][3] > self.GAP_S:
                segs.append(cur)
                cur = []
            cur.append([round(lat, 5), round(lon, 5), alt, ts])
        if cur:
            segs.append(cur)
        return segs


class FlightLog:
    """A permanent log of the flights we have received: aircraft, callsign, route, type, registration, flight rules, and when
    it was first and last seen. A new row starts when an aircraft returns after a 3 hour silence or changes callsign."""
    VISIT_GAP_S = 3 * 3600

    def __init__(self, path):
        self.path = path
        db = sqlite3.connect(path)
        db.execute("""CREATE TABLE IF NOT EXISTS flights (id INTEGER PRIMARY KEY AUTOINCREMENT, icao TEXT, callsign TEXT,
                      reg TEXT, type TEXT, dep TEXT, arr TEXT, rules TEXT, route TEXT, first_seen INTEGER,
                      last_seen INTEGER, max_alt INTEGER)""")
        db.execute("CREATE INDEX IF NOT EXISTS flights_seen ON flights (last_seen)")
        db.execute("CREATE INDEX IF NOT EXISTS flights_icao ON flights (icao, last_seen)")
        empty = db.execute("SELECT COUNT(*) FROM flights").fetchone()[0] == 0
        db.commit()
        db.close()
        if empty:
            self._backfill()
        self.open = {}
        db = sqlite3.connect(path)
        for fid, icao, cs, last in db.execute("SELECT id, icao, callsign, last_seen FROM flights WHERE last_seen>? ORDER BY last_seen",
                                              (int(time.time()) - self.VISIT_GAP_S,)):
            self.open[icao] = {"id": fid, "cs": cs, "last": last}
        db.close()
        threading.Thread(target=self._run, daemon=True).start()

    def _backfill(self):
        """On first use, rebuild the log from the position history and the flight plans already known."""
        info = {}
        if plans is not None:
            for r in list(plans.by_gufi.values()):
                if r.get("hex"):
                    info[r["hex"]] = r
        positions = sqlite3.connect(tracks.path)
        rows = positions.execute("SELECT icao, ts, alt FROM pos ORDER BY icao, ts").fetchall()
        positions.close()
        out, visit = [], None

        def flush(v):
            if v:
                p = info.get(v["icao"]) or {}
                out.append((v["icao"], p.get("cs"), p.get("reg"), p.get("type"), p.get("dep"), p.get("arr"), p.get("rules"),
                            p.get("route"), v["first"], v["last"], v["alt"]))
        for icao, ts, alt in rows:
            if visit and visit["icao"] == icao and ts - visit["last"] <= self.VISIT_GAP_S:
                visit["last"] = ts
                visit["alt"] = max(visit["alt"] or 0, alt or 0)
            else:
                flush(visit)
                visit = {"icao": icao, "first": ts, "last": ts, "alt": alt}
        flush(visit)
        db = sqlite3.connect(self.path)
        db.executemany("INSERT INTO flights (icao, callsign, reg, type, dep, arr, rules, route, first_seen, last_seen, max_alt)"
                       " VALUES (?,?,?,?,?,?,?,?,?,?,?)", out)
        db.commit()
        db.close()
        print("flight log started from the position history: %d flights" % len(out))

    def _run(self):
        db = sqlite3.connect(self.path)
        while True:
            time.sleep(5)
            try:
                self._update(db)
            except Exception as e:
                print("flight log update failed:", type(e).__name__, e)

    def _update(self, db):
        now = time.time()
        with lock:
            snap = [(a["icao"], a["cs"], a["alt"], a["lat"], a["last"]) for a in aircraft.values()]
        if hub is not None and cfg.get("hub", "enabled"):
            snap += hub.nearby()                                    # shared flights near us are logged like our own
        for icao, cs, alt, lat, last in snap:
            if lat is None or now - last > 30:                      # only aircraft heard recently, with a position
                continue
            rec = plans.get(icao, cs) if plans is not None else None
            cur = self.open.get(icao)
            if cur and (now - cur["last"] > self.VISIT_GAP_S or (cs and cur["cs"] and cs != cur["cs"])):
                cur = None
            if cur is None:
                fid = db.execute("INSERT INTO flights (icao, first_seen, last_seen) VALUES (?,?,?)",
                                 (icao, int(last), int(last))).lastrowid
                cur = self.open[icao] = {"id": fid, "cs": None, "last": last}
            cur["last"] = last
            if cs:
                cur["cs"] = cs
            p = rec or {}
            db.execute("UPDATE flights SET last_seen=?, callsign=COALESCE(?,callsign), reg=COALESCE(?,reg), type=COALESCE(?,type),"
                       " dep=COALESCE(?,dep), arr=COALESCE(?,arr), rules=COALESCE(?,rules), route=COALESCE(?,route),"
                       " max_alt=MAX(COALESCE(max_alt,0), COALESCE(?,0)) WHERE id=?",
                       (int(last), cs, p.get("reg"), p.get("type"), p.get("dep"), p.get("arr"), p.get("rules"), p.get("route"),
                        alt, cur["id"]))
        db.commit()

    def query(self, days, limit=5000):
        db = sqlite3.connect(self.path)
        db.row_factory = sqlite3.Row
        try:
            rows = db.execute("SELECT icao, callsign, reg, type, dep, arr, rules, route, first_seen, last_seen, max_alt FROM flights"
                              " WHERE last_seen>? ORDER BY last_seen DESC LIMIT ?",
                              (int(time.time() - days * 86400), limit)).fetchall()
        finally:
            db.close()
        return [dict(r) for r in rows]


def _hav_km(la1, lo1, la2, lo2):
    p1, p2 = math.radians(la1), math.radians(la2)
    h = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lo2 - lo1) / 2) ** 2
    return 12742.0 * math.asin(min(1.0, math.sqrt(h)))


class SbsOut:
    """SBS (BaseStation, port 30003 style) text output of the aircraft we decode, for tools such as Virtual Radar Server.

    It listens on 127.0.0.1 only, so nothing leaves this computer. Every line passes sanity checks first (valid address,
    callsign characters, position inside radio range, no impossible jumps, believable altitude and speed), and what is
    rejected is counted and kept for review at /sbs.json."""
    MAX_RANGE_KM = 650            # an ADS-B receiver cannot hear much beyond this
    MAX_JUMP_KT = 800             # implied speed between two positions
    MAX_GS_KT = 900
    ALT_MIN, ALT_MAX, MAX_FPM = -1500, 60000, 20000

    def __init__(self, port):
        self.port = port
        self.clients = []
        self.queue = collections.deque(maxlen=20000)
        self.sent = 0
        self.rejects = collections.Counter()
        self.recent = collections.deque(maxlen=40)
        self.last = {}                                       # icao -> last accepted position {ts, lat, lon, alt}
        self.bad_run = collections.Counter()
        self.share_on = False                                # only true while the upload to ADSBHub is connected
        self.share_queue = collections.deque(maxlen=20000)
        self.server = socket.socket()
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("127.0.0.1", port))
        self.server.listen(5)
        threading.Thread(target=self._accept, daemon=True).start()
        threading.Thread(target=self._send, daemon=True).start()

    def _accept(self):
        while True:
            conn, _ = self.server.accept()
            conn.settimeout(2)
            self.clients.append(conn)

    def _send(self):
        while True:
            time.sleep(0.05)
            if not self.queue:
                continue
            data = b""
            while self.queue:
                data += self.queue.popleft()
            for c in list(self.clients):
                try:
                    c.sendall(data)
                except OSError:
                    self.clients.remove(c)
                    c.close()

    def _reject(self, reason, icao, detail=""):
        self.rejects[reason] += 1
        self.recent.append("%s %s %s %s" % (time.strftime("%H:%M:%S"), icao, reason, detail))

    @staticmethod
    def _stamp(ts):
        t = time.gmtime(ts)                                  # UTC, so feeds from different places can be merged
        d = time.strftime("%Y/%m/%d", t)
        return d, time.strftime("%H:%M:%S", t) + ".%03d" % int((ts % 1) * 1000)

    def _line(self, kind, icao, ts, callsign="", alt="", gs="", track="", lat="", lon="", vr=""):
        d, t = self._stamp(ts)
        line = ("MSG,%d,1,1,%s,1,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,,,,,\r\n" % (
            kind, icao, d, t, d, t, callsign, alt, gs, track, lat, lon, vr)).encode()
        self.queue.append(line)
        if self.share_on:                                    # the same checked lines, for the upload when it is switched on
            self.share_queue.append(line)
        self.sent += 1

    def _icao_ok(self, icao):
        if not re.fullmatch(r"[0-9A-F]{6}", icao or "") or icao in ("000000", "FFFFFF"):
            self._reject("bad address", icao or "?")
            return False
        return True

    def callsign(self, icao, ts, cs):
        if not self._icao_ok(icao):
            return
        if not re.fullmatch(r"[A-Z0-9]{1,8}", cs or ""):
            self._reject("bad callsign", icao, repr(cs))
            return
        self._line(1, icao, ts, callsign=cs)

    def position(self, icao, ts, lat, lon, alt):
        if not self._icao_ok(icao):
            return
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            self._reject("position out of range", icao, "%s,%s" % (lat, lon))
            return
        if receiver and _hav_km(receiver[0], receiver[1], lat, lon) > self.MAX_RANGE_KM:
            self._reject("beyond %d km from the receiver" % self.MAX_RANGE_KM, icao, "%.3f,%.3f" % (lat, lon))
            return
        if alt is not None and not (self.ALT_MIN <= alt <= self.ALT_MAX):
            self._reject("altitude out of range", icao, str(alt))
            alt = None
        prev = self.last.get(icao)
        if prev and ts > prev["ts"]:
            dt = ts - prev["ts"]
            nm = _hav_km(prev["lat"], prev["lon"], lat, lon) / 1.852
            kt = nm / (dt / 3600.0) if dt >= 1 else (0 if nm < 0.6 else 99999)
            fpm = abs((alt or 0) - (prev["alt"] or 0)) / (dt / 60.0) if (alt is not None and prev["alt"] is not None and dt >= 1) else 0
            if kt > self.MAX_JUMP_KT or fpm > self.MAX_FPM:
                self.bad_run[icao] += 1
                self._reject("position jump" if kt > self.MAX_JUMP_KT else "altitude jump", icao,
                             "%.0f kt, %.0f ft/min over %.1f s" % (kt, fpm, dt))
                if self.bad_run[icao] < 3:                    # three in a row means our baseline was the bad one, start again
                    return
        self.bad_run[icao] = 0
        self.last[icao] = {"ts": ts, "lat": lat, "lon": lon, "alt": alt}
        self._line(3, icao, ts, alt="" if alt is None else alt, lat="%.5f" % lat, lon="%.5f" % lon)

    def velocity(self, icao, ts, gs, track, vr):
        if not self._icao_ok(icao):
            return
        if gs is None or track is None or not (0 <= gs <= self.MAX_GS_KT and 0 <= track < 360):
            self._reject("speed or heading out of range", icao, "%s kt, %s deg" % (gs, track))
            return
        if vr is not None and abs(vr) > 10000:
            self._reject("climb rate out of range", icao, str(vr))
            vr = None
        self._line(4, icao, ts, gs=gs, track=track, vr="" if vr is None else vr)

    def status(self):
        return {"port": self.port, "clients": len(self.clients), "sent": self.sent,
                "rejected": sum(self.rejects.values()), "reasons": dict(self.rejects.most_common(8))}


class HubUplink:
    """Uploads the checked SBS lines to a shared feed (ADSBHub, data.adsbhub.org port 5001).

    It does nothing, and opens no connection, unless the sharing switch in Settings is on. Turning the switch off closes
    the connection straight away. ADSBHub recognizes the connection by the IP address registered for the station."""

    def __init__(self, source):
        self.source = source                                 # the SbsOut whose checked lines are sent
        self.state = "off"
        self.sent = 0
        self.error = ""
        self.since = None
        threading.Thread(target=self._run, daemon=True).start()

    def _target(self):
        return cfg.get("share", "host"), cfg.get("share", "port")

    def _enabled(self):
        return cfg.get("share", "enabled")

    def _run(self):
        wait = 3
        while True:
            if not self._enabled():
                self.state, self.error, self.since = "off", "", None
                self.source.share_on = False
                self.source.share_queue.clear()
                wait = 3
                time.sleep(1)
                continue
            target = self._target()
            self.state = "connecting"
            try:
                conn = socket.create_connection(target, timeout=10)
            except OSError as e:
                self.state, self.error = "error", "%s: %s" % (type(e).__name__, e)
                time.sleep(wait)
                wait = min(60, wait * 2)
                continue
            self.source.share_queue.clear()
            self.source.share_on = True
            self.state, self.error, self.since, wait = "sharing", "", time.time(), 3
            try:
                while self._enabled() and self._target() == target:
                    if self.source.share_queue:
                        data = b""
                        while self.source.share_queue:
                            data += self.source.share_queue.popleft()
                        conn.sendall(data)
                        self.sent += data.count(b"\n")
                    else:
                        time.sleep(0.2)
            except OSError as e:
                self.state, self.error = "error", "%s: %s" % (type(e).__name__, e)
                time.sleep(wait)
            finally:
                self.source.share_on = False
                try:
                    conn.close()
                except OSError:
                    pass

    def status(self):
        return {"enabled": bool(self._enabled()), "state": self.state, "sent": self.sent, "error": self.error,
                "host": cfg.get("share", "host"), "port": cfg.get("share", "port")}


class HubDownlink:
    """Receives other people's aircraft from a shared SBS feed (ADSBHub, data.adsbhub.org port 5002).

    It does nothing, and opens no connection, unless the switch in Settings is on. It only reads: nothing received here is ever
    passed to the local SBS output or to the upload, so shared data cannot be sent back. ADSBHub recognizes this computer by its
    IP address. Every line is treated as untrusted and checked field by field before it is kept."""
    EXPIRE_S = 90
    MAX_AIRCRAFT = 8000
    MAX_LINE = 400
    KEEP_STEP_S = 30                       # one stored position per nearby aircraft every 30 s, to keep the history small

    def __init__(self):
        self.ac = {}
        self.lock = threading.Lock()
        self.state, self.error, self.since = "off", "", None
        self.lines = self.rejected = 0
        self.last_data = None
        self.reasons = {}
        threading.Thread(target=self._run, daemon=True).start()

    def _target(self):
        return cfg.get("hub", "host"), cfg.get("hub", "port")

    def _enabled(self):
        return cfg.get("hub", "enabled")

    def _run(self):
        wait = 3
        while True:
            if not self._enabled():
                self.state, self.error, self.since = "off", "", None
                with self.lock:
                    self.ac.clear()
                wait = 3
                time.sleep(1)
                continue
            target = self._target()
            self.state = "connecting"
            try:
                conn = socket.create_connection(target, timeout=10)
                conn.settimeout(1)                                # so switching off is noticed within a second
            except OSError as e:
                self.state, self.error = "error", "%s: %s" % (type(e).__name__, e)
                time.sleep(wait)
                wait = min(60, wait * 2)
                continue
            self.state, self.since = "connected, waiting for data", time.time()          # the last error stays visible until data arrives
            buf, got = b"", False
            try:
                while self._enabled() and self._target() == target:
                    try:
                        data = conn.recv(65536)
                    except socket.timeout:
                        if self.last_data is None or time.time() - self.last_data > 60:
                            if self.state == "receiving":
                                self.state = "connected, no data for a minute"
                        continue
                    if not data:
                        self.error = "the server closed the connection" + ("" if got else " without sending anything (is this IP authorized, and are you feeding?)")
                        self.state = "error"
                        break
                    buf += data
                    if len(buf) > 1 << 20:                       # never let a stream without line breaks grow without bound
                        buf = b""
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self._line(line)
                        got = True
                    if got and self.state != "receiving":
                        self.state, self.error, wait = "receiving", "", 3
            except OSError as e:
                self.state, self.error = "error", "%s: %s" % (type(e).__name__, e)
            finally:
                try:
                    conn.close()
                except OSError:
                    pass
            if self._enabled():
                time.sleep(wait)
                wait = min(60, wait * 2)

    def _reject(self, why):
        self.rejected += 1
        self.reasons[why] = self.reasons.get(why, 0) + 1

    def _line(self, raw, now=None):
        now = now or time.time()
        if len(raw) > self.MAX_LINE:
            self._reject("line too long")
            return
        f = raw.decode("latin-1").strip().split(",")
        if len(f) < 17 or f[0] != "MSG":
            self._reject("not an SBS MSG line")
            return
        kind, icao = f[1], f[4].strip().upper()
        if kind not in ("1", "2", "3", "4", "5") or not re.fullmatch(r"[0-9A-F]{6}", icao) or icao in ("000000", "FFFFFF"):
            if kind in ("1", "2", "3", "4", "5"):
                self._reject("bad address")
            return
        try:
            upd = {}
            if kind == "1":
                cs = f[10].strip()
                if not re.fullmatch(r"[A-Z0-9]{1,8}", cs):
                    self._reject("bad callsign")
                    return
                upd["cs"] = cs
            if kind in ("2", "3", "5") and f[11].strip() != "":
                alt = float(f[11])
                if not -1500 <= alt <= 60000:
                    self._reject("altitude out of range")
                    return
                upd["alt"] = alt
            if kind in ("2", "3"):
                lat, lon = float(f[14]), float(f[15])
                if not (-90 <= lat <= 90 and -180 <= lon <= 180) or (lat == 0 and lon == 0):
                    self._reject("position out of range")
                    return
                upd["lat"], upd["lon"] = lat, lon
            if kind in ("2", "4"):
                if f[12].strip() != "":
                    gs = float(f[12])
                    if not 0 <= gs <= 1000:
                        self._reject("speed out of range")
                        return
                    upd["gs"] = gs
                if f[13].strip() != "":
                    trk = float(f[13])
                    if not 0 <= trk <= 360:
                        self._reject("heading out of range")
                        return
                    upd["trk"] = trk % 360
            if kind == "4" and f[16].strip() != "":
                vr = float(f[16])
                if abs(vr) > 10000:
                    self._reject("climb rate out of range")
                    return
        except ValueError:
            self._reject("a number could not be read")
            return
        with self.lock:
            a = self.ac.get(icao)
            if a is None:
                if len(self.ac) >= self.MAX_AIRCRAFT:
                    self._reject("too many aircraft")
                    return
                a = self.ac[icao] = {"icao": icao}
            if "lat" in upd and a.get("lat") is not None:
                nm = _hav_km(a["lat"], a["lon"], upd["lat"], upd["lon"]) / 1.852
                if nm > 5 and nm / (max(now - a["t"], 1.0) / 3600.0) > 1000:      # feeders differ slightly, so only a real jump counts
                    self._reject("position jump over 1000 kt")
                    return
            a.update(upd)
            if "lat" in upd:
                a["t"] = now
                rad = cfg.get("hub", "radius_mi")                    # 0 means the whole world
                a["near"] = rad == 0 or (bool(receiver) and _hav_km(receiver[0], receiver[1], upd["lat"], upd["lon"]) <= rad * 1.609344)
                if a["near"] and tracks is not None and icao not in aircraft and now - a.get("kept", 0) >= self.KEEP_STEP_S:
                    a["kept"] = now                                  # near us: kept in the position history like our own aircraft
                    tracks.add(icao, now, upd["lat"], upd["lon"], a.get("alt"))
            a.setdefault("t", now)
            a["seen"] = now
        self.lines += 1
        self.last_data = now

    def snapshot(self, south, west, north, east, exclude=(), limit=2500):
        """Aircraft with a position inside the box (west may be greater than east across the date line)."""
        now = time.time()
        out = []
        with self.lock:
            for k in [k for k, a in self.ac.items() if now - a["seen"] > self.EXPIRE_S]:
                del self.ac[k]
            for near_pass in (True, False):                          # aircraft inside the radius first, so a world view never drops them
                for k, a in self.ac.items():
                    if bool(a.get("near")) != near_pass or a.get("lat") is None or k in exclude or not south <= a["lat"] <= north:
                        continue
                    if (west <= a["lon"] <= east) if west <= east else (a["lon"] >= west or a["lon"] <= east):
                        out.append({"icao": k, "cs": a.get("cs"), "lat": a["lat"], "lon": a["lon"], "alt": a.get("alt"),
                                    "gs": a.get("gs"), "trk": a.get("trk"), "age": round(now - a["seen"]), "near": near_pass})
                        if len(out) >= limit:
                            break
                if len(out) >= limit:
                    break
            total = len(self.ac)
        return out, total

    def get(self, icao):
        with self.lock:
            a = self.ac.get(icao)
            return dict(a) if a else None

    def nearby(self):
        """Recently heard aircraft inside the radius, for the flight log: (icao, callsign, altitude, latitude, time)."""
        now = time.time()
        with self.lock:
            return [(k, a.get("cs"), a.get("alt"), a["lat"], a["seen"]) for k, a in self.ac.items()
                    if a.get("near") and a.get("lat") is not None and now - a["seen"] < 30 and k not in aircraft]

    def status(self):
        with self.lock:
            n = len(self.ac)
        return {"enabled": bool(self._enabled()), "state": self.state, "error": self.error, "aircraft": n, "lines": self.lines,
                "rejected": self.rejected, "reasons": dict(self.reasons), "host": cfg.get("hub", "host"), "port": cfg.get("hub", "port"),
                "radius_mi": cfg.get("hub", "radius_mi"),
                "age": round(time.time() - self.last_data) if self.last_data else None}


def crc24(value, nbits):
    reg = 0
    for i in range(nbits - 1, -1, -1):
        reg = (reg << 1) | ((value >> i) & 1)
        if reg & 0x1000000:
            reg ^= 0x1FFF409
    return reg


def nl(lat):
    if abs(lat) < 1e-9:
        return 59
    if abs(lat) == 87:
        return 2
    if abs(lat) > 87:
        return 1
    a = 1 - math.cos(math.pi / 30)
    b = math.cos(math.pi / 180 * abs(lat)) ** 2
    return int(math.floor(2 * math.pi / math.acos(1 - a / b)))


def cpr_global(even, odd, even_is_latest):
    la0, lo0 = even[0] / 131072, even[1] / 131072
    la1, lo1 = odd[0] / 131072, odd[1] / 131072
    j = math.floor(59 * la0 - 60 * la1 + 0.5)
    lat0 = 360 / 60 * ((j % 60) + la0)
    lat1 = 360 / 59 * ((j % 59) + la1)
    if lat0 >= 270:
        lat0 -= 360
    if lat1 >= 270:
        lat1 -= 360
    if nl(lat0) != nl(lat1):
        return None
    if even_is_latest:
        lat, n = lat0, nl(lat0)
        ni = max(n, 1)
        m = math.floor(lo0 * (n - 1) - lo1 * n + 0.5)
        lon = 360 / ni * ((m % ni) + lo0)
    else:
        lat, n = lat1, nl(lat1)
        ni = max(n - 1, 1)
        m = math.floor(lo0 * (n - 1) - lo1 * n + 0.5)
        lon = 360 / ni * ((m % ni) + lo1)
    if lon >= 180:
        lon -= 360
    return lat, lon


def cpr_local(cpr, odd, ref_lat, ref_lon):
    la, lo = cpr[0] / 131072, cpr[1] / 131072
    dlat = 360 / (59 if odd else 60)
    j = math.floor(ref_lat / dlat) + math.floor(0.5 + (ref_lat % dlat) / dlat - la)
    lat = dlat * (j + la)
    ni = max(nl(lat) - (1 if odd else 0), 1)
    dlon = 360 / ni
    m = math.floor(ref_lon / dlon) + math.floor(0.5 + (ref_lon % dlon) / dlon - lo)
    lon = dlon * (m + lo)
    if lon >= 180:
        lon -= 360
    return lat, lon


def handle_message(df, value, nbits, rssi, now):
    icao = (value >> (nbits - 32)) & 0xFFFFFF
    with lock:
        a = aircraft.get(icao)
        if a is None:
            if df == 11 and crc24(value, nbits) != 0:
                return
            a = aircraft[icao] = {"icao": "%06X" % icao, "cs": None, "alt": None, "spd": None,
                                  "hdg": None, "vr": None, "lat": None, "lon": None, "trail": [],
                                  "msgs": 0, "first": now, "last": now, "rssi": 0.0,
                                  "even": None, "odd": None}
        a["msgs"] += 1
        a["last"] = now
        a["rssi"] = rssi
        stats["msgs"] += 1
        msg_times.append(now)
        if df == 11:
            return
        me = (value >> 24) & ((1 << 56) - 1)
        tc = me >> 51
        if 1 <= tc <= 4:
            s = me & ((1 << 48) - 1)
            a["cs"] = "".join(CHARSET[(s >> (42 - 6 * i)) & 63] for i in range(8)).strip("#_ ") or None
            if sbs is not None and a["cs"]:
                sbs.callsign(a["icao"], now, a["cs"])
        elif 9 <= tc <= 18 or 20 <= tc <= 22:
            raw = (me >> 36) & 0xFFF
            if 9 <= tc <= 18 and (raw >> 4) & 1:
                a["alt"] = (((raw >> 5) << 4) | (raw & 0xF)) * 25 - 1000
            odd = (me >> 34) & 1
            cpr = ((me >> 17) & 0x1FFFF, me & 0x1FFFF)
            a["odd" if odd else "even"] = (cpr, now)
            pos = None
            if a["even"] and a["odd"] and abs(a["even"][1] - a["odd"][1]) < 10:
                pos = cpr_global(a["even"][0], a["odd"][0], a["even"][1] > a["odd"][1])
            elif a["lat"] is not None:
                pos = cpr_local(cpr, odd, a["lat"], a["lon"])
            if pos and abs(pos[0]) <= 90:
                if a["lat"] is None or (abs(pos[0] - a["lat"]) < 1.0 and abs(pos[1] - a["lon"]) < 1.0):
                    a["lat"], a["lon"] = round(pos[0], 5), round(pos[1], 5)
                    if tracks is not None:
                        tracks.add(a["icao"], now, a["lat"], a["lon"], a["alt"])
                    if sbs is not None:
                        sbs.position(a["icao"], now, a["lat"], a["lon"], a["alt"])
                    if not a["trail"] or a["trail"][-1] != [a["lat"], a["lon"]]:
                        a["trail"].append([a["lat"], a["lon"]])
                        del a["trail"][:-40]
        elif tc == 19 and ((me >> 48) & 7) in (1, 2):
            sub = (me >> 48) & 7
            vx = ((me >> 32) & 0x3FF) - 1
            vy = ((me >> 21) & 0x3FF) - 1
            if (me >> 42) & 1:
                vx = -vx
            if (me >> 31) & 1:
                vy = -vy
            a["spd"] = round(math.hypot(vx, vy) * (4 if sub == 2 else 1))
            a["hdg"] = round((math.degrees(math.atan2(vx, vy)) + 360) % 360)
            vr = ((me >> 10) & 0x1FF)
            if vr:
                a["vr"] = (vr - 1) * 64 * (-1 if (me >> 19) & 1 else 1)
            if sbs is not None:
                sbs.velocity(a["icao"], now, a["spd"], a["hdg"], a["vr"])


def decode_chunk(m, now):
    """m: float32 magnitude array. Decodes every message that starts in m[:len(m)-OVERLAP]."""
    lc = len(m) - OVERLAP
    if lc <= 0:
        return
    p = sum(m[k:k + lc] for k in PULSE) / 8
    g = sum(m[k:k + lc] for k in GAP) / len(GAP)
    noise = float(np.median(m[::16]))
    stats["noise"] = noise
    cand = np.flatnonzero((p > 2.0 * g) & (p > 3.0 * noise))
    if not len(cand):
        return
    keep = np.ones(len(cand), dtype=bool)               # keep only local maxima of the score
    for o in (-3, -2, -1, 1, 2, 3):
        nb = np.clip(cand + o, 0, lc - 1)
        keep &= p[cand] >= p[nb]
    cand = cand[keep]
    idx = cand[:, None] + 32 + 4 * np.arange(112)[None, :]
    bits = (m[idx] + m[idx + 1]) > (m[idx + 2] + m[idx + 3])
    df = bits[:, 0] * 16 + bits[:, 1] * 8 + bits[:, 2] * 4 + bits[:, 3] * 2 + bits[:, 4] * 1
    sel = np.flatnonzero(np.isin(df, (11, 17, 18)))
    if not len(sel):
        return
    packed = np.packbits(bits[sel], axis=1)
    for row, c in zip(packed, cand[sel]):
        full = int.from_bytes(row.tobytes(), "big")
        d = full >> 107                                  # top 5 bits of the 112 bit window
        if d == 11:
            value, nbits = full >> 56, 56
            if crc24(value, nbits) >= 80:
                continue
        else:
            value, nbits = full, 112
            if crc24(value, nbits) != 0:
                continue
        handle_message(d, value, nbits, round(20 * math.log10(max(float(p[c]), 1e-3)), 1), now)


def decoder_loop(source, is_file):
    tail = np.zeros(0, dtype=np.float32)
    paced = time.time()
    while True:
        raw = source.read(CHUNK * 2)
        if not raw or len(raw) < CHUNK * 2:
            if is_file:
                stats["status"] = "replay finished"
                return
            stats["status"] = "HackRF stopped"
            return
        t0 = time.time()
        d = np.frombuffer(raw, dtype=np.int8)
        stats["peak_adc"] = max(int(d.max()), -int(d.min()))
        m = np.hypot(d[0::2].astype(np.float32), d[1::2].astype(np.float32))
        buf = np.concatenate((tail, m))
        decode_chunk(buf, t0)
        tail = buf[len(buf) - OVERLAP:]
        busy = time.time() - t0
        stats["load"] = round(100 * busy / (CHUNK / FS), 1)
        stats["status"] = "receiving" if not is_file else "replaying"
        if stats["error"] == RETRY_MSG:
            stats["error"] = ""
        if is_file:                                   # replay at real-time speed
            paced += CHUNK / FS
            time.sleep(max(0.0, paced - time.time()))


PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>HackRF ADS-B</title>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.css">
<style>
:root{--bg:#fff;--fg:#1c1e21;--mut:#6b7280;--line:#e5e7eb;--panel:#f7f8fa;--warn:#b45309}
@media(prefers-color-scheme:dark){:root{--bg:#15181c;--fg:#e6e8eb;--mut:#9aa3ad;--line:#2a2f36;--panel:#1b1f24;--warn:#fbbf24}}
*{box-sizing:border-box}body{margin:0;font:14px system-ui,sans-serif;background:var(--bg);color:var(--fg);display:flex;flex-direction:column;height:100vh}
header{padding:8px 14px;border-bottom:1px solid var(--line);display:flex;gap:18px;flex-wrap:wrap;align-items:center;background:var(--panel)}
header b{font-size:15px}header span{color:var(--mut)}header .warn{color:var(--warn);font-weight:600}
main{flex:1;display:flex;min-height:0}#map{flex:1;min-width:0}
aside{position:relative;width:560px;max-width:55%;border-left:1px solid var(--line);display:flex;flex-direction:column;min-height:0}
aside.closed{width:0;border-left:0}aside.closed>:not(#sidetab){display:none}
#sidetab{position:absolute;top:50%;left:-23px;transform:translateY(-50%);z-index:1000;width:23px;height:64px;padding:0;border:1px solid var(--line);border-right:0;border-radius:9px 0 0 9px;background:var(--panel);color:var(--mut);font-size:20px;line-height:1;cursor:pointer;box-shadow:-2px 0 6px rgba(0,0,0,.15)}
#sidetab:hover,#sidetab:focus-visible{color:var(--fg);background:var(--bg)}
#tools{display:flex;gap:6px;align-items:center;padding:6px 8px;border-bottom:1px solid var(--line);background:var(--panel)}
#q,#hist{font:inherit;padding:4px 8px;border:1px solid var(--line);border-radius:6px;background:var(--bg);color:var(--fg)}
#q{flex:1;min-width:0}#hist{padding:3px 4px}#cnt{color:var(--mut);white-space:nowrap;font-size:12px}
#tbl{flex:1;overflow:auto}
table{width:100%;border-collapse:collapse}th,td{padding:5px 8px;text-align:right;white-space:nowrap}
th{position:sticky;top:0;background:var(--panel);color:var(--mut);font-weight:600;border-bottom:1px solid var(--line)}
td:first-child,th:first-child{text-align:left}tr{cursor:pointer}tr:hover{background:var(--panel)}tr.stale{opacity:.5}tr.shared td:first-child{border-left:3px solid #7aa5d1}tr.old td{color:var(--mut)}
th.s{cursor:pointer;user-select:none}th.s:hover,th.s.on{color:var(--fg)}th .pr{font-size:9px;vertical-align:super}th.s:focus-visible{outline:2px solid #2563eb;outline-offset:-2px}
.ac{width:0;height:0;border-left:7px solid transparent;border-right:7px solid transparent;border-bottom:18px solid #888;filter:drop-shadow(0 0 1px #000)}
.trail{filter:drop-shadow(0 0 2px #fff) drop-shadow(0 0 1px #fff)}
.plan{filter:drop-shadow(0 0 1.5px #00394a)}
td:nth-child(2),th:nth-child(2){text-align:left}header button.on{border-color:#00b8d4;color:#00b8d4}
tr.sel{background:rgba(255,0,255,.16)}.ac.sel{filter:drop-shadow(0 0 3px #f0f) drop-shadow(0 0 3px #f0f)}
.rules-i{color:#0891b2;font-weight:600}.rules-v{color:#b45309;font-weight:600}#selbtn{display:none;border-color:#f0f;color:#f0f}
.lbl{background:transparent;border:0;box-shadow:none;font-weight:600;font-size:11px;color:#111;text-shadow:0 0 3px #fff,0 0 3px #fff}
dialog{border:1px solid var(--line);border-radius:10px;background:var(--bg);color:var(--fg);padding:0;width:min(640px,94vw);max-height:90vh}
dialog::backdrop{background:rgba(0,0,0,.45)}#setform{padding:16px 20px;overflow:auto;max-height:88vh}
#setform h2{margin:0 0 4px}#setform h3{margin:18px 0 6px;font-size:14px;border-top:1px solid var(--line);padding-top:12px}
#setform label{display:flex;gap:8px;align-items:center;margin:5px 0}#setform label span{width:150px;color:var(--mut);flex:none}
#setform input[type=text],#setform input[type=number],#setform input[type=password],#setform select{flex:1;min-width:0;font:inherit;padding:4px 8px;border:1px solid var(--line);border-radius:6px;background:var(--bg);color:var(--fg)}
#setform small{color:var(--mut);display:block;margin:2px 0 6px}#dlgmsg{margin:8px 0;padding:8px;border-radius:6px;display:none;white-space:pre-line}
#dlgmsg.ok{display:block;background:rgba(22,163,74,.14)}#dlgmsg.err{display:block;background:rgba(220,38,38,.14)}
#setform .row{display:flex;gap:8px;justify-content:flex-end;margin-top:16px}.warnbox{border:1px solid #dc2626;border-radius:8px;padding:8px 10px;margin:8px 0}
button.danger{border-color:#dc2626;color:#dc2626}
button{font:inherit;padding:3px 10px;border:1px solid var(--line);background:var(--bg);color:var(--fg);border-radius:6px;cursor:pointer}
@media(max-width:700px){main{flex-direction:column}aside{width:100%;max-width:none;height:45%;border-left:0;border-top:1px solid var(--line)}}
</style></head><body>
<header><b>HackRF ADS-B</b><span id="s1"></span><span id="s2"></span><span id="s3"></span><button id="fit">Fit all</button><button id="rt" title="Show or hide the filed flight plan routes">Routes</button><button id="selbtn" title="Click a flight to see its recorded track. Shift-click to select several. Esc or a click on the map clears.">Selected</button><button id="gear" title="Settings">Settings</button><span id="shr"></span><span id="hubb"></span></header>
<main><div id="map"></div><aside id="side"><button id="sidetab" aria-controls="side" aria-expanded="true"></button>
<div id="tools"><input id="q" type="search" placeholder="Search callsign, registration, type, airport... (try intl, overseas, IFR, VFR, live)" title="Press / to search. Words are combined, so 'intl 787' finds international 787s. intl = a non-US airport, overseas = outside North America."><select id="hist" title="Also list previous flights from the permanent log"><option value="0">Live only</option><option value="1">+ last 24 h</option><option value="7">+ 7 days</option><option value="30">+ 30 days</option><option value="3650">+ all</option></select><label id="hublw" style="display:none;white-space:nowrap;font-size:12px" title="Also list the aircraft received from the shared feed that are in the map view. Typing the word shared in the search box does the same."><input type="checkbox" id="hubl"> Shared</label><span id="cnt"></span></div>
<div id="tbl"><table><thead><tr><th class="s" data-k="name" data-l="Flight" tabindex="0" title="Click to sort: ascending, descending, off. Shift-click adds another sort column.">Flight</th><th class="s" data-k="route" data-l="Route" tabindex="0" title="Filed route, origin to destination, from the FAA SWIM feed. Hover a row for the full route text.">Route</th><th class="s" data-k="rules" data-l="Rules" tabindex="0" title="IFR or VFR, from the filed flight plan.">Rules</th><th class="s" data-k="alt" data-l="Alt ft" tabindex="0" title="Altitude. For previous flights, the highest altitude seen.">Alt ft</th><th class="s" data-k="spd" data-l="kt" tabindex="0" title="Ground speed in knots">kt</th><th class="s" data-k="hdg" data-l="Hdg" tabindex="0" title="Heading">Hdg</th><th class="s" data-k="msgs" data-l="Msgs" tabindex="0" title="Messages received from this aircraft">Msgs</th><th class="s" data-k="last" data-l="Last seen" tabindex="0" title="When we last received this aircraft. Today shows the time, other days show the date and time.">Last seen</th></tr></thead><tbody id="rows"></tbody></table></div>
</aside></main>
<dialog id="dlg"><form id="setform" onsubmit="return false">
<h2>Settings</h2><small>Secrets are kept in the macOS Keychain. They are never shown here, never saved in the settings file, and never sent back to this page.</small>
<div id="dlgmsg"></div>
<h3>Sharing with ADSBHub</h3>
<label><input type="checkbox" id="f_share"><b>Share my aircraft with ADSBHub</b></label>
<small id="shareinfo"></small>
<label><span>Server</span><input type="text" id="f_shost"></label><label><span>Port</span><input type="number" id="f_sport" min="1" max="65535"></label>
<label><span>Station key</span><input type="password" id="s_adsbhub_ckey" autocomplete="off"><label style="margin:0"><input type="checkbox" id="r_adsbhub_ckey">remove</label></label>
<small>Sharing sends position, altitude, callsign, speed and heading of every aircraft this receiver decodes to the server above. ADSBHub recognizes the connection by the IP address registered for your station, so keep that up to date on their site.</small>
<h3>Receiving from ADSBHub</h3>
<label><input type="checkbox" id="f_hub"><b>Show aircraft from ADSBHub on the map</b></label>
<small id="hubinfo"></small>
<label><span>Server</span><input type="text" id="f_hhost"></label><label><span>Port</span><input type="number" id="f_hport" min="1" max="65535"></label>
<label><span>Track within</span><select id="f_hrad"><option value="10">10 miles</option><option value="20">20 miles</option><option value="50">50 miles</option><option value="100">100 miles</option><option value="200">200 miles</option><option value="500">500 miles</option><option value="1000">1000 miles</option><option value="2000">2000 miles</option><option value="0">The whole world</option></select></label>
<small>Reads the shared feed (port 5002). ADSBHub recognizes this computer by its IP address, so that IP must be listed under Data Access IPs on their site. Aircraft within the chosen distance of your receiver get a track, their filed route is drawn, and they are saved in your position history and flight log (about 30 seconds apart) like your own. Farther ones are only shown as dots and are never saved. Aircraft your own radio hears always come from your radio. Nothing received here is put on the local SBS output or sent back.</small>
<h3>Receiver</h3>
<label><span>Latitude</span><input type="number" id="f_lat" step="any"></label><label><span>Longitude</span><input type="number" id="f_lon" step="any"></label>
<label><span>LNA gain (dB)</span><select id="f_lna"><option>0</option><option>8</option><option>16</option><option>24</option><option>32</option><option>40</option></select></label>
<label><span>VGA gain (dB)</span><input type="number" id="f_vga" min="0" max="62" step="2"></label>
<label><span>RF amplifier</span><input type="checkbox" id="f_amp" style="flex:none"></label>
<small>Changing the gain restarts the radio for a few seconds. The amplifier adds about 14 dB and can overload on strong signals.</small>
<h3>Position history</h3>
<label><span>Keep days</span><input type="number" id="f_keep" min="1" max="3650"></label>
<h3>Filed flight plans (FAA SWIM)</h3>
<label><input type="checkbox" id="f_routes"><b>Show filed routes</b></label><small id="swiminfo"></small>
<label><span>Server URL</span><input type="text" id="f_swurl" placeholder="tcps://host:port"></label><label><span>Message VPN</span><input type="text" id="f_swvpn"></label>
<label><span>User name</span><input type="password" id="s_swim_user" autocomplete="off"><label style="margin:0"><input type="checkbox" id="r_swim_user">remove</label></label>
<label><span>Password</span><input type="password" id="s_swim_password" autocomplete="off"><label style="margin:0"><input type="checkbox" id="r_swim_password">remove</label></label>
<label><span>Queue name</span><input type="password" id="s_swim_queue" autocomplete="off"><label style="margin:0"><input type="checkbox" id="r_swim_queue">remove</label></label>
<h3>Local SBS output</h3>
<label><span>Port</span><input type="number" id="f_sbs" min="0" max="65535"></label><small id="sbsinfo"></small>
<div id="legacybox" class="warnbox" style="display:none"><b>Plain text credential files found</b><small>~/.swim_credentials and ~/.adsbhub_credentials hold secrets in readable text. Import them into the Keychain, check that everything works, then delete the files.</small>
<div class="row" style="margin:0"><button id="b_import">Import into Keychain</button><button id="b_delete" class="danger">Delete the plain text files</button></div></div>
<div class="row"><button id="b_close">Close</button><button id="b_save" style="font-weight:600">Save</button></div>
</form></dialog>
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.js"></script>
<script>
const store=(k,v)=>{try{localStorage.setItem(k,v)}catch(e){}};
const load=(k,d)=>{try{const v=localStorage.getItem(k);return v===null?d:v}catch(e){return d}};
const map=L.map('map').setView([39,-98],4);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:18,attribution:'&copy; OpenStreetMap contributors'}).addTo(map);
const GREEN='#00ff2a',CASING='#063d12';                           // tracks: magenta for aircraft our radio hears, bright green for shared ones
function greenLine(pts,w){const a=L.polyline(pts,{color:CASING,weight:w+3,opacity:.55,className:'trail',interactive:false}),b=L.polyline(pts,{color:GREEN,weight:w,opacity:.95,className:'trail',interactive:false}),g=L.layerGroup([a,b]);
 g.setLatLngs=l=>{a.setLatLngs(l);b.setLatLngs(l)};g.setStyle=o=>{a.setStyle({opacity:o.opacity?.55:0});b.setStyle({opacity:o.opacity})};return g}
const marks={},trails={};let fitted=false,home=null;
const sel=new Set(),selLayers={},selHours={},fitPending=new Set();   // selected aircraft (icao), their track layers, hours of history to show
const col=a=>a==null?'#888':`hsl(${Math.max(0,Math.min(1,a/40000))*240},85%,50%)`;
function icon(a){return L.divIcon({className:'',iconSize:[14,18],iconAnchor:[7,9],
 html:`<div class="ac${sel.has(a.icao)?' sel':''}" style="transform:rotate(${a.hdg||0}deg);border-bottom-color:${col(a.alt)}"></div>`})}
const routeLines={};let showRoutes=load('adsbRoutes','on')!=='off';
function fitAll(){const p=Object.values(marks).map(m=>m.getLatLng());if(home)p.push(home);  // aircraft markers only, never routes or tracks
 if(p.length)map.fitBounds(L.latLngBounds(p).pad(.2),{maxZoom:11})}
document.getElementById('fit').onclick=fitAll;
const rtBtn=document.getElementById('rt');
function drawRt(){rtBtn.textContent='Routes: '+(showRoutes?'on':'off');rtBtn.className=showRoutes?'on':''}
rtBtn.onclick=()=>{showRoutes=!showRoutes;store('adsbRoutes',showRoutes?'on':'off');drawRt();if(lastJ)render(lastJ);if(hubOn)hubTick()};
drawRt();
// The flight list can be hidden to give the map the whole window
let showList=load('adsbList','on')!=='off';const lstBtn=document.getElementById('sidetab'),side=document.getElementById('side');
function drawList(){side.classList.toggle('closed',!showList);lstBtn.textContent=showList?'\u203a':'\u2039';lstBtn.setAttribute('aria-expanded',showList);
 const t=showList?'Hide the flight list':'Show the flight list';lstBtn.title=t;lstBtn.setAttribute('aria-label',t);setTimeout(()=>map.invalidateSize(),60)}
lstBtn.onclick=()=>{showList=!showList;store('adsbList',showList?'on':'off');drawList()};
drawList();
const apt=s=>s?(s.length===4&&s[0]==='K'?s.slice(1):s):'?';
function rtStyle(est){return{color:'#00d8f0',weight:2.5,opacity:est?.7:.95,dashArray:est?'2 8':'9 7',className:'plan',interactive:false}}
// a third value of 1 marks a leg that is only an estimate (US text joined to a foreign airport): dotted, not dashed
function mkRoute(pl){const g=L.layerGroup();let cur=[pl[0]],est=false;
 for(let i=1;i<pl.length;i++){const e=pl[i].length>2;
  if(e!==est&&cur.length>1){g.addLayer(L.polyline(cur,rtStyle(est)));cur=[pl[i-1]]}
  est=e;cur.push(pl[i])}
 if(cur.length>1)g.addLayer(L.polyline(cur,rtStyle(est)));return g}
function dropRoute(k){if(routeLines[k]){map.removeLayer(routeLines[k].l);delete routeLines[k]}}
// ---- rows: the live aircraft plus, optionally, previous flights from the log
let histDays=+load('adsbHist','1'),hist=[];
const histSel=document.getElementById('hist');histSel.value=String(histDays);
async function loadHist(){if(!histDays){hist=[];return}try{hist=await (await fetch('flights.json?days='+histDays,{cache:'no-store'})).json()}catch(e){}}
histSel.onchange=async()=>{histDays=+histSel.value;store('adsbHist',histDays);await loadHist();if(lastJ)render(lastJ)};
setInterval(async()=>{await loadHist();if(lastJ)render(lastJ)},20000);
const nowTs=()=>Date.now()/1000;
function rowsFor(j,withHub){const t=nowTs(),rows=[],live=new Set();
 for(const a of j.aircraft){live.add(a.icao);const p=a.plan;
  rows.push({live:true,icao:a.icao,name:a.cs||a.icao,reg:p&&p.reg,type:p&&p.type,dep:p&&p.dep,arr:p&&p.arr,rules:p&&p.rules,route:p&&p.route,off:p&&p.off,
   alt:a.alt,spd:a.spd,hdg:a.hdg,msgs:a.msgs,last:t-a.age,age:a.age})}
 for(const f of hist){if(live.has(f.icao)&&t-f.last_seen<120)continue;                 // that one is the live row
  rows.push({live:false,icao:f.icao,name:f.callsign||f.icao,reg:f.reg,type:f.type,dep:f.dep,arr:f.arr,rules:f.rules,route:f.route,
   alt:f.max_alt,spd:null,hdg:null,msgs:null,last:f.last_seen,first:f.first_seen})}
 if(withHub&&hubOn){const cap=qEl.value.trim()?5000:600;let n=0;                   // shared feed aircraft in view, after our own
  for(const a of hubList){if(live.has(a.icao))continue;if(n++>=cap)break;const p=a.plan;
   rows.push({live:true,hub:true,icao:a.icao,name:a.cs||a.icao,reg:p&&p.reg,type:p&&p.type,dep:p&&p.dep,arr:p&&p.arr,rules:p&&p.rules,route:p&&p.route,
    alt:a.alt,spd:a.gs==null?null:Math.round(a.gs),hdg:a.trk==null?null:Math.round(a.trk),msgs:null,last:t-a.age,age:a.age})}}
 return rows}
const US=/^(K|PA|PH|PG|PF|PO|PP|PM|PW)/;
const icao4=c=>c&&c.length===4;                                  // 4 letter ICAO codes only, FAA codes like S05 are small US airfields
const isIntl=r=>[r.dep,r.arr].some(c=>icao4(c)&&!US.test(c));
const isFar=r=>[r.dep,r.arr].some(c=>icao4(c)&&!'KCMPT'.includes(c[0]));
const AKA=[[/^B73[1-9]$|^B3[789]M$/,'737'],[/^B74\d$/,'747'],[/^B75[23]$/,'757'],[/^B76\d$/,'767'],[/^B77[2-9LW]$/,'777'],[/^B78[89X]$/,'787'],
 [/^A19N$/,'a319'],[/^A20N$/,'a320'],[/^A21N$/,'a321'],[/^A33[239]$/,'a330'],[/^A34\d$/,'a340'],[/^A35[9K]$/,'a350'],[/^A388$/,'a380'],
 [/^BCS[13]$/,'a220'],[/^E(17|19|29|75)\w$|^E\d{3}$/,'embraer'],[/^CRJ\w$/,'crj bombardier'],[/^DH8\w$/,'dash 8 q400'],
 [/^C172$/,'cessna 172'],[/^PC12$/,'pilatus pc12'],[/^SR2\d$/,'cirrus']];
const aka=t=>{t=(t||'').toUpperCase();const m=AKA.find(x=>x[0].test(t));return m?m[1]:''};
function match(r,t){
 if(t==='intl')return !!isIntl(r);if(t==='overseas')return !!isFar(r);if(t==='live')return r.live;if(t==='shared')return !!r.hub;if(t==='local'||t==='ours')return r.live&&!r.hub;if(t==='previous'||t==='history')return !r.live;
 return [r.name,r.icao,r.reg,r.type,aka(r.type),r.dep,r.arr,r.route,r.rules].join(' ').toLowerCase().includes(t)}
const qEl=document.getElementById('q'),hublEl=document.getElementById('hubl');hublEl.checked=load('adsbHubList','off')==='on';
hublEl.onchange=()=>{store('adsbHubList',hublEl.checked?'on':'off');if(lastJ)render(lastJ)};qEl.oninput=()=>{if(lastJ)render(lastJ)};
function fmtTime(ts){const d=new Date(ts*1000),n=new Date();
 const hm=d.toLocaleTimeString([],{hour:'2-digit',minute:'2-digit',second:'2-digit'});
 return d.toDateString()===n.toDateString()?hm:d.toLocaleDateString([],{month:'short',day:'numeric'})+' '+hm.slice(0,5)}
// ---- sorting
let sorts=[];try{sorts=JSON.parse(load('adsbSort','[]'))}catch(e){sorts=[]}
const val={name:r=>r.name,route:r=>(r.dep||r.arr)?apt(r.dep)+'>'+apt(r.arr):null,rules:r=>r.rules||null,alt:r=>r.alt,spd:r=>r.spd,hdg:r=>r.hdg,msgs:r=>r.msgs,last:r=>r.last};
sorts=(Array.isArray(sorts)?sorts:[]).filter(s=>s&&val[s.k]&&(s.d==='asc'||s.d==='desc'));
function cmp(x,y){for(const s of sorts){const a=val[s.k](x),b=val[s.k](y);if(a==null&&b==null)continue;if(a==null)return 1;if(b==null)return -1;
 const c=typeof a==='string'?a.localeCompare(b):a-b;if(c)return s.d==='asc'?c:-c}return (x.hub?1:0)-(y.hub?1:0)||y.last-x.last}
function drawHeads(){for(const th of document.querySelectorAll('th.s')){const i=sorts.findIndex(s=>s.k===th.dataset.k);
 th.textContent=th.dataset.l;th.classList.toggle('on',i>=0);th.setAttribute('aria-sort',i<0?'none':sorts[i].d==='asc'?'ascending':'descending');
 if(i>=0){th.append(' '+(sorts[i].d==='asc'?'▲':'▼'));if(sorts.length>1){const p=document.createElement('span');p.className='pr';p.textContent=i+1;th.appendChild(p)}}}}
function setSort(k,multi){const i=sorts.findIndex(s=>s.k===k);
 if(multi){if(i<0)sorts.push({k,d:'asc'});else if(sorts[i].d==='asc')sorts[i].d='desc';else sorts.splice(i,1)}
 else if(i>=0&&sorts.length===1){if(sorts[0].d==='asc')sorts[0].d='desc';else sorts=[]}
 else sorts=[{k,d:'asc'}];
 store('adsbSort',JSON.stringify(sorts));drawHeads();if(lastJ)render(lastJ)}
for(const th of document.querySelectorAll('th.s')){th.onclick=e=>setSort(th.dataset.k,e.shiftKey);th.onkeydown=e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();setSort(th.dataset.k,e.shiftKey)}}}
drawHeads();
// ---- selection: click a flight to show everything recorded for it. Shift, Ctrl or Cmd adds to the selection.
function select(k,multi,pan,row){
 const wasSel=sel.has(k);
 if(multi){if(wasSel)sel.delete(k);else sel.add(k)}
 else if(sel.size===1&&wasSel)sel.clear();
 else{sel.clear();sel.add(k)}
 if(sel.has(k)&&!wasSel){
  if(row&&!row.live){selHours[k]=Math.max(24,Math.ceil((nowTs()-row.first)/3600)+1);fitPending.add(k)}   // a previous flight: show its whole visit
  else selHours[k]=24;
  if(pan&&!multi&&(marks[k]||hubMk[k]))map.setView((marks[k]||hubMk[k]).getLatLng(),Math.max(map.getZoom(),9))}
 syncSel()}
const selBtn=document.getElementById('selbtn');
function drawSel(){selBtn.style.display=sel.size?'inline-block':'none';selBtn.textContent='Selected: '+sel.size+' (clear)'}
selBtn.onclick=()=>{sel.clear();syncSel()};
map.on('click',()=>{if(sel.size){sel.clear();syncSel()}});
document.addEventListener('keydown',e=>{
 if(e.key==='/'&&document.activeElement!==qEl){e.preventDefault();if(!showList){showList=true;store('adsbList','on');drawList()}qEl.focus();return}
 if(e.key==='Escape'){if(document.activeElement===qEl){qEl.value='';qEl.blur();if(lastJ)render(lastJ)}else if(sel.size){sel.clear();syncSel()}}});
function syncSel(){drawSel();for(const k of Object.keys(hubMk)){if(hubMk[k]._near){if(hubTrLine[k])hubTrLine[k].setStyle({opacity:sel.has(k)?0:.95})}else hubMk[k].setStyle(hubStyle(k))}
 for(const k of Object.keys(selLayers))if(!sel.has(k)){map.removeLayer(selLayers[k]);delete selLayers[k]}
 for(const k of sel)loadTrack(k);
 if(lastJ)render(lastJ)}
function fitSel(){let b=null;for(const g of Object.values(selLayers))g.eachLayer(l=>{
  const bb=l.getBounds?l.getBounds():(l.getLatLng?L.latLngBounds([l.getLatLng()]):null);
  if(bb&&bb.isValid())b=b?b.extend(bb):L.latLngBounds(bb.getSouthWest(),bb.getNorthEast())});
 if(b&&b.isValid())map.fitBounds(b.pad(.15),{maxZoom:11})}
async function loadTrack(k){
 let d;try{d=await (await fetch('tracks.json?icao='+k+'&hours='+(selHours[k]||24),{cache:'no-store'})).json()}catch(e){return}
 if(!sel.has(k))return;const segs=d[k]||[];
 if(selLayers[k])map.removeLayer(selLayers[k]);
 const g=L.layerGroup(),shared=!marks[k]&&!!hubMk[k];
 for(const s of segs){
  if(s.length>1){if(shared)greenLine(s.map(p=>[p[0],p[1]]),3.5).addTo(g);else L.polyline(s.map(p=>[p[0],p[1]]),{color:'#ff00ff',weight:3.5,opacity:.95,className:'trail',interactive:false}).addTo(g)}
  const f=s[0];L.circleMarker([f[0],f[1]],{radius:3.5,color:shared?GREEN:'#ff00ff',weight:2,fillColor:'#fff',fillOpacity:1})
   .bindTooltip(new Date(f[3]*1000).toLocaleString(undefined,{month:'short',day:'numeric',hour:'2-digit',minute:'2-digit'})+', first recorded here').addTo(g)}
 g.addTo(map);selLayers[k]=g;
 if(fitPending.has(k)){fitPending.delete(k);fitSel()}}
setInterval(()=>{for(const k of sel)loadTrack(k)},10000);
drawSel();
// ---- settings dialog
const dlg=document.getElementById('dlg'),$=id=>document.getElementById(id);
const SECRETS=['swim_user','swim_password','swim_queue','adsbhub_ckey'];
function note(text,ok){const m=$('dlgmsg');m.textContent=text;m.className=text?(ok?'ok':'err'):''}
let lastRad=100;
async function loadSettings(){
 let r;try{r=await (await fetch('config.json',{cache:'no-store'})).json()}catch(e){note('Could not read the settings: '+e,false);return}
 const c=r.config;
 $('f_hub').checked=c.hub.enabled;$('f_hhost').value=c.hub.host;$('f_hport').value=c.hub.port;$('f_hrad').value=String(c.hub.radius_mi);lastRad=c.hub.radius_mi;$('f_share').checked=c.share.enabled;$('f_shost').value=c.share.host;$('f_sport').value=c.share.port;
 $('f_lat').value=c.receiver.lat===null?'':c.receiver.lat;$('f_lon').value=c.receiver.lon===null?'':c.receiver.lon;
 $('f_lna').value=c.radio.lna;$('f_vga').value=c.radio.vga;$('f_amp').checked=!!c.radio.amp;$('f_keep').value=c.history.keep_days;
 $('f_routes').checked=c.routes.enabled;$('f_swurl').value=c.swim.url;$('f_swvpn').value=c.swim.vpn;$('f_sbs').value=c.sbs.port;
 for(const n of SECRETS){const st=c.secrets[n].stored,i=$('s_'+n);i.value='';i.placeholder=st?'Stored in Keychain, type to replace':'Not set';
  $('r_'+n).checked=false;$('r_'+n).parentElement.style.display=st?'flex':'none'}
 const sh=r.share;$('shareinfo').textContent=sh?'Status: '+(sh.enabled?sh.state:'off')+(sh.enabled?', '+sh.sent+' lines sent':'')+(sh.error?' ('+sh.error+')':''):'The SBS output is off, so sharing is not available.';
 const hb=r.hub;$('hubinfo').textContent=hb?'Status: '+(hb.enabled?hb.state:'off')+(hb.enabled&&hb.aircraft?', '+hb.aircraft+' aircraft':'')+(hb.rejected?', '+hb.rejected+' lines rejected':'')+(hb.error?' ('+hb.error+')':''):'';
 $('swiminfo').textContent=r.swim?'Status: '+r.swim.state+(r.swim.state==='connected'?', '+r.swim.plans+' plans':''):'Not started.';
 $('sbsinfo').textContent=r.sbs?'Serving on 127.0.0.1:'+r.sbs.port+' (this computer only), '+r.sbs.clients+' clients, '+r.sbs.sent+' lines, '+r.sbs.rejected+' rejected. A port change applies on the next start.':'Off.';
 $('legacybox').style.display=c.legacy_files.length?'block':'none'}
async function post(body){
 let j;try{j=await (await fetch('config',{method:'POST',headers:{'Content-Type':'application/json','X-ADSB-Settings':'1'},body:JSON.stringify(body)})).json()}catch(e){note('Could not save: '+e,false);return null}
 note(j.ok?(j.notes.length?j.notes.join('\n'):'Saved.'):j.error,j.ok);await loadSettings();return j}
function collect(){const num=id=>$(id).value===''?null:+$(id).value;
 const b={set:{receiver:{lat:num('f_lat'),lon:num('f_lon')},radio:{lna:+$('f_lna').value,vga:num('f_vga'),amp:$('f_amp').checked?1:0},history:{keep_days:num('f_keep')},
  routes:{enabled:$('f_routes').checked},swim:{url:$('f_swurl').value,vpn:$('f_swvpn').value},sbs:{port:num('f_sbs')},
  share:{enabled:$('f_share').checked,host:$('f_shost').value,port:num('f_sport')},
  hub:{enabled:$('f_hub').checked,host:$('f_hhost').value,port:num('f_hport'),radius_mi:+$('f_hrad').value}},secrets:{}};
 for(const n of SECRETS){const v=$('s_'+n).value;if(v)b.secrets[n]=v;else if($('r_'+n).checked)b.secrets[n]=null}return b}
$('gear').onclick=async()=>{note('',true);await loadSettings();dlg.showModal()};
$('b_close').onclick=()=>dlg.close();
$('b_save').onclick=()=>post(collect());
$('f_hrad').onchange=e=>{const v=+e.target.value;if((v===0||v>=1000)&&!confirm('Track '+(v?'everything within '+v+' miles':'the whole world')+'?\n\nThis saves a position every 30 seconds for each aircraft, so the history grows quickly: up to roughly 1 GB a day for the whole world, much less for 1000 miles. It also draws a lot of aircraft. You can lower Keep days under Position history to limit the disk use.'))e.target.value=String(lastRad);else lastRad=v};
$('f_share').onchange=e=>{if(e.target.checked&&!confirm('Share your aircraft data with ADSBHub?\n\nThis sends the position, altitude, callsign, speed and heading of every aircraft your receiver decodes, live, to '+$('f_shost').value+'. ADSBHub\'s terms say shared data may be republished and used commercially by anyone. The feed is tied to the station location and IP address you registered.\n\nYou can switch it off here at any time.'))e.target.checked=false};
$('b_import').onclick=()=>post({action:'import_legacy'});
$('b_delete').onclick=()=>{if(confirm('Delete ~/.swim_credentials and ~/.adsbhub_credentials?\n\nOnly do this after the Keychain import worked and the FAA connection still shows connected.'))post({action:'delete_legacy'})};
// ---- aircraft from the shared feed (ADSBHub), drawn small and grey-blue under our own so they are easy to tell apart
const hubR=L.canvas({padding:.5}),hubMk={},hubRt={},hubRtLines={},hubTr={},hubTrLine={};let hubBusy=false,hubList=[];
const hubStyle=k=>sel.has(k)?{radius:7,color:GREEN,weight:3,fillColor:'#063d12',fillOpacity:1}:{radius:3.5,color:'#27496d',weight:1,fillColor:'#7aa5d1',fillOpacity:.9};
function hubDrop(k){if(hubMk[k]){map.removeLayer(hubMk[k]);delete hubMk[k]}
 if(hubTrLine[k]){map.removeLayer(hubTrLine[k]);delete hubTrLine[k]}delete hubTr[k]}
function hubClear(){for(const k of Object.keys(hubMk))hubDrop(k)
 for(const k of Object.keys(hubRtLines)){map.removeLayer(hubRtLines[k]);delete hubRtLines[k]}hubList=[]}
function hubTip(a){const d=document.createElement('div');d.textContent=(a.cs||a.icao)+(a.alt!=null?'  '+Math.round(a.alt).toLocaleString()+' ft':'')+(a.gs!=null?'  '+Math.round(a.gs)+' kt':'')
 +(a.plan&&(a.plan.dep||a.plan.arr)?'  '+apt(a.plan.dep)+'>'+apt(a.plan.arr):'')+'  (shared feed)';return d}
// aircraft within the radius are drawn like our own (arrow, track, filed route); farther ones are small dots
async function hubSeed(ids){            // the recorded track from the last hour, so a reload does not start the trails from nothing
 for(let i=0;i<ids.length;i+=30){const part=ids.slice(i,i+30);
  try{const d=await (await fetch('tracks.json?icao='+part.join(',')+'&hours=1',{cache:'no-store'})).json();
   for(const k of part){if(!hubMk[k])continue;const segs=d[k]||[],last=segs.length?segs[segs.length-1]:[];hubTr[k]=last.map(p=>[p[0],p[1]]).slice(-150).concat(hubTr[k]||[])}}catch(e){}}}
async function hubRoutes(list){
 const want=new Set(),c=map.getCenter();
 if(showRoutes){const cand=list.filter(a=>a.near&&a.plan).sort((x,y)=>(sel.has(y.icao)-sel.has(x.icao))||(Math.abs(x.lat-c.lat)+Math.abs(x.lon-c.lng))-(Math.abs(y.lat-c.lat)+Math.abs(y.lon-c.lng))).slice(0,250);
  for(const a of cand)want.add(a.icao);
  const need=cand.filter(a=>{const r=hubRt[a.icao];return !r||Date.now()-r.t>300000}).slice(0,12);
  await Promise.all(need.map(async a=>{try{const j=await (await fetch('route.json?icao='+a.icao,{cache:'no-store'})).json();hubRt[a.icao]={t:Date.now(),pts:j.pts}}catch(e){}}));
  for(const a of cand){const pts=hubRt[a.icao]&&hubRt[a.icao].pts;
   if(pts&&pts.length>1){const key=JSON.stringify(pts);
    if(!hubRtLines[a.icao]||hubRtLines[a.icao].k!==key){if(hubRtLines[a.icao])map.removeLayer(hubRtLines[a.icao]);const g=mkRoute(pts);g.k=key;hubRtLines[a.icao]=g.addTo(map)}}
   else if(hubRtLines[a.icao]){map.removeLayer(hubRtLines[a.icao]);delete hubRtLines[a.icao]}}}
 for(const k of Object.keys(hubRtLines))if(!want.has(k)){map.removeLayer(hubRtLines[k]);delete hubRtLines[k]}   // out of range or routes switched off
 for(const k of Object.keys(hubRt))if(!want.has(k)&&Date.now()-hubRt[k].t>300000)delete hubRt[k]}
async function hubTick(){
 if(hubBusy||!hubOn)return;hubBusy=true;
 try{const b=map.getBounds();const j=await (await fetch('hub.json?s='+b.getSouth()+'&n='+b.getNorth()+'&w='+b.getWest()+'&e='+b.getEast(),{cache:'no-store'})).json();
  const seen=new Set(),fresh=[];hubList=j.aircraft;
  for(const a of j.aircraft){seen.add(a.icao);const k=a.icao;let m=hubMk[k];
   if(m&&m._near!==!!a.near){hubDrop(k);m=null}
   if(!m){
    if(a.near){m=hubMk[k]=L.marker([a.lat,a.lon],{icon:icon({icao:k,hdg:a.trk,alt:a.alt}),opacity:.9}).addTo(map);m._near=true;fresh.push(k)}
    else{m=hubMk[k]=L.circleMarker([a.lat,a.lon],{renderer:hubR,bubblingMouseEvents:false,...hubStyle(k)}).addTo(map);m._near=false}
    m.bindTooltip(hubTip(a),{direction:'top'});
    m.on('click',e=>select(k,e.originalEvent.shiftKey||e.originalEvent.metaKey||e.originalEvent.ctrlKey,false))}
   else{m.setLatLng([a.lat,a.lon]);m.setTooltipContent(hubTip(a));if(a.near)m.setIcon(icon({icao:k,hdg:a.trk,alt:a.alt}))}
   if(a.near){const t=hubTr[k];
    if(t){const l=t[t.length-1];if(!l||Math.abs(l[0]-a.lat)+Math.abs(l[1]-a.lon)>0.002){t.push([a.lat,a.lon]);if(t.length>200)t.shift()}
     if(t.length>1){if(!hubTrLine[k])hubTrLine[k]=greenLine(t,3).addTo(map);else hubTrLine[k].setLatLngs(t);
      hubTrLine[k].setStyle({opacity:sel.has(k)?0:.95})}}}}
  for(const k of Object.keys(hubMk))if(!seen.has(k))hubDrop(k);
  for(const k of Object.keys(hubTr))if(!hubMk[k]||!hubMk[k]._near)hubDrop(k);
  if(fresh.length)hubSeed(fresh.filter(k=>!hubTr[k]));
  for(const k of fresh)if(!hubTr[k])hubTr[k]=[];
  hubN=j.aircraft.length;hubTot=j.total;hubNear=j.aircraft.filter(a=>a.near).length;
  await hubRoutes(j.aircraft)}catch(e){}
 hubBusy=false;drawHub();if(lastJ&&($('hubl').checked||qEl.value.toLowerCase().includes('shared')))render(lastJ)}
let hubOn=false,hubN=0,hubTot=0,hubNear=0,hubSt=null;
function drawHub(){const el=$('hubb');$('hublw').style.display=hubSt&&hubSt.enabled?'inline':'none';
 if(!hubSt||!hubSt.enabled){el.textContent='';el.title='';return}
 el.textContent='Shared feed: '+(hubSt.state==='receiving'?hubNear+' tracked within '+(hubSt.radius_mi?hubSt.radius_mi+' mi':'worldwide')+', '+hubN+' in view of '+hubTot:hubSt.state);
 el.title=hubSt.error||('Aircraft received from '+hubSt.host+':'+hubSt.port+'. '+(hubSt.radius_mi?'Within '+hubSt.radius_mi+' miles of your receiver':'Everywhere')+' they are tracked, saved and get their filed route; farther ones are dots, only displayed.')}
setInterval(hubTick,3000);map.on('moveend',()=>{if(hubOn)hubTick()});
// ---- live data
let lastJ=null;
async function tick(){
 let j;try{j=await (await fetch('aircraft.json',{cache:'no-store'})).json()}catch(e){return}
 lastJ=j;hubSt=j.hub||null;const on=!!(hubSt&&hubSt.enabled);if(on!==hubOn){hubOn=on;if(on)hubTick();else hubClear()}drawHub();render(j)}
function render(j){
 if(j.receiver&&!home){home=L.latLng(j.receiver);L.circleMarker(home,{radius:6,color:'#2563eb',fillOpacity:.8}).addTo(map).bindTooltip('Receiver')}
 const seen=new Set();let withPos=0;
 for(const a of j.aircraft){                                  // the map: markers, short trails and filed routes of the live aircraft
  const name=a.cs||a.icao;seen.add(a.icao);
  if(a.lat!=null){withPos++;const ll=[a.lat,a.lon];
   if(!marks[a.icao]){const k=a.icao;
    marks[k]=L.marker(ll,{icon:icon(a)}).addTo(map).bindTooltip(name,{permanent:true,direction:'right',offset:[8,0],className:'lbl'})
     .on('click',e=>select(k,e.originalEvent.shiftKey||e.originalEvent.metaKey||e.originalEvent.ctrlKey,false));
    trails[k]=L.polyline(a.trail,{color:'#ff00ff',weight:4,opacity:1,className:'trail',interactive:false}).addTo(map);if(!fitted){fitted=true;fitAll()}}
   else{marks[a.icao].setLatLng(ll).setIcon(icon(a)).setTooltipContent(name);trails[a.icao].setLatLngs(a.trail)}
   trails[a.icao].bringToFront();                               // our own tracks stay above the shared ones
   trails[a.icao].setStyle({opacity:sel.has(a.icao)?0:1});     // a selected aircraft shows its full recorded track instead of the short trail
   marks[a.icao].setOpacity(a.age>30?.4:1)}
  const pl=showRoutes&&a.plan&&a.plan.pts?a.plan.pts:null;      // filed route, dashed cyan, drawn under the markers
  if(pl){const key=JSON.stringify(pl);
   if(!routeLines[a.icao]||routeLines[a.icao].k!==key){dropRoute(a.icao);routeLines[a.icao]={k:key,l:mkRoute(pl).addTo(map)}}}
  else dropRoute(a.icao)}
 for(const k of Object.keys(marks))if(!seen.has(k)){map.removeLayer(marks[k]);map.removeLayer(trails[k]);delete marks[k];delete trails[k]}
 for(const k of Object.keys(routeLines))if(!seen.has(k))dropRoute(k)           // a culled aircraft takes its route with it
 // the list: live aircraft and previous flights, searched and sorted
 const toks=qEl.value.toLowerCase().split(/\s+/).filter(Boolean);
 const all=rowsFor(j,hublEl.checked||toks.includes('shared')),rows=all.filter(r=>toks.every(t=>match(r,t))).sort(cmp);
 const tb=document.getElementById('rows');tb.textContent='';
 for(const r of rows.slice(0,1500)){
  const tr=document.createElement('tr');if(!r.live)tr.className='old';else if(r.age>30)tr.className='stale';else if(r.hub)tr.className='shared';
  if(sel.has(r.icao))tr.classList.add('sel');
  tr.title=(r.hub?'Shared feed (ADSBHub), not received by your radio\n':'')+(r.dep||r.arr?r.dep+' to '+r.arr:'No filed plan')+(r.type?', '+r.type:'')+(r.reg?', '+r.reg:'')+(r.off?'\nPlan does not fit this aircraft position, not drawn':'')+(r.route?'\n'+r.route:'')+(r.live?'':'\nPrevious flight, first seen '+new Date(r.first*1000).toLocaleString());
  [r.name,(r.dep||r.arr)?apt(r.dep)+'>'+apt(r.arr)+(r.off?' ?':''):'-',r.rules||'-',r.alt==null?'-':r.alt.toLocaleString(),r.spd==null?'-':r.spd,r.hdg==null?'-':r.hdg,r.msgs==null?'-':r.msgs,
   r.live&&r.age<20?'now':fmtTime(r.last)].forEach((v,i)=>{
   const td=document.createElement('td');td.textContent=v;if(i===2&&v==='IFR')td.className='rules-i';if(i===2&&v==='VFR')td.className='rules-v';
   if(i===7)td.title=new Date(r.last*1000).toLocaleString();tr.appendChild(td)});
  tr.onclick=e=>select(r.icao,e.shiftKey||e.metaKey||e.ctrlKey,r.live,r);tb.appendChild(tr)}
 document.getElementById('cnt').textContent=(toks.length?rows.length+' of '+all.length:all.length)+' flights'+(rows.length>1500?' (first 1500)':'')+(hublEl.checked&&hubOn?' | '+hubN+' shared in view':'');
 document.getElementById('s1').textContent=`${j.aircraft.length} aircraft, ${withPos} on map`;
 document.getElementById('s2').textContent=`${j.msg_rate.toFixed(1)} msg/s, status: ${j.status}`;
 const s3=document.getElementById('s3');s3.textContent=`ADC peak ${j.peak_adc}/127, CPU load ${j.load}%`+(j.peak_adc>110?' (near clipping, lower the gain)':'');
 s3.className=j.peak_adc>110||j.load>90?'warn':'';
 if(j.swim)s3.textContent+=' | routes: '+(j.swim.state==='connected'?j.swim.plans+' plans':j.swim.state);
 const shr=$('shr');if(j.share&&j.share.enabled){shr.textContent='SHARING ON ('+j.share.state+')';shr.style.cssText='background:#dc2626;color:#fff;padding:2px 8px;border-radius:6px;font-weight:700'}else{shr.textContent='';shr.style.cssText=''}
 if(j.sbs)s3.textContent+=' | SBS local :'+j.sbs.port+' '+j.sbs.clients+' clients, '+j.sbs.sent+' lines, '+j.sbs.rejected+' rejected';
 if(j.error)s3.textContent+=' | '+j.error}
loadHist().then(()=>{if(lastJ)render(lastJ)});
setInterval(tick,1000);tick();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            body, ctype = PAGE.encode(), "text/html; charset=utf-8"
        elif path == "/aircraft.json":
            now = time.time()
            with lock:
                for k in [k for k, a in aircraft.items() if now - a["last"] > EXPIRE_S]:
                    del aircraft[k]
                while msg_times and now - msg_times[0] > 10:
                    msg_times.popleft()
                out = [{k: v for k, v in a.items() if k not in ("even", "odd", "first", "last")}
                       | {"age": round(now - a["last"], 1)} for a in aircraft.values()]
                data = {"aircraft": out, "msg_rate": len(msg_times) / 10, "receiver": receiver, **stats}
            if plans is not None and cfg.get("routes", "enabled"):  # attach the filed flight plan where we have one
                for a in out:
                    rec = plans.get(a["icao"], a["cs"])
                    if rec:
                        pts, off = rec.get("pts"), False
                        if pts and a["lat"] is not None and not route_fits(pts, a["lat"], a["lon"], OFF_ROUTE_KM):
                            pts, off = None, True
                        a["plan"] = {"dep": rec.get("dep"), "arr": rec.get("arr"), "route": rec.get("route"),
                                     "type": rec.get("type"), "reg": rec.get("reg"), "rules": rec.get("rules"),
                                     "pts": pts, "off": off}
                data["swim"] = plans.status()
            if hub is not None:
                data["hub"] = hub.status()
            if sbs is not None:
                data["sbs"] = sbs.status()
            if uplink is not None:
                data["share"] = uplink.status()
            body, ctype = json.dumps(data).encode(), "application/json"
        elif path == "/tracks.json":                               # /tracks.json?icao=A4794E,A12B4D
            ids = [i for i in parse_qs(urlparse(self.path).query).get("icao", [""])[0].upper().split(",")
                   if re.fullmatch(r"[0-9A-F]{6}", i)][:30]
            q = parse_qs(urlparse(self.path).query)
            try:
                hours = max(1, min(int(q.get("hours", ["24"])[0]), 24 * 3650))     # the window to show, 24 hours by default
            except ValueError:
                hours = 24
            data = {i: tracks.read(i, hours=hours) for i in ids} if tracks is not None else {}
            body, ctype = json.dumps(data).encode(), "application/json"
        elif path == "/config.json":                               # the settings, never including a secret value
            if not self._same_origin(post=False):
                self.send_error(403)
                return
            body = json.dumps({"config": cfg.public(), "share": uplink.status() if uplink else None,
                               "swim": plans.status() if plans else None, "sbs": sbs.status() if sbs else None, "hub": hub.status() if hub else None,
                               "radio": stats["status"]}).encode()
            ctype = "application/json"
        elif path == "/hub.json":                                  # /hub.json?s=..&w=..&n=..&e=.. : aircraft from the shared feed in view
            q = parse_qs(urlparse(self.path).query)
            try:
                s_, w_, n_, e_ = (float(q.get(k, [d])[0]) for k, d in (("s", -90), ("w", -180), ("n", 90), ("e", 180)))
            except ValueError:
                self.send_error(400)
                return
            with lock:
                heard = set(aircraft)                              # what our own radio hears always takes priority
            lst, total = hub.snapshot(s_, w_, n_, e_, heard) if hub is not None and cfg.get("hub", "enabled") else ([], 0)
            if plans is not None and cfg.get("routes", "enabled"):     # the filed plan summary, where we hold one (no lines here)
                for a in lst:
                    p = plans.peek(a["icao"], a["cs"])
                    if p:
                        a["plan"] = p
            body, ctype = json.dumps({"aircraft": lst, "total": total}).encode(), "application/json"
        elif path == "/route.json":                                # /route.json?icao=HEX : the line for one shared aircraft
            icao = parse_qs(urlparse(self.path).query).get("icao", [""])[0].upper()
            a = hub.get(icao) if hub is not None and re.fullmatch(r"[0-9A-F]{6}", icao) else None
            rec = plans.get(icao, a.get("cs")) if (a and plans is not None and cfg.get("routes", "enabled")) else None
            pts = rec.get("pts") if rec else None
            if pts and a.get("lat") is not None and not route_fits(pts, a["lat"], a["lon"], OFF_ROUTE_KM):
                pts = None                                         # the plan does not fit where this aircraft is
            body, ctype = json.dumps({"pts": pts}).encode(), "application/json"
        elif path == "/sbs.json":                                  # what the SBS output sent and rejected, for review
            body, ctype = json.dumps({**(sbs.status() if sbs else {}), "recent_rejects": list(sbs.recent) if sbs else []}).encode(), "application/json"
        elif path == "/flights.json":                              # /flights.json?days=7, newest first
            try:
                days = max(0.01, min(float(parse_qs(urlparse(self.path).query).get("days", ["7"])[0]), 3650))
            except ValueError:
                days = 7
            body, ctype = json.dumps(flightlog.query(days) if flightlog is not None else []).encode(), "application/json"
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _same_origin(self, post=True):
        """Settings can only be read or changed by this page itself. Other websites, even though they can reach
        127.0.0.1, are refused: the Host and Origin must be ours and a POST must carry our own header."""
        port = self.server.server_address[1]
        host = self.headers.get("Host", "")
        if host not in ("127.0.0.1:%d" % port, "localhost:%d" % port):      # also blocks DNS rebinding
            return False
        origin = self.headers.get("Origin")
        if origin and origin != "http://" + host:
            return False
        if post:
            return (self.headers.get("X-ADSB-Settings") == "1"
                    and self.headers.get("Content-Type", "").startswith("application/json"))
        return True

    def do_POST(self):
        if urlparse(self.path).path != "/config" or not self._same_origin(post=True):
            self.send_error(403)
            return
        try:
            n = int(self.headers.get("Content-Length", "0"))
            if not 0 < n <= 100_000:
                raise ConfigError("the request is empty or too large")
            req = json.loads(self.rfile.read(n))
            if not isinstance(req, dict):
                raise ConfigError("the request must be a JSON object")
            out = {"ok": True, "notes": apply_settings(req)}
            code = 200
        except (ConfigError, ValueError) as e:
            out, code = {"ok": False, "error": str(e)}, 400
        out["config"] = cfg.public()
        body = json.dumps(out).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


def apply_settings(req):
    """Applies {"set": {...}, "secrets": {"name": "value" or null}, "action": "import_legacy" | "delete_legacy"}.
    Returns short notes for the user. Raises ConfigError if anything is invalid."""
    global receiver
    notes = []
    old = json.loads(json.dumps(cfg.data))
    if req.get("set"):
        cfg.update(req["set"])
    touched = []
    for name, val in (req.get("secrets") or {}).items():
        if val is None:
            cfg.clear_secret(name)
            notes.append("Removed %s from the Keychain." % name)
        else:
            cfg.set_secret(name, val)
            notes.append("Stored %s in the Keychain." % name)
        touched.append(name)
    action = req.get("action")
    if action == "import_legacy":
        done = cfg.import_legacy()
        touched += [d for d in done if not d.startswith("swim.")]
        notes.append("Imported into your settings and Keychain: " + (", ".join(done) or "nothing found") + ". The old files are still there.")
    elif action == "delete_legacy":
        notes.append("Deleted the plain text credential files: " + (", ".join(cfg.delete_legacy()) or "none found") + ".")
    elif action:
        raise ConfigError("unknown action")
    new = cfg.data
    if new["receiver"] != old["receiver"]:
        r = new["receiver"]
        receiver = [r["lat"], r["lon"]] if r["lat"] is not None and r["lon"] is not None else None
        notes.append("Receiver location updated.")
    if new["routes"] != old["routes"] and new["routes"]["enabled"] and plans is None:
        start_plans()
        notes.append("Connecting to the FAA feed.")
    if new["history"] != old["history"]:
        Tracks.KEEP_S = new["history"]["keep_days"] * 24 * 3600
        notes.append("Position history will be kept for %d days." % new["history"]["keep_days"])
    if new["radio"] != old["radio"] and radio["proc"] is not None:
        radio["proc"].terminate()                                    # the main loop starts it again with the new gains
        notes.append("The radio is restarting with the new gain settings.")
    if (new["swim"] != old["swim"] or any(t.startswith("swim_") for t in touched)) and plans is not None:
        plans.reconnect()
        notes.append("Reconnecting to the FAA feed with the new details.")
    if new["sbs"] != old["sbs"]:
        notes.append("The SBS port change takes effect the next time the map is started.")
    if new["hub"]["radius_mi"] != old["hub"]["radius_mi"]:
        r = new["hub"]["radius_mi"]
        notes.append("Shared aircraft %s are tracked and have their filed routes drawn." % ("anywhere in the world" if r == 0 else "within %d miles" % r))
    if {k: v for k, v in new["hub"].items() if k != "radius_mi"} != {k: v for k, v in old["hub"].items() if k != "radius_mi"}:
        notes.append("Receiving from %s:%d is now %s." % (new["hub"]["host"], new["hub"]["port"], "ON" if new["hub"]["enabled"] else "off"))
    if new["share"] != old["share"]:
        notes.append("Sharing to %s:%d is now %s." % (new["share"]["host"], new["share"]["port"],
                                                     "ON" if new["share"]["enabled"] else "off"))
    return notes


def swim_cred():
    """The FAA connection details: plain settings from the config file, secrets from the Keychain."""
    c = {"SWIM_URL": cfg.get("swim", "url"), "SWIM_VPN": cfg.get("swim", "vpn"), "SWIM_USER": cfg.secret("swim_user"),
         "SWIM_PASSWORD": cfg.secret("swim_password"), "SWIM_QUEUE": cfg.secret("swim_queue")}
    return c if all(c.values()) else None


def start_plans():
    global plans
    if Plans is None or plans is not None:
        return
    try:
        plans = Plans(Nav(), cred_source=swim_cred)
        plans.start()
    except Exception as e:                                         # for example the navigation data file is missing
        print("filed routes disabled:", type(e).__name__, e)


def radio_cmd():
    r = cfg.data["radio"]
    return ["hackrf_transfer", "-r", "-", "-f", "1090000000", "-s", str(FS), "-b", "2500000",
            "-l", str(r["lna"]), "-g", str(r["vga"]), "-a", str(r["amp"])]


def main():
    global receiver, plans, tracks, flightlog, sbs, cfg, uplink, hub
    ap = argparse.ArgumentParser(description="Live ADS-B map for a HackRF. Most settings live in the Settings dialog "
                                             "(stored in adsb_data/config.json and the macOS Keychain). These flags "
                                             "override them for this run only.")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--lna", type=int, help="LNA gain 0-40 dB, steps of 8")
    ap.add_argument("--vga", type=int, help="VGA gain 0-62 dB, steps of 2")
    ap.add_argument("--amp", type=int, choices=(0, 1), help="HackRF RF amplifier")
    ap.add_argument("--lat", type=float, help="your latitude, to mark the receiver on the map")
    ap.add_argument("--lon", type=float, help="your longitude")
    ap.add_argument("--file", help="replay a 4 Msps int8 IQ capture instead of using the HackRF")
    ap.add_argument("--no-routes", action="store_true", help="do not connect to the FAA SWIM feed for filed routes")
    ap.add_argument("--keep-days", type=int, help="days of position history to keep")
    ap.add_argument("--sbs-port", type=int, help="serve SBS text on this port, on 127.0.0.1 only (0 turns it off)")
    args = ap.parse_args()

    cfg = Config()
    first_run = not cfg.existed
    if first_run:
        cfg.save()                                                  # write the defaults, private to you
    over = {}
    for sec, key, val in (("radio", "lna", args.lna), ("radio", "vga", args.vga), ("radio", "amp", args.amp),
                          ("receiver", "lat", args.lat), ("receiver", "lon", args.lon),
                          ("history", "keep_days", args.keep_days), ("sbs", "port", args.sbs_port)):
        if val is not None:
            over.setdefault(sec, {})[key] = val
    if args.no_routes:
        over["routes"] = {"enabled": False}
    if over:
        try:
            cfg.update(over, persist=first_run)                     # on the first run the flags become the saved settings
        except ConfigError as e:
            sys.exit("bad command line option: %s" % e)
    r = cfg.data["receiver"]
    if r["lat"] is not None and r["lon"] is not None:
        receiver = [r["lat"], r["lon"]]

    Tracks.KEEP_S = cfg.get("history", "keep_days") * 24 * 3600
    tracks = Tracks(os.path.join(HERE, "adsb_data", "tracks.sqlite"))
    if cfg.get("routes", "enabled"):
        start_plans()
    flightlog = FlightLog(os.path.join(HERE, "adsb_data", "flights.sqlite"))
    port = cfg.get("sbs", "port")
    if port:
        try:
            sbs = SbsOut(port)
            print(f"SBS output on 127.0.0.1:{port} (this computer only)")
            uplink = HubUplink(sbs)                                 # idle: it connects only if sharing is switched on
        except OSError as e:
            print(f"SBS output disabled, port {port} unavailable: {e}")

    hub = HubDownlink()                                            # idle: it connects only if receiving is switched on
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"Open http://127.0.0.1:{args.port}   (Ctrl-C to stop)")
    if cfg.get("share", "enabled"):
        print("NOTE: sharing to %s:%d is switched ON in the settings" % (cfg.get("share", "host"), cfg.get("share", "port")))

    if args.file:
        try:
            decoder_loop(open(args.file, "rb"), True)
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        return

    proc = None
    try:
        while True:                                   # the radio can vanish (a USB glitch, say), so the map keeps retrying
            try:
                proc = radio["proc"] = subprocess.Popen(radio_cmd(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            except FileNotFoundError:
                sys.exit("hackrf_transfer not found. Install it with: brew install hackrf")

            def drain(p=proc):
                for line in p.stderr:
                    text = line.decode(errors="replace").strip()
                    if "error" in text.lower() or "not found" in text.lower():
                        stats["error"] = text
            threading.Thread(target=drain, daemon=True).start()
            decoder_loop(proc.stdout, False)          # returns when the radio stream ends
            proc.terminate()
            stats["status"] = "waiting for the HackRF"
            stats["error"] = RETRY_MSG
            print(time.strftime("%H:%M:%S"), RETRY_MSG)
            time.sleep(3)
    except KeyboardInterrupt:
        pass
    finally:
        if proc:
            proc.terminate()


if __name__ == "__main__":
    main()

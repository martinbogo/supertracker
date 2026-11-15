#!/usr/bin/env python3
"""FAA SWIM (SFDPS) flight plan consumer and route resolver, used by adsb_map.py.

Reads the connection details from ~/.swim_credentials (never printed), keeps the latest filed plan of every flight in
memory, and turns each filed route text (for example KMSP./.SMERF..DIK.J36.MLP.GLASR3.KSEA/0353) into map coordinates
using the FAA navigation data in adsb_data/navdata.json (see adsb_data/build_navdata.py).

Needs: solace-pubsubplus and certifi (pip install solace-pubsubplus certifi). Without them adsb_map.py still works and
simply shows no routes.

Self test on saved messages:  python3 swim_routes.py --test DIRECTORY_WITH_XML_FILES
"""
import glob
import json
import math
import os
import re
import sys
import threading
import time
from xml.etree import ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
CRED = os.path.expanduser("~/.swim_credentials")
NAVDATA = os.path.join(HERE, "adsb_data", "navdata.json")
WORLD = os.path.join(HERE, "adsb_data", "world_airports.json")   # ICAO code -> [lat, lon], from OurAirports (public domain)
ROUTE_VERSION = 2                                # bump when the way lines are worked out changes, so cached lines are redone
CACHE = os.path.join(HERE, "adsb_data", "plans_cache.json")
MAX_AGE_S = 18 * 3600
MAX_PLANS = 60000


def hav_km(a, b):
    p1, p2 = math.radians(a[0]), math.radians(b[0])
    dl = math.radians(b[1] - a[1])
    h = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 12742.0 * math.asin(min(1.0, math.sqrt(h)))


def densify(pts, step_km=40.0):
    """Points along the great circle legs, so distances to a long leg are measured to the leg, not just its ends."""
    out = []
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(hav_km(a, b) / step_km))
        la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
        d = 2 * math.asin(min(1.0, math.sqrt(math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2)))
        for i in range(n):
            f = i / n
            if d < 1e-9:
                out.append(a)
                continue
            A, B = math.sin((1 - f) * d) / math.sin(d), math.sin(f * d) / math.sin(d)
            x = A * math.cos(la1) * math.cos(lo1) + B * math.cos(la2) * math.cos(lo2)
            y = A * math.cos(la1) * math.sin(lo1) + B * math.cos(la2) * math.sin(lo2)
            z = A * math.sin(la1) + B * math.sin(la2)
            out.append([math.degrees(math.atan2(z, math.hypot(x, y))), math.degrees(math.atan2(y, x))])
    if pts:
        out.append(pts[-1])
    return out


def distance_to_route_km(pts, lat, lon):
    return min((hav_km(p, (lat, lon)) for p in densify(pts)), default=1e9)


def route_fits(pts, lat, lon, filed_km=250.0, est_km=1500.0):
    """True when the aircraft is near the filed part of the line, or within est_km of an estimated part. Estimated legs
    (marked by a third value of 1 on their end points) are straight great circles, which real flights follow only loosely."""
    filed = [p for i, p in enumerate(pts) if not (len(p) > 2 and p[2]) or (i + 1 < len(pts) and not (len(pts[i + 1]) > 2 and pts[i + 1][2]))]
    est = [p for p in pts if len(p) > 2 and p[2]]
    if filed and min((hav_km(p, (lat, lon)) for p in densify(filed)), default=1e9) <= filed_km:
        return True
    return bool(est) and min((hav_km(p, (lat, lon)) for p in densify(pts)), default=1e9) <= est_km


def gc_points(a, b, step_km=150.0):
    """Points along the great circle from a to b, excluding a and including b, longitudes continuing from a (no jump at +-180)."""
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    d = 2 * math.asin(min(1.0, math.sqrt(math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2)))
    n = max(1, int(d * 6371.0 / step_km))
    out, prev_lon = [], a[1]
    for i in range(1, n + 1):
        f = i / n
        if d < 1e-9:
            break
        A, B = math.sin((1 - f) * d) / math.sin(d), math.sin(f * d) / math.sin(d)
        x = A * math.cos(la1) * math.cos(lo1) + B * math.cos(la2) * math.cos(lo2)
        y = A * math.cos(la1) * math.sin(lo1) + B * math.cos(la2) * math.sin(lo2)
        z = A * math.sin(la1) + B * math.sin(la2)
        lat, lon = math.degrees(math.atan2(z, math.hypot(x, y))), math.degrees(math.atan2(y, x))
        while lon - prev_lon > 180:
            lon -= 360
        while lon - prev_lon < -180:
            lon += 360
        out.append([lat, lon])
        prev_lon = lon
    return out


class Nav:
    def __init__(self, path=NAVDATA):
        with open(path) as f:
            d = json.load(f)
        self.apt, self.pt, self.awy, self.star, self.dp = d["apt"], d["pt"], d["awy"], d["star"], d["dp"]
        try:
            with open(WORLD) as f:
                self.world = json.load(f)
        except (OSError, ValueError):
            self.world = {}

    def point(self, name, near):
        c = self.pt.get(name)
        if not c:
            return None
        if len(c) == 1 or near is None:
            return c[0]
        return min(c, key=lambda p: hav_km(p, near))

    def airport(self, ident):
        return self.apt.get(ident)

    def foreign(self, ident):
        """Coordinates of an airport that is not in the FAA data, or None."""
        return None if ident in self.apt else self.world.get(ident or "")

    def faa_id(self, icao):
        return icao[1:] if len(icao) == 4 and icao[0] == "K" else icao


_RAD = re.compile(r"^([A-Z]{2,4})(\d{3})(\d{3})$")
_LL = [re.compile(r"^(\d{2})([NS])(\d{3})([EW])$"), re.compile(r"^(\d{2})(\d{2})([NS])(\d{3})(\d{2})([EW])$")]


def parse_latlon(tok):
    m = _LL[0].match(tok)
    if m:
        la, lo = float(m.group(1)), float(m.group(3))
        return [la if m.group(2) == "N" else -la, lo if m.group(4) == "E" else -lo]
    m = _LL[1].match(tok)
    if m:
        la, lo = int(m.group(1)) + int(m.group(2)) / 60.0, int(m.group(4)) + int(m.group(5)) / 60.0
        return [la if m.group(3) == "N" else -la, lo if m.group(6) == "E" else -lo]
    return None


def resolve_route(nav, text, dep, arr):
    """Returns a list of [lat, lon] along the filed route, or None if it cannot be drawn."""
    pts = []
    origin, dest = nav.airport(dep or ""), nav.airport(arr or "")
    far_origin, far_dest = (None if origin else nav.foreign(dep)), (None if dest else nav.foreign(arr))
    if origin:
        pts.append(origin)
    toks = [t.split("/")[0] for t in re.split(r"[.\s]+", (text or "").upper())]
    toks = [t for t in toks if t]
    prev, airway = None, None
    for idx, t in enumerate(toks):
        near = pts[-1] if pts else dest
        if idx == 0 and t in (dep, nav.faa_id(dep or "")):
            continue
        if idx == len(toks) - 1 and t in (arr, nav.faa_id(arr or "")):
            continue
        ll = parse_latlon(t)
        if ll:
            pts.append(ll)
            prev, airway = None, None
            continue
        m = _RAD.match(t)                                   # navaid, radial and distance, for example OKK356039
        if m and m.group(1) in nav.pt:
            base = nav.point(m.group(1), near)
            brg, dist = math.radians(int(m.group(2))), int(m.group(3)) * 1.852
            la1, lo1 = math.radians(base[0]), math.radians(base[1])
            d = dist / 6371.0
            la2 = math.asin(math.sin(la1) * math.cos(d) + math.cos(la1) * math.sin(d) * math.cos(brg))
            lo2 = lo1 + math.atan2(math.sin(brg) * math.sin(d) * math.cos(la1), math.cos(d) - math.sin(la1) * math.sin(la2))
            pts.append([math.degrees(la2), math.degrees(lo2)])
            prev, airway = None, None
            continue
        if prev and t in nav.awy and t not in nav.pt:
            airway = t
            continue
        proc = nav.dp if idx <= 1 and t in nav.dp else (nav.star if t in nav.star else None)
        if proc is not None and t not in nav.pt:
            options = proc[t]
            key = nav.faa_id(arr or "") if proc is nav.star else nav.faa_id(dep or "")
            best = next((o for o in options if key and key in o["rt"].replace("-", " ").split()), options[0])
            for name in best["pts"]:
                p = nav.point(name, pts[-1] if pts else near)
                if p and p != (pts[-1] if pts else None):
                    pts.append(p)
                    prev = name
            airway = None
            continue
        p = nav.point(t, near)
        if not p:
            continue
        if airway and prev:
            seg = None
            for s in nav.awy.get(airway, []):
                if prev in s and t in s:
                    i1, i2 = s.index(prev), s.index(t)
                    seg = s[i1:i2 + 1] if i1 <= i2 else s[i2:i1 + 1][::-1]
                    break
            if seg:
                for name in seg[1:]:
                    q = nav.point(name, pts[-1] if pts else near)
                    if q:
                        pts.append(q)
            else:
                pts.append(p)
        else:
            pts.append(p)
        prev, airway = t, None
    if dest:
        pts.append(dest)
    clean = []
    for p in pts:
        if not clean or hav_km(clean[-1], p) > 0.5:
            clean.append([round(p[0], 4), round(p[1], 4)])
    if len(clean) < 1 or (len(clean) < 2 and not (far_origin or far_dest)):
        return None
    # The FAA text covers only the part flown in US-controlled airspace. Join it to a foreign airport with an estimated
    # great circle leg, marked with a third value of 1 on each of its points, anchored to the filed end so the line
    # stays next to the US map view even when it crosses the date line.
    if far_origin:
        leg = gc_points(clean[0], far_origin)                 # from the filed end towards the airport
        head = [[round(q[0], 4), round(q[1], 4), 1] for q in reversed(leg)]
        if head:
            head[0] = head[0][:2]                             # the airport itself starts the line
        clean[0] = clean[0] + [1] if head else clean[0]
        clean = head + clean
    if far_dest:
        clean += [[round(q[0], 4), round(q[1], 4), 1] for q in gc_points(clean[-1], far_dest)]
    return clean if len(clean) >= 2 else None


def local(tag):
    return tag.split("}")[-1]


def child(el, name):
    if el is None:
        return None
    for c in el:
        if local(c.tag) == name:
            return c
    return None


class Plans:
    def __init__(self, nav, cred_source=None):
        self.nav = nav
        self.cred_source = cred_source
        self._reload = False
        self.lock = threading.Lock()
        self.by_gufi, self.by_hex, self.by_cs = {}, {}, {}
        self.msgs = 0
        self.errors = 0
        self.state = "not started"
        self.last_msg = 0.0
        self.last_purge = time.time()
        self.last_save = 0.0
        self.kinds = {}                                  # message source code and whether it carried a route, to tune the subscription
        self._load_cache()

    # Plans are kept on disk so a restart does not lose the flights already in the air.
    def _load_cache(self):
        try:
            with open(CACHE) as f:
                saved = json.load(f)
        except Exception:
            return
        now = time.time()
        for rec in saved:
            if now - rec.get("ts", 0) < MAX_AGE_S:
                if rec.get("rv") != ROUTE_VERSION:
                    rec["pts"], rec["resolved"], rec["rv"] = None, False, ROUTE_VERSION     # redo the line with the current rules
                self.by_gufi[rec["gufi"]] = rec
                if rec.get("hex"):
                    self.by_hex[rec["hex"]] = rec["gufi"]
                if rec.get("cs"):
                    self.by_cs[rec["cs"]] = rec["gufi"]

    def save_cache(self):
        with self.lock:
            data = [r for r in self.by_gufi.values() if r.get("seen")]    # only flights that have appeared on the map
        tmp = CACHE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f, separators=(",", ":"))
        os.replace(tmp, CACHE)
        self.last_save = time.time()

    def ingest(self, xml_text):
        root = ET.fromstring(xml_text)
        for fl in root.iter():
            if local(fl.tag) == "flight":
                self._flight(fl)
        self.msgs += 1
        self.last_msg = time.time()
        if self.last_msg - self.last_save > 30:
            self.save_cache()

    def _flight(self, fl):
        gufi_el = child(fl, "gufi")
        fid = child(fl, "flightIdentification")
        gufi = (gufi_el.text or "").strip() if gufi_el is not None else ""
        cs = fid.get("aircraftIdentification") if fid is not None else None
        if not gufi:
            gufi = "cs:" + (cs or "?")
        acd = child(fl, "aircraftDescription")
        route_el = child(child(fl, "agreed"), "route")
        status_el = child(fl, "flightStatus")
        dep_el, arr_el = child(fl, "departure"), child(fl, "arrival")
        alt = child(child(fl, "assignedAltitude"), "simple")
        upd = {
            "cs": cs,
            "hex": (acd.get("aircraftAddress") or "").upper() if acd is not None else None,
            "reg": acd.get("registration") if acd is not None else None,
            "type": None,
            "dep": dep_el.get("departurePoint") if dep_el is not None else None,
            "arr": arr_el.get("arrivalPoint") if arr_el is not None else None,
            "route": route_el.get("nasRouteText") if route_el is not None else None,
            "rules": route_el.get("initialFlightRules") if route_el is not None else None,
            "alt": float(alt.text) if alt is not None and alt.text else None,
            "center": fl.get("centre"),
            "status": status_el.get("fdpsFlightStatus") if status_el is not None else None,
        }
        t = child(child(acd, "aircraftType"), "icaoModelIdentifier") if acd is not None else None
        if t is not None:
            upd["type"] = (t.text or "").strip()
        with self.lock:
            kind = "%s %s" % (fl.get("source") or "?", "route" if upd["route"] else "no route")
            self.kinds[kind] = self.kinds.get(kind, 0) + 1
            if upd["status"] and "CANCEL" in upd["status"].upper():
                self._drop(gufi)
                return
            rec = self.by_gufi.setdefault(gufi, {"gufi": gufi, "pts": None, "resolved": False})
            for k, v in upd.items():
                if v not in (None, "") and rec.get(k) != v:
                    rec[k] = v
                    if k in ("route", "dep", "arr"):
                        rec["pts"], rec["resolved"] = None, False   # the line is worked out later, only if we ever see this flight
            rec["ts"] = time.time()
            if rec.get("hex"):
                self.by_hex[rec["hex"]] = gufi
            if rec.get("cs"):
                self.by_cs[rec["cs"]] = gufi
            if time.time() - self.last_purge > 600:
                self._purge()

    def _drop(self, gufi):
        rec = self.by_gufi.pop(gufi, None)
        if rec:
            if self.by_hex.get(rec.get("hex")) == gufi:
                del self.by_hex[rec["hex"]]
            if self.by_cs.get(rec.get("cs")) == gufi:
                del self.by_cs[rec["cs"]]

    def _purge(self):
        self.last_purge = time.time()
        for g in [g for g, r in self.by_gufi.items() if time.time() - r["ts"] > MAX_AGE_S]:
            self._drop(g)
        if len(self.by_gufi) > MAX_PLANS:              # keep memory bounded when the feed is nationwide
            for g, _ in sorted(self.by_gufi.items(), key=lambda kv: kv[1]["ts"])[:len(self.by_gufi) - MAX_PLANS]:
                self._drop(g)

    def get(self, hex_addr, callsign):
        with self.lock:
            g = self.by_hex.get((hex_addr or "").upper()) or self.by_cs.get(callsign or "")
            rec = self.by_gufi.get(g) if g else None
            if not rec:
                return None
            rec["seen"] = time.time()                  # a flight we have displayed, so it is worth keeping on disk
            route, dep, arr = rec.get("route"), rec.get("dep"), rec.get("arr")
            need = bool(route) and not rec.get("resolved")
        if need:                                       # nationwide the feed is large, so routes are resolved on demand
            pts = resolve_route(self.nav, route, dep, arr)
            with self.lock:
                if rec.get("route") == route:          # not amended in the meantime
                    rec["pts"], rec["resolved"], rec["rv"] = pts, True, ROUTE_VERSION
        with self.lock:
            return dict(rec)

    def peek(self, hex_addr, callsign):
        """The plan summary for an aircraft, without drawing a line or marking the flight as seen (used for the shared feed)."""
        with self.lock:
            g = self.by_hex.get((hex_addr or "").upper()) or self.by_cs.get(callsign or "")
            rec = self.by_gufi.get(g) if g else None
            return {k: rec.get(k) for k in ("dep", "arr", "route", "type", "reg", "rules")} if rec else None

    def status(self):
        with self.lock:
            return {"state": self.state, "msgs": self.msgs, "plans": len(self.by_gufi), "errors": self.errors, "kinds": dict(self.kinds),
                    "age": round(time.time() - self.last_msg) if self.last_msg else None}

    # ------------------------------------------------------------ live connection
    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        wait = 5
        while True:
            try:
                self._session()
                wait = 5
            except Exception as e:
                self.state = "error: " + self._redact(type(e).__name__ + " " + str(e))[:160]
            time.sleep(wait)
            wait = min(60, wait * 2)

    def _load_cred(self):
        """Connection details from the settings and the Keychain (cred_source), else from the old plain text file."""
        cred = self.cred_source() if self.cred_source else None
        if not cred and os.path.exists(CRED):
            cred = {}
            with open(CRED, encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith("#") and "=" in line:
                        k, v = line.split("=", 1)
                        cred[k.strip()] = v.strip().strip('"').strip("'")
        self._cred = cred or {}
        return self._cred

    def reconnect(self):
        """Ask the running session to end so it starts again with the current details."""
        self._reload = True

    def _redact(self, text):
        for k in ("SWIM_PASSWORD", "SWIM_USER", "SWIM_QUEUE", "SWIM_VPN", "SWIM_URL"):
            v = getattr(self, "_cred", {}).get(k)
            if v:
                text = text.replace(v, "<" + k + ">")
        return text

    def _session(self):
        try:
            import certifi
            from solace.messaging.config.transport_security_strategy import TLS
            from solace.messaging.messaging_service import MessagingService
            from solace.messaging.resources.queue import Queue
        except ImportError:
            self.state = "solace-pubsubplus not installed"
            time.sleep(3600)
            return
        self._reload = False
        cred = self._load_cred()
        missing = [k for k in ("SWIM_URL", "SWIM_VPN", "SWIM_USER", "SWIM_PASSWORD", "SWIM_QUEUE") if not cred.get(k)]
        if missing:
            self.state = "no connection details yet, open Settings" if len(missing) == 5 else "details missing: " + ", ".join(missing)
            time.sleep(15)                      # check again soon, the details may have just been saved
            return
        self.state = "connecting"
        props = {
            "solace.messaging.transport.host": cred["SWIM_URL"],
            "solace.messaging.service.vpn-name": cred["SWIM_VPN"],
            "solace.messaging.authentication.scheme.basic.username": cred["SWIM_USER"],
            "solace.messaging.authentication.scheme.basic.password": cred["SWIM_PASSWORD"],
        }
        tls = TLS.create().with_certificate_validation(False, True, trust_store_file_path=os.path.dirname(certifi.where()))
        service = MessagingService.builder().from_properties(props).with_transport_security_strategy(tls).build()
        service.connect()
        try:
            receiver = service.create_persistent_message_receiver_builder().with_message_auto_acknowledgement().build(
                Queue.durable_exclusive_queue(cred["SWIM_QUEUE"]))
            receiver.start()
            self.state = "connected"
            while not self._reload:
                msg = receiver.receive_message(timeout=5000)
                if msg is None:
                    continue
                payload = msg.get_payload_as_string()
                if payload is None:
                    payload = (msg.get_payload_as_bytes() or b"").decode("utf-8", errors="replace")
                try:
                    self.ingest(payload)
                except Exception:
                    self.errors += 1
        finally:
            try:
                service.disconnect()
            except Exception:
                pass


def _selftest(directory):
    nav = Nav()
    plans = Plans(nav)
    for f in sorted(glob.glob(os.path.join(directory, "*.xml"))):
        plans.ingest(open(f, encoding="utf-8").read())
    print("loaded %d messages, %d plans" % (plans.msgs, len(plans.by_gufi)))
    for rec in sorted(plans.by_gufi.values(), key=lambda r: r.get("cs") or ""):
        rec = plans.get(rec.get("hex"), rec.get("cs")) or rec     # routes are resolved on first use
        pts = rec.get("pts")
        print("%-8s %s %s>%s  %-5s hex %s  route points: %s" % (
            rec.get("cs"), rec.get("reg") or "-", rec.get("dep"), rec.get("arr"), rec.get("type") or "-", rec.get("hex"),
            len(pts) if pts else "NONE"))
        print("          ", rec.get("route"))
        if pts:
            print("           first", pts[0], "last", pts[-1])


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--test":
        _selftest(sys.argv[2])
    else:
        print(__doc__)

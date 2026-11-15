#!/usr/bin/env python3
"""Tests the shared feed receiver against a local mock server (it never contacts ADSBHub).

Run:  ./adsb_venv/bin/python tests/test_hub_downlink.py        Exits with status 1 if any check fails.
"""
import os
import socket
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import adsb_config
import adsb_map

cfg = adsb_config.Config(os.path.join(tempfile.mkdtemp(), "c.json"))
adsb_map.cfg = cfg
adsb_map.receiver = [45.5152, -122.6784]
failures = []


def check(name, ok, detail=""):
    print(("ok    " if ok else "FAIL  ") + name + (("  " + str(detail)) if detail and not ok else ""))
    if not ok:
        failures.append(name)


def msg(kind, hexid, cs="", alt="", gs="", trk="", lat="", lon="", vr=""):
    return ("MSG,%s,1,1,%s,1,2026/10/20,12:00:00.000,2026/10/20,12:00:00.000,%s,%s,%s,%s,%s,%s,%s,,,,,\r\n"
            % (kind, hexid, cs, alt, gs, trk, lat, lon, vr)).encode()


good = [msg(1, "4CA7B4", cs="RYR123"), msg(3, "4CA7B4", alt="36000", lat="53.4", lon="-6.2"), msg(4, "4CA7B4", gs="450", trk="90", vr="0"),
        msg(3, "7C6B2A", alt="12000", lat="-33.9", lon="151.2"),
        msg(3, "A1B2C3", alt="30000", lat="35.5", lon="179.5"), msg(3, "A1B2C4", alt="30000", lat="35.5", lon="-179.5"),
        msg(3, "A0A0A0", alt="20000", lat="45.6", lon="-122.5")]                       # about 7 miles from the receiver
bad = [msg(1, "4CA7B5", cs="<script>"), msg(3, "4CA7B6", lat="95", lon="10", alt="1000"), msg(3, "4CA7B7", lat="10", lon="200", alt="1000"),
       msg(3, "4CA7B8", lat="10", lon="10", alt="999999"), msg(4, "4CA7B9", gs="5000", trk="10"), msg(3, "ZZZZZZ", lat="1", lon="1"),
       b"garbage line\r\n", b"MSG,3,short\r\n", b"X" * 5000 + b"\r\n", msg(3, "000000", lat="1", lon="1", alt="100"),
       msg(3, "4CA7BA", lat="abc", lon="1", alt="100")]
jump = [msg(3, "4CA7BC", lat="10", lon="10", alt="1000"), msg(3, "4CA7BC", lat="20", lon="20", alt="1000")]

conns, mode = [], {"m": "data"}
srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", 0))
srv.listen(5)
port = srv.getsockname()[1]


def handle(c):
    conns.append(time.time())
    if mode["m"] == "close":
        c.close()
        return
    for line in good + bad + jump:
        c.sendall(line)
    time.sleep(30)
    c.close()


def serve():
    while True:
        c, _ = srv.accept()
        threading.Thread(target=handle, args=(c,), daemon=True).start()


threading.Thread(target=serve, daemon=True).start()

hub = adsb_map.HubDownlink()
time.sleep(2.5)
check("switched off: no connection is made", len(conns) == 0 and hub.state == "off")

cfg.update({"hub": {"host": "127.0.0.1", "port": port, "enabled": True, "radius_mi": 100}}, persist=False)
t = time.time()
while time.time() - t < 10 and hub.state != "receiving":
    time.sleep(0.2)
time.sleep(1)
st = hub.status()
check("receives", st["state"] == "receiving")
snap, total = hub.snapshot(-90, -180, 90, 180)
names = sorted(a["icao"] for a in snap)
check("keeps only valid aircraft", names == sorted(["4CA7B4", "7C6B2A", "A1B2C3", "A1B2C4", "A0A0A0", "4CA7BC"]), names)
check("rejects the position jump", st["reasons"].get("position jump over 1000 kt") == 1, st["reasons"])
check("rejects malformed, out of range and oversized lines", st["rejected"] >= 11, st["rejected"])
check("a hostile callsign is never stored", not any("<" in str(v) for v in hub.ac.values()))
check("near flag follows the radius", {a["icao"]: a["near"] for a in snap}["A0A0A0"] and not {a["icao"]: a["near"] for a in snap}["4CA7B4"])
check("near aircraft come first in a capped snapshot", hub.snapshot(-90, -180, 90, 180, limit=1)[0][0]["icao"] == "A0A0A0")
check("box over Ireland", [x["icao"] for x in hub.snapshot(50, -12, 56, 0)[0]] == ["4CA7B4"])
check("box across the date line", sorted(x["icao"] for x in hub.snapshot(30, 170, 40, -170)[0]) == ["A1B2C3", "A1B2C4"])
check("aircraft our own radio hears are excluded", all(x["icao"] != "4CA7B4" for x in hub.snapshot(-90, -180, 90, 180, exclude={"4CA7B4"})[0]))

cfg.update({"hub": {"radius_mi": 0}}, persist=False)
time.sleep(0.2)
cfg.update({"hub": {"enabled": False}}, persist=False)
time.sleep(2.5)
check("switching off drops the connection and the aircraft", hub.state == "off" and not hub.ac)

n = len(conns)
mode["m"] = "close"
cfg.update({"hub": {"enabled": True}}, persist=False)
time.sleep(7)
check("a server that closes at once is reported", hub.state == "error" and "closed the connection" in hub.error, (hub.state, hub.error))
cfg.update({"hub": {"enabled": False}}, persist=False)

try:
    cfg.update({"hub": {"radius_mi": 250}}, persist=False)
    check("an unlisted radius is refused", False)
except adsb_config.ConfigError:
    check("an unlisted radius is refused", True)

print("\n%d check(s) failed" % len(failures) if failures else "\nall checks passed")
sys.exit(1 if failures else 0)

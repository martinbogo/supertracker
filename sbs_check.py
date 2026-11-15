#!/usr/bin/env python3
"""Audit an SBS (BaseStation) stream: connect, read for a while, and report anything that looks wrong.

This is independent of the server's own checks, so it tests what actually comes out. It only reads.

Usage:  python3 sbs_check.py [--seconds 60] [--host 127.0.0.1] [--port 30003] [--lat 45.5152 --lon -122.6784]
Exit status is 0 when no problems were found, 1 otherwise.
"""
import argparse
import collections
import math
import re
import socket
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=30003)
ap.add_argument("--seconds", type=float, default=60)
ap.add_argument("--lat", type=float, help="receiver latitude, to check positions are inside radio range")
ap.add_argument("--lon", type=float)
ap.add_argument("--range-km", type=float, default=650)
args = ap.parse_args()


def hav_km(a, b, c, d):
    p1, p2 = math.radians(a), math.radians(c)
    h = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(math.radians(d - b) / 2) ** 2
    return 12742.0 * math.asin(min(1.0, math.sqrt(h)))


problems = collections.Counter()
examples = collections.defaultdict(list)


def problem(kind, line):
    problems[kind] += 1
    if len(examples[kind]) < 3:
        examples[kind].append(line[:150])


s = socket.create_connection((args.host, args.port), timeout=10)
s.settimeout(3)
buf, lines, end = b"", [], time.time() + args.seconds
print("connected to %s:%d, reading for %.0f s" % (args.host, args.port, args.seconds))
while time.time() < end:
    try:
        d = s.recv(65536)
    except socket.timeout:
        continue
    if not d:
        break
    buf += d
    while b"\n" in buf:
        ln, buf = buf.split(b"\n", 1)
        lines.append((time.time(), ln.decode("latin-1").rstrip("\r")))
s.close()

kinds, hexes, last = collections.Counter(), set(), {}
callsigns = collections.defaultdict(set)
for t, ln in lines:
    f = ln.split(",")
    if f[0] != "MSG":
        problem("not a MSG line", ln)
        continue
    if len(f) != 22:
        problem("wrong field count (%d, expected 22)" % len(f), ln)
        continue
    kinds[f[1]] += 1
    h = f[4]
    if not re.fullmatch(r"[0-9A-F]{6}", h) or h in ("000000", "FFFFFF"):
        problem("bad aircraft address", ln)
        continue
    hexes.add(h)
    if not (re.fullmatch(r"\d{4}/\d\d/\d\d", f[6]) and re.fullmatch(r"\d\d:\d\d:\d\d\.\d{3}", f[7])):
        problem("bad date or time", ln)
    try:
        if f[1] == "1":
            if not re.fullmatch(r"[A-Z0-9]{1,8}", f[10]):
                problem("bad callsign", ln)
            callsigns[h].add(f[10])
        elif f[1] == "3":
            lat, lon = float(f[14]), float(f[15])
            alt = float(f[11]) if f[11] != "" else None
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                problem("position out of range", ln)
            if args.lat is not None and hav_km(args.lat, args.lon, lat, lon) > args.range_km:
                problem("position beyond %d km from the receiver" % args.range_km, ln)
            if alt is not None and not (-1500 <= alt <= 60000):
                problem("altitude out of range", ln)
            p = last.get(h)
            if p:
                dt = t - p[0]
                nm = hav_km(p[1], p[2], lat, lon) / 1.852
                if dt > 0.5 and nm / (dt / 3600.0) > 900:
                    problem("position jump over 900 kt", ln)
            last[h] = (t, lat, lon)
        elif f[1] == "4":
            gs, trk = float(f[12]), float(f[13])
            if not (0 <= gs <= 900 and 0 <= trk < 360):
                problem("speed or heading out of range", ln)
            if f[16] != "" and abs(float(f[16])) > 10000:
                problem("climb rate out of range", ln)
        else:
            problem("unexpected message type " + f[1], ln)
    except ValueError:
        problem("a number could not be read", ln)

n = len(lines)
print("\nlines read: %d (%.1f per second)" % (n, n / max(1.0, args.seconds)))
print("message types:", dict(sorted(kinds.items())), "(1 callsign, 3 position, 4 velocity)")
print("distinct aircraft:", len(hexes), "| aircraft with a callsign:", len(callsigns),
      "| aircraft seen with two different callsigns:", sum(1 for v in callsigns.values() if len(v) > 1))
if lines:
    print("sample lines:")
    for _, ln in lines[:3] + lines[len(lines) // 2:len(lines) // 2 + 2]:
        print("  ", ln)
if problems:
    print("\nPROBLEMS FOUND:")
    for k, c in problems.most_common():
        print("  %5d  %s" % (c, k))
        for e in examples[k]:
            print("           e.g.", e)
    sys.exit(1)
print("\nno problems found" if n else "\nno data received (are there aircraft in range?)")

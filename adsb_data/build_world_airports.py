#!/usr/bin/env python3
"""Build world_airports.json (ICAO code -> [lat, lon]) from the OurAirports airports.csv (public domain, https://ourairports.com/data/).

Usage: python3 build_world_airports.py airports.csv [OUTPUT.json]
The map uses it to join a filed route to a foreign airport with an estimated great circle leg.
"""
import csv
import json
import os
import sys

out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "world_airports.json")
airports = {}
with open(sys.argv[1], newline="", encoding="utf-8") as f:
    for r in csv.DictReader(f):
        if r["type"] in ("closed", "heliport", "seaplane_base", "balloonport"):
            continue
        for k in (r["icao_code"], r["gps_code"], r["ident"]):
            if len(k) == 4 and k.isalpha() and k.isupper():
                airports.setdefault(k, [round(float(r["latitude_deg"]), 4), round(float(r["longitude_deg"]), 4)])
with open(out, "w") as f:
    json.dump(airports, f, separators=(",", ":"))
print(len(airports), "airports written to", out)

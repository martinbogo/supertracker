#!/usr/bin/env python3
"""Build a compact navigation database (airports, fixes, navaids, airways, STAR and DP procedures) from the FAA NASR
CSV files (https://www.faa.gov/air_traffic/flight_info/aeronav/aero_data/NASR_Subscription/).

Usage: python3 build_navdata.py DIR [OUTPUT.json]
DIR holds the unzipped APT_BASE.csv, FIX_BASE.csv, NAV_BASE.csv, AWY_BASE.csv, STAR_RTE.csv and DP_RTE.csv
(for example in subfolders x_APT, x_FIX, x_NAV, x_AWY, x_STAR, x_DP).
"""
import csv
import glob
import json
import os
import sys

src = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "navdata.json")


def rows(name):
    path = glob.glob(os.path.join(src, "**", name), recursive=True)
    if not path:
        sys.exit("missing " + name)
    with open(path[0], newline="", encoding="utf-8-sig") as f:
        yield from csv.DictReader(f)


def ll(r):
    try:
        return [round(float(r["LAT_DECIMAL"]), 4), round(float(r["LONG_DECIMAL"]), 4)]
    except (KeyError, ValueError):
        return None


apt, pt, awy, star, dp = {}, {}, {}, {}, {}
for r in rows("APT_BASE.csv"):
    p = ll(r)
    if not p:
        continue
    faa = r["ARPT_ID"].strip()
    icao = r.get("ICAO_ID", "").strip()
    apt[faa] = p
    if icao:
        apt[icao] = p
    elif len(faa) == 3 and faa.isalpha():
        apt["K" + faa] = p                        # contiguous US airports use K plus the FAA identifier
for name, key in (("FIX_BASE.csv", "FIX_ID"), ("NAV_BASE.csv", "NAV_ID")):
    for r in rows(name):
        p = ll(r)
        if p:
            lst = pt.setdefault(r[key].strip(), [])
            if p not in lst:
                lst.append(p)
for r in rows("AWY_BASE.csv"):
    s = r["AIRWAY_STRING"].split()
    if len(s) >= 2:
        awy.setdefault(r["AWY_ID"].strip(), []).append(s)


def procedures(name, codecol, dest):
    tmp = {}
    for r in rows(name):
        if r["ROUTE_PORTION_TYPE"].strip() != "BODY":
            continue
        code = r[codecol].strip().split(".")[-1]
        key = (code, r["ROUTE_NAME"].strip())
        tmp.setdefault(key, []).append((int(r["BODY_SEQ"] or 0), int(r["POINT_SEQ"] or 0), r["POINT"].strip()))
    for (code, route), pts in tmp.items():
        pts.sort()
        dest.setdefault(code, []).append({"rt": route, "pts": [p[2] for p in pts]})


procedures("STAR_RTE.csv", "STAR_COMPUTER_CODE", star)
procedures("DP_RTE.csv", "DP_COMPUTER_CODE", dp)
data = {"cycle": "2026-10-01", "apt": apt, "pt": pt, "awy": awy, "star": star, "dp": dp}
with open(out, "w") as f:
    json.dump(data, f, separators=(",", ":"))
print("wrote", out, "%.1f MB" % (os.path.getsize(out) / 1048576),
      "airports", len(apt), "fixes and navaids", len(pt), "airways", len(awy), "STARs", len(star), "DPs", len(dp))

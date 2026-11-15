# supertracker

A local ADS-B (and AIS) radio toolkit for a HackRF One on macOS: decodes aircraft at 1090 MHz, shows them on a live map, draws FAA filed
routes, keeps a 30 day position history, and can share to and receive from ADSBHub.

## Run

    python3 -m venv adsb_venv && ./adsb_venv/bin/pip install numpy solace-pubsubplus certifi
    brew install hackrf
    ./adsb_venv/bin/python adsb_map.py --lat YOUR_LAT --lon YOUR_LON      # then open http://127.0.0.1:8080

Most settings live in the Settings dialog (stored in `adsb_data/config.json`, private to you). Secrets (FAA SWIM user, password and queue,
ADSBHub station key) are stored in the macOS Keychain under the service name `adsb-map` and never in files.

## Files

- `adsb_map.py` the decoder, web server and page (map, list, settings)
- `adsb_config.py` settings and Keychain storage
- `swim_routes.py` FAA SWIM (SFDPS) consumer and filed route resolver
- `sbs_check.py` independent audit of the SBS output on port 30003
- `ais_map.py` AIS (ship) tracker; its coherent GMSK decoder follows the approach of AIS-Catcher (GPL-3), so the whole repository is released under the GPL-3.0 (see `LICENSE`)
- `adsb_data/build_navdata.py` builds `navdata.json` from the FAA NASR CSV files
- `adsb_data/build_world_airports.py` builds `world_airports.json` from OurAirports `airports.csv`

## Sharing and receiving

Both are off until you switch them on in Settings. Sharing sends your checked aircraft data to data.adsbhub.org:5001. Receiving reads
data.adsbhub.org:5002, shows other aircraft as dots, and tracks and saves those within the chosen radius of your receiver.
Received data is never put on the local SBS output and never sent back.

## Data sources

FAA NASR (public), FAA SWIM SCDS (needs your own subscription), OurAirports (public domain), ADSBHub (see their terms).

## License

GPL-3.0, see `LICENSE`.

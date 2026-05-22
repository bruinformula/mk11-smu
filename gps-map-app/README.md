# GPS Map App

Desktop parser app for mixed binary plus NMEA GNSS logs.

## What it does

- Opens noisy PuTTY-style captures that contain both binary garbage and NMEA sentences.
- Uses a Python parser to recover valid RMC fixes with checksum validation.
- Draws the recovered route on an embedded Leaflet map with OpenStreetMap tiles.
- Exports the track as GeoJSON or KML.

## Run it

1. Install dependencies:
   npm install
2. Start the app:
   npm start

Run those commands from the `gps-map-app` folder.

## Notes

- The parser uses the Python standard library only. By default the app calls `python3`.
- If your Python executable lives somewhere else, set `GPS_MAP_PYTHON` before launching the app.
- KML export is the easiest way to bring the recovered track into Google My Maps.

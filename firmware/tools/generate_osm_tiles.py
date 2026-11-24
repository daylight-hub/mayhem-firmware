#!/usr/bin/env python3
"""
generate_osm_tiles.py - build PortaPack /OSM/{z}/{x}/{y}.bmp detailed-map tiles
for a bounding box, from a slippy-map (XYZ) tile provider.

The PortaPack geomap reads 24-bit BMP tiles in the standard /OSM/{z}/{x}/{y}
layout. This script downloads tiles for a region, converts them to 24-bit BMP,
and writes that tree (default: VA / MD / DC / WV).

IMPORTANT - tile source policy:
  Do NOT point this at the public OpenStreetMap tile servers
  (tile.openstreetmap.org). Their usage policy forbids bulk downloading and
  the run will (rightly) be blocked. Use a provider whose terms allow bulk /
  offline tiles and supply its URL template via --url / TILE_URL_TEMPLATE,
  e.g. MapTiler, Thunderforest, Stadia, Geoapify (with your API key), or your
  own self-hosted renderer. You are responsible for complying with that
  provider's terms and attribution.

The URL template uses {z} {x} {y} placeholders, e.g.:
  https://api.maptiler.com/maps/streets-v2/{z}/{x}/{y}.png?key=YOUR_KEY

Resumable: existing .bmp tiles are skipped, so re-runs only fetch what's missing.
"""

import argparse
import io
import math
import os
import re
import sys
import time
import urllib.request
import urllib.error


def redact(url):
    """Hide the API key so it never lands in logs."""
    return re.sub(r'((?:key|apikey|api_key)=)[^&\s]+', r'\1REDACTED', url, flags=re.I)

try:
    from PIL import Image
except ImportError:
    sys.exit("Pillow is required: pip install Pillow  (CI installs python3-pil)")


def deg2num(lat_deg, lon_deg, zoom):
    """Standard slippy-map lat/lon -> tile x,y."""
    lat_rad = math.radians(lat_deg)
    n = 2 ** zoom
    x = int((lon_deg + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    # clamp into valid range
    x = max(0, min(n - 1, x))
    y = max(0, min(n - 1, y))
    return x, y


def tile_ranges(bbox, zoom):
    """Return inclusive (x0,x1,y0,y1) tile range covering bbox at zoom."""
    lon_min, lat_min, lon_max, lat_max = bbox
    x0, y0 = deg2num(lat_max, lon_min, zoom)  # top-left (north-west)
    x1, y1 = deg2num(lat_min, lon_max, zoom)  # bottom-right (south-east)
    return min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)


def count_tiles(bbox, zmin, zmax):
    total = 0
    for z in range(zmin, zmax + 1):
        x0, x1, y0, y1 = tile_ranges(bbox, z)
        total += (x1 - x0 + 1) * (y1 - y0 + 1)
    return total


def fetch(url, user_agent, timeout, retries, referer):
    last = None
    headers = {"User-Agent": user_agent}
    if referer:
        # Some providers restrict a key to specific HTTP origins; sending a
        # matching Referer makes a server-side request look allowed.
        headers["Referer"] = referer
        headers["Origin"] = referer
    for attempt in range(retries):
        try:
            req = urllib.request.Request(url, headers=headers)
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return r.read()
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError) as e:
            last = e
            time.sleep(1.5 * (attempt + 1))  # backoff
    # NB: redact() keeps the API key out of the log.
    raise RuntimeError(f"failed after {retries} tries: {redact(url)} ({last})")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=os.environ.get("TILE_URL_TEMPLATE", ""),
                    help="XYZ URL template with {z} {x} {y}. Or set TILE_URL_TEMPLATE.")
    ap.add_argument("--out", default="sdcard/OSM", help="output /OSM root")
    # Default bbox: VA / MD / DC / WV  (lon_min, lat_min, lon_max, lat_max)
    # covers WV's northern panhandle (~40.7) down to VA's south border (~36.5),
    # and the Atlantic side of the Delmarva (~-74.9) west to WV (~-83.7).
    ap.add_argument("--bbox", default="-83.7,36.5,-74.9,40.7",
                    help="lon_min,lat_min,lon_max,lat_max")
    ap.add_argument("--zmin", type=int, default=int(os.environ.get("OSM_ZMIN", 6)))
    ap.add_argument("--zmax", type=int, default=int(os.environ.get("OSM_ZMAX", 11)))
    ap.add_argument("--delay", type=float, default=float(os.environ.get("OSM_DELAY", 0.2)),
                    help="seconds between requests (be polite to the provider)")
    ap.add_argument("--max-tiles", type=int, default=int(os.environ.get("OSM_MAX_TILES", 30000)),
                    help="safety cap; abort if the job would exceed this")
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--retries", type=int, default=3)
    ap.add_argument("--user-agent",
                    default=os.environ.get("OSM_USER_AGENT",
                                           "portapack-mayhem-osm-tiler/1.0 (personal build)"))
    ap.add_argument("--referer", default=os.environ.get("TILE_REFERER", ""),
                    help="Referer/Origin to send. Set this to an origin your key "
                         "allows if the provider restricts the key to HTTP origins.")
    args = ap.parse_args()

    if not args.url:
        sys.exit("No tile URL template. Pass --url or set TILE_URL_TEMPLATE "
                 "(a provider whose terms allow bulk tiles; NOT tile.openstreetmap.org).")
    if "openstreetmap.org" in args.url:
        sys.exit("Refusing tile.openstreetmap.org: bulk download violates OSM's tile policy. "
                 "Use a provider that permits it.")

    bbox = tuple(float(v) for v in args.bbox.split(","))
    if len(bbox) != 4:
        sys.exit("--bbox needs lon_min,lat_min,lon_max,lat_max")

    planned = count_tiles(bbox, args.zmin, args.zmax)
    print(f"bbox={bbox} zoom={args.zmin}..{args.zmax} -> {planned} tiles")
    if planned > args.max_tiles:
        sys.exit(f"planned {planned} tiles exceeds --max-tiles {args.max_tiles}. "
                 f"Lower --zmax or raise the cap deliberately.")

    done = skipped = failed = 0
    for z in range(args.zmin, args.zmax + 1):
        x0, x1, y0, y1 = tile_ranges(bbox, z)
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                out_path = os.path.join(args.out, str(z), str(x), f"{y}.bmp")
                if os.path.exists(out_path):
                    skipped += 1
                    continue
                url = args.url.format(z=z, x=x, y=y)
                try:
                    data = fetch(url, args.user_agent, args.timeout, args.retries, args.referer)
                    img = Image.open(io.BytesIO(data)).convert("RGB")  # 24-bit
                    os.makedirs(os.path.dirname(out_path), exist_ok=True)
                    img.save(out_path, format="BMP")
                    done += 1
                    if done % 100 == 0:
                        print(f"  {done} tiles written (z{z})...")
                    time.sleep(args.delay)
                except Exception as e:  # noqa: BLE001 - keep going, report at end
                    failed += 1
                    print(f"  WARN {z}/{x}/{y}: {e}", file=sys.stderr)

    print(f"done: {done} written, {skipped} already present, {failed} failed")
    if failed and done == 0:
        sys.exit("all tiles failed - check the URL template / API key / provider terms")


if __name__ == "__main__":
    main()

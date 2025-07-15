import os
import math
import time
import argparse
import requests
from tqdm import tqdm


TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"

def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int(
        (1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n
    )
    return xtile, ytile

def download_tiles(min_lat, min_lon, max_lat, max_lon, zoom_levels, output_dir="tiles"):
    os.makedirs(output_dir, exist_ok=True)

    for z in zoom_levels:
        print(f"Zoom level {z}:")
        x_start, y_start = deg2num(max_lat, min_lon, z)  # upper-left
        x_end, y_end = deg2num(min_lat, max_lon, z)      # lower-right

        for x in tqdm(range(x_start, x_end + 1), desc=f"Zoom {z}"):
            for y in range(y_start, y_end + 1):
                tile_url = TILE_URL.format(z=z, x=x, y=y)
                tile_path = os.path.join(output_dir, str(z), str(x))
                tile_file = os.path.join(tile_path, f"{y}.png")

                if not os.path.exists(tile_file):
                    os.makedirs(tile_path, exist_ok=True)
                    try:
                        time.sleep(0.2)
                        #r = requests.get(tile_url, timeout=10)
                        headers = {"User-Agent": "Mozilla/5.0 (Raspberry Pi; Offline Tile Downloader)"}
                        r = requests.get(tile_url, headers=headers, timeout=10)
                        #r = requests.get(tile_url, headers={'User-Agent': 'Mozilla/5.0'}, timeout=10)
                        if r.status_code == 200:
                            with open(tile_file, "wb") as f:
                                f.write(r.content)
                        else:
                            print(f"Failed: {tile_url} ({r.status_code})")
                    except Exception as e:
                        print(f"Error downloading {tile_url}: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Download OSM tiles for offline use")
    parser.add_argument("--bbox", nargs=4, type=float, metavar=('MIN_LAT', 'MIN_LON', 'MAX_LAT', 'MAX_LON'), required=True, help="Bounding box")
    parser.add_argument("--zoom", nargs="+", type=int, required=True, help="Zoom levels (e.g. 12 13 14)")
    parser.add_argument("--output", default="tiles", help="Output directory for tiles")

    args = parser.parse_args()
    download_tiles(args.bbox[0], args.bbox[1], args.bbox[2], args.bbox[3], args.zoom, args.output)


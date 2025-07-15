import os
import math
import argparse
import requests
import time
from tqdm import tqdm

TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
HEADERS = {"User-Agent": "Mozilla/5.0 (Raspberry Pi; Offline Tile Downloader)"}

def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2 ** zoom
    x = int((lon_deg + 180.0) / 360.0 * n)
    y = int((1 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2 * n)
    return x, y

def download_tile(z, x, y, output_dir):
    tile_url = TILE_URL.format(z=z, x=x, y=y)
    tile_dir = os.path.join(output_dir, str(z), str(x))
    tile_path = os.path.join(tile_dir, f"{y}.png")
    if os.path.exists(tile_path):
        return  # already exists

    os.makedirs(tile_dir, exist_ok=True)

    for attempt in range(3):  # Retry logic
        try:
            r = requests.get(tile_url, headers=HEADERS, timeout=10)
            if r.status_code == 200:
                with open(tile_path, "wb") as f:
                    f.write(r.content)
                return
            else:
                print(f"[{r.status_code}] Failed: {tile_url}")
        except Exception as e:
            print(f"[ERROR] {tile_url}: {e}")
        time.sleep(1)  # Wait before retry

def download_tiles(min_lat, min_lon, max_lat, max_lon, zoom_levels, output_dir="tiles"):
    for z in zoom_levels:
        print(f"\n📡 Zoom level {z}")
        #x_start, y_start = deg2num(max_lat, min_lon, z)  # top-left
        #x_end, y_end = deg2num(min_lat, max_lon, z)      # bottom-right
        x1, y1 = deg2num(min_lat, min_lon, z)
        x2, y2 = deg2num(max_lat, max_lon, z)
        x_start, x_end = sorted([x1, x2])
        y_start, y_end = sorted([y1, y2])
        print(f"Zoom {z}: X from {x_start} to {x_end}, Y from {y_start} to {y_end}")

        total = (x_end - x_start + 1) * (y_end - y_start + 1)
        with tqdm(total=total, desc=f"Downloading Z{z}") as pbar:
            for x in range(x_start, x_end + 1):
                for y in range(y_start, y_end + 1):
                    download_tile(z, x, y, output_dir)
                    pbar.update(1)
                    time.sleep(0.2)  # Optional: throttle to avoid 418

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Offline Tile Downloader")
    parser.add_argument("--bbox", nargs=4, type=float, metavar=('MIN_LAT', 'MIN_LON', 'MAX_LAT', 'MAX_LON'), required=True, help="Bounding box")
    parser.add_argument("--zoom", nargs="+", type=int, required=True, help="Zoom levels (e.g. 14 15 16)")
    parser.add_argument("--output", default="tiles", help="Directory to save tiles")

    args = parser.parse_args()
    download_tiles(args.bbox[0], args.bbox[1], args.bbox[2], args.bbox[3], args.zoom, args.output)


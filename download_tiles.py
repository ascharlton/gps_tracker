import os
import math
import argparse
import requests
import time
import sqlite3
from tqdm import tqdm

# Default tile URLs
TILE_URLS = {
    "osm": "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
    "satellite": "http://www.google.cn/maps/vt?lyrs=s@189&gl=cn&x={x}&y={y}&z={z}",
    "dark": "http://a.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png"
}
#"satellite": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"

HEADERS = {"User-Agent": "Mozilla/5.0 (Raspberry Pi; Offline Tile Downloader)"}

def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2 ** zoom
    x = int((lon_deg + 180.0) / 360.0 * n)
    y = int((1 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2 * n)
    return x, y

def download_tile(z, x, y, output_dir, tile_url):
    tile_dir = os.path.join(output_dir, str(z), str(x))
    os.makedirs(tile_dir, exist_ok=True)
    tile_path = os.path.join(tile_dir, f"{y}.png")

    if os.path.exists(tile_path):
        return tile_path  # already downloaded

    for attempt in range(5):  # more retries
        try:
            r = requests.get(tile_url, headers=HEADERS, timeout=15)
            if r.status_code == 200:
                with open(tile_path, "wb") as f:
                    f.write(r.content)
                return tile_path
            else:
                print(f"[{r.status_code}] Failed: {tile_url}")
        except Exception as e:
            print(f"[ERROR] {tile_url}: {e}")
        time.sleep(1)  # wait before retry
    print(f"[FAIL] Could not download tile {z}/{x}/{y}")
    return None

def init_mbtiles(mbtiles_path, name="Offline Map"):
    if os.path.exists(mbtiles_path):
        os.remove(mbtiles_path)

    conn = sqlite3.connect(mbtiles_path)
    cur = conn.cursor()
    cur.execute("CREATE TABLE tiles (zoom_level INTEGER, tile_column INTEGER, tile_row INTEGER, tile_data BLOB)")
    cur.execute("CREATE TABLE metadata (name TEXT, value TEXT)")
    cur.execute("CREATE UNIQUE INDEX tile_index ON tiles (zoom_level, tile_column, tile_row)")
    metadata = [
        ("name", name),
        ("type", "baselayer"),
        ("version", "1"),
        ("description", "Downloaded tiles"),
        ("format", "png")
    ]
    cur.executemany("INSERT INTO metadata (name, value) VALUES (?, ?)", metadata)
    conn.commit()
    return conn

def insert_tile(conn, z, x, y, tile_path):
    if tile_path and os.path.exists(tile_path):
        with open(tile_path, "rb") as f:
            tile_data = f.read()
        tms_y = (2 ** z - 1) - y
        conn.execute("INSERT OR REPLACE INTO tiles (zoom_level, tile_column, tile_row, tile_data) VALUES (?, ?, ?, ?)",
                     (z, x, tms_y, tile_data))

def download_tiles(min_lat, min_lon, max_lat, max_lon, zoom_levels, tile_type, output_dir=None, mbtiles=None):
    # Choose folder names compatible with your Flask setup
    if not output_dir:
        output_dir = f"tiles_{tile_type}"  # tiles_osm or tiles_satellite or dark
    os.makedirs(output_dir, exist_ok=True)

    tile_url_template = TILE_URLS[tile_type]
    conn = init_mbtiles(mbtiles, name=f"{tile_type.upper()} Tiles") if mbtiles else None

    for z in zoom_levels:
        print(f"\n📡 Zoom level {z}")
        x1, y1 = deg2num(min_lat, min_lon, z)
        x2, y2 = deg2num(max_lat, max_lon, z)
        x_start, x_end = sorted([x1, x2])
        y_start, y_end = sorted([y1, y2])
        print(f"Zoom {z}: X {x_start}-{x_end}, Y {y_start}-{y_end}")

        total = (x_end - x_start + 1) * (y_end - y_start + 1)
        with tqdm(total=total, desc=f"Downloading Z{z}") as pbar:
            for x in range(x_start, x_end + 1):
                for y in range(y_start, y_end + 1):
                    tile_url = tile_url_template.format(z=z, x=x, y=y)
                    tile_path = download_tile(z, x, y, output_dir, tile_url)
                    if conn:
                        insert_tile(conn, z, x, y, tile_path)
                    pbar.update(1)
                    time.sleep(0.05)

    if conn:
        conn.commit()
        conn.close()
        print(f"✅ MBTiles saved to {mbtiles}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Offline Tile Downloader")
    parser.add_argument("--bbox", nargs=4, type=float, metavar=('MIN_LAT', 'MIN_LON', 'MAX_LAT', 'MAX_LON'), required=True, help="Bounding box")
    parser.add_argument("--zoom", nargs="+", type=int, required=True, help="Zoom levels (e.g. 14 15 16)")
    parser.add_argument("--type", choices=["osm", "satellite", "dark"], default="osm", help="Tile source (osm or satellite or dark)")
    parser.add_argument("--output", help="Directory to save tiles (default: tiles_osm or tiles_satellite or tiles_dark)")
    parser.add_argument("--mbtiles", help="Optional MBTiles output file")
    args = parser.parse_args()

    download_tiles(args.bbox[0], args.bbox[1], args.bbox[2], args.bbox[3],
                   args.zoom, args.type, args.output, args.mbtiles)


from flask import Flask, jsonify, send_from_directory, render_template
from gps3 import gps3
import csv
import os
import time
import datetime
import threading

app = Flask(__name__, static_folder='static', static_url_path='/static')

csv_file = "track.csv"
lock = threading.Lock()
last_logged_time = 0
log_filename = datetime.datetime.now().strftime("raw_gps_log_%Y%m%d_%H%M%S.json")

# Shared state
latest_data = {
    'lat': None,
    'lon': None,
    'speed': None,
    'track': None,
    'epx': None,
    'epy': None,
    'epv': None,
    'eps': None
}

def gps_polling_thread():
    global last_logged_time

    gps_socket = gps3.GPSDSocket()
    data_stream = gps3.DataStream()
    gps_socket.connect()
    gps_socket.watch()

    print("[INFO] GPS polling thread started.")

    for new_data in gps_socket:
        if not new_data:
            continue

        #print(f"[RAW] {new_data}")
        with open(log_filename, "a") as f:
            f.write(new_data + "\n")

        try:
            data_stream.unpack(new_data)
        except Exception as e:
            #print(f"[WARN] Failed to unpack GPSD data: {e}")
            continue

        tpv = data_stream.TPV
        lat = tpv.get("lat", "n/a")
        lon = tpv.get("lon", "n/a")
        #print(f"[GPS] lat: {lat}, lon: {lon}")

        if lat != "n/a" and lon != "n/a":
            # Update shared latest_data
            latest_data.update({
                'lat': lat,
                'lon': lon,
                'speed': tpv.get("speed", None),  # m/s
                'track': tpv.get("track", None),  # degrees
                'epx': tpv.get("epx", None),
                'epy': tpv.get("epy", None),
                'epv': tpv.get("epv", None),
                'eps': tpv.get("eps", None)
            })

            now = time.time()
            if now - last_logged_time > 5:
                with lock:
                    with open(csv_file, mode="a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow([
                            time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)),
                            lat,
                            lon,
                            latest_data['speed'],
                            latest_data['track'],
                            latest_data['epx'],
                            latest_data['epy'],
                            latest_data['epv'],
                            latest_data['eps']
                        ])
                last_logged_time = now

@app.route('/')
#def serve_index():
def index():
    return render_template('index.html')

@app.route('/track.csv')
def download_track():
    return send_from_directory('.', csv_file, as_attachment=True)

@app.route('/track')
def serve_track_viewer():
    return send_from_directory('', 'track.html')

@app.route('/gps')
def get_gps():
    return jsonify(latest_data)

#@app.route('/tiles/<int:z>/<int:x>/<int:y>.png')
#def serve_tile(z, x, y):
#    tile_path = f"{z}/{x}/{y}.png"
#    print(f"[TILE REQUEST] /tiles/{tile_path}")
#    return send_from_directory("tiles", tile_path)

# Serve OSM tiles
@app.route('/tiles_osm/<int:z>/<int:x>/<int:y>.png')
def tiles_osm(z, x, y):
    return send_from_directory("tiles_osm", f"{z}/{x}/{y}.png")

# Serve Satellite tiles
@app.route('/tiles_satellite/<int:z>/<int:x>/<int:y>.png')
def tiles_satellite(z, x, y):
    return send_from_directory("tiles_sat", f"{z}/{x}/{y}.png")


if __name__ == '__main__':
    # Create CSV with new headers if needed
    if not os.path.exists(csv_file):
        with open(csv_file, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", "latitude", "longitude",
                "speed (m/s)", "heading (°)",
                "epx", "epy", "epv", "eps"
            ])

    thread = threading.Thread(target=gps_polling_thread, daemon=True)
    thread.start()

    print("[INFO] Flask server running at http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)


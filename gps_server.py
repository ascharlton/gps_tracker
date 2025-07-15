from flask import Flask, jsonify, send_from_directory
from gps3 import gps3
import csv
import os
import time
import threading

app = Flask(__name__, static_url_path='')

csv_file = "track.csv"
lock = threading.Lock()
last_logged_time = 0

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

        print(f"[RAW] {new_data}")
        with open("raw_gps_log.json", "a") as f:
            f.write(new_data + "\n")

        try:
            data_stream.unpack(new_data)
        except Exception as e:
            print(f"[WARN] Failed to unpack GPSD data: {e}")
            continue

        tpv = data_stream.TPV
        lat = tpv.get("lat", "n/a")
        lon = tpv.get("lon", "n/a")

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
def serve_index():
    return send_from_directory('', 'index.html')

@app.route('/track.csv')
def download_track():
    return send_from_directory('.', csv_file, as_attachment=True)

@app.route('/track')
def serve_track_viewer():
    return send_from_directory('', 'track.html')

@app.route('/gps')
def get_gps():
    return jsonify(latest_data)

@app.route('/tiles/<int:z>/<int:x>/<int:y>.png')
def serve_tile(z, x, y):
    return send_from_directory("tiles", f"{z}/{x}/{y}.png")


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
    app.run(host='0.0.0.0', port=5000, debug=True)


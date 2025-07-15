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

# Shared GPS state
latest_lat = None
latest_lon = None

def gps_polling_thread():
    global latest_lat, latest_lon, last_logged_time

    gps_socket = gps3.GPSDSocket()
    data_stream = gps3.DataStream()
    gps_socket.connect()
    gps_socket.watch()

    print("[INFO] GPS polling thread started.")

    for new_data in gps_socket:
        if not new_data:
            continue

        try:
            data_stream.unpack(new_data)
        except Exception as e:
            print(f"[WARN] Failed to unpack GPSD data: {e}")
            continue

        lat = data_stream.TPV.get("lat", "n/a")
        lon = data_stream.TPV.get("lon", "n/a")

        if lat != "n/a" and lon != "n/a":
            latest_lat = lat
            latest_lon = lon

            now = time.time()
            if now - last_logged_time > 10:
                with lock:
                    with open(csv_file, mode="a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow([
                            time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)),
                            lat,
                            lon
                        ])
                last_logged_time = now

@app.route('/')
def serve_index():
    return send_from_directory('', 'index.html')

@app.route('/gps')
def get_gps():
    if latest_lat is not None and latest_lon is not None:
        return jsonify({'lat': latest_lat, 'lon': latest_lon})
    return jsonify({'lat': None, 'lon': None})

if __name__ == '__main__':
    # Ensure CSV exists
    if not os.path.exists(csv_file):
        with open(csv_file, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "latitude", "longitude"])

    # Start GPS polling in background
    thread = threading.Thread(target=gps_polling_thread, daemon=True)
    thread.start()

    print("[INFO] Flask server running at http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000, debug=True)


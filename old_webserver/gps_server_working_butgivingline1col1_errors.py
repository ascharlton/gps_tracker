



from flask import Flask, jsonify, send_from_directory
from gps3 import gps3
import csv
import os
import time
from threading import Lock

app = Flask(__name__, static_url_path='')

csv_file = "track.csv"
lock = Lock()
last_logged_time = 0

print("[INFO] Initializing GPS socket...")

try:
    gps_socket = gps3.GPSDSocket()
    data_stream = gps3.DataStream()
    gps_socket.connect()
    gps_socket.watch()
    print("[INFO] GPSD socket connected.")
except Exception as e:
    print("[ERROR] Failed to connect to GPSD:", e)
    exit(1)

# Create CSV file with headers if not present
if not os.path.exists(csv_file):
    with open(csv_file, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "latitude", "longitude"])
    print("[INFO] CSV file initialized.")

@app.route('/')
def serve_index():
    return send_from_directory('', 'index.html')


@app.route('/gps')
def get_gps():
    global last_logged_time
    try:
        for new_data in gps_socket:
            if not new_data:
                continue

            try:
                data_stream.unpack(new_data)
            except Exception as e:
                print("[WARN] Failed to unpack GPSD data:", e)
                continue  # skip to next packet

            lat = data_stream.TPV.get('lat', 'n/a')
            lon = data_stream.TPV.get('lon', 'n/a')

            if lat != 'n/a' and lon != 'n/a':
                now = time.time()
                if now - last_logged_time > 10:
                    with lock:
                        with open(csv_file, mode='a', newline='') as f:
                            writer = csv.writer(f)
                            writer.writerow([
                                time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)),
                                lat,
                                lon
                            ])
                    last_logged_time = now
                return jsonify({'lat': lat, 'lon': lon})

    except Exception as e:
        print("[ERROR] Exception in /gps route:", e)

    return jsonify({'lat': None, 'lon': None})



# Run the Flask server
if __name__ == '__main__':
    print("[INFO] Starting Flask server on http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000, debug=True)


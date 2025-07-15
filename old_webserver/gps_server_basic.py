from flask import Flask, jsonify, send_from_directory
import gps

session = gps.gps(mode=gps.WATCH_ENABLE)
app = Flask(__name__, static_url_path='')

@app.route('/')
def root():
    return send_from_directory('', 'index.html')

@app.route('/gps')
def get_gps():
    report = session.next()
    while report['class'] != 'TPV':
        report = session.next()
    latitude = getattr(report, 'lat', None)
    longitude = getattr(report, 'lon', None)
    return jsonify({'lat': latitude, 'lon': longitude})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)


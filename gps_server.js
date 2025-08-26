// gps_server.js
const express = require('express');
const http = require('http');
const path = require('path');
const net = require('net');
const fs = require('fs');
const readline = require('readline');

const app = express();
const PORT = 5001;

// --- Serve static files (frontend + tiles) ---
app.use('/static', express.static(path.join(__dirname, 'static')));
app.use('/tiles_osm', express.static(path.join(__dirname, 'tiles_osm')));
app.use('/tiles_satellite', express.static(path.join(__dirname, 'tiles_satellite')));
app.use(express.static(__dirname)); // serve index.html at "/"

// --- GPS state ---
let latestFix = { lat: null, lon: null, time: null };

// Ensure tracks dir exists
const tracksDir = path.join(__dirname, 'tracks');
if (!fs.existsSync(tracksDir)) fs.mkdirSync(tracksDir);

// --- Logging helper ---
function appendTrack(fix) {
  if (!fix.lat || !fix.lon) return;
  const dateStr = fix.time ? fix.time.slice(0, 10) : new Date().toISOString().slice(0, 10); // YYYY-MM-DD
  const filePath = path.join(tracksDir, `${dateStr}.csv`);
  const line = `${fix.time},${fix.lat},${fix.lon}\n`;
  fs.appendFile(filePath, line, (err) => {
    if (err) console.error('[Track] Write error:', err.message);
  });
}

// --- GPSD socket ---
function startGpsStream() {
  const client = net.createConnection({ port: 2947, host: '127.0.0.1' }, () => {
    console.log('[GPSD] Connected to gpsd');
    client.write('?WATCH={"enable":true,"json":true};\n');
  });

  client.on('data', (data) => {
    try {
      const lines = data.toString().split('\n');
      for (const line of lines) {
        if (!line.trim()) continue;
        const json = JSON.parse(line);
        if (json.class === 'TPV' && json.lat && json.lon) {
          latestFix = {
            lat: json.lat,
            lon: json.lon,
            time: json.time || new Date().toISOString()
          };
          console.log(`[GPSD] Fix: ${latestFix.lat}, ${latestFix.lon}`);
          appendTrack(latestFix);
        }
      }
    } catch (err) {
      console.error('[GPSD] Parse error:', err.message);
    }
  });

  client.on('error', (err) => {
    console.error('[GPSD] Connection error:', err.message);
    setTimeout(startGpsStream, 5000);
  });

  client.on('end', () => {
    console.warn('[GPSD] Disconnected. Reconnecting...');
    setTimeout(startGpsStream, 5000);
  });
}

// Start GPS stream
startGpsStream();

// --- API: live GPS ---
app.get('/gps', (req, res) => {
  res.json(latestFix);
});

// --- API: load a day's track ---
app.get('/track/:date', async (req, res) => {
  const date = req.params.date; // expect YYYY-MM-DD
  const filePath = path.join(tracksDir, `${date}.csv`);

  if (!fs.existsSync(filePath)) {
    return res.status(404).json({ error: `No track for ${date}` });
  }

  const points = [];
  const rl = readline.createInterface({
    input: fs.createReadStream(filePath),
    crlfDelay: Infinity
  });

  for await (const line of rl) {
    const [time, lat, lon] = line.split(',');
    if (lat && lon) {
      points.push({ time, lat: parseFloat(lat), lon: parseFloat(lon) });
    }
  }

  res.json(points);
});

// --- Start HTTP server ---
http.createServer(app).listen(PORT, '0.0.0.0', () => {
  console.log(`[INFO] Server running at http://0.0.0.0:${PORT}`);
});


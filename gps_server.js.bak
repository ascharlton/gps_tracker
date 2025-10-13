// gps_server.js
const express = require("express");
const fs = require("fs");
const path = require("path");
const http = require("http");
const socketIo = require("socket.io");
const { spawn } = require("child_process");

const app = express();
const server = http.createServer(app);
const io = socketIo(server);

const PORT = 5000;
const LOG_DIR = path.join(__dirname, "logs");
if (!fs.existsSync(LOG_DIR)) fs.mkdirSync(LOG_DIR);

// Serve static files (map, leaflet, icons, tiles, etc.)
app.use("/static", express.static(path.join(__dirname, "static")));
app.use("/tiles_osm", express.static(path.join(__dirname, "tiles_osm")));
app.use("/tiles_satellite", express.static(path.join(__dirname, "tiles_satellite")));

app.get("/", (req, res) => {
  res.sendFile(path.join(__dirname, "index.html"));
});
app.get('/favicon.ico', (req, res) => {
  res.sendFile(path.join(__dirname, 'static', 'favicon.ico'));
});

// --- Daily track + raw log filenames ---
function getDateString() {
  return new Date().toISOString().split("T")[0];
}
function getTrackFilename() {
  return path.join(LOG_DIR, `track_${getDateString()}.csv`);
}
function getRawFilename() {
  return path.join(LOG_DIR, `raw_${getDateString()}.log`);
}

// Track endpoint (review past tracks)
app.get("/track/:date", (req, res) => {
  const filename = path.join(LOG_DIR, `track_${req.params.date}.csv`);
  if (fs.existsSync(filename)) res.sendFile(filename);
  else res.status(404).send("Track file not found");
});

// --- GPSD stream ---
const gpsd = spawn("gpspipe", ["-w"]);
gpsd.stdout.on("data", (data) => {
  const lines = data.toString().split("\n").filter(Boolean);
  for (let line of lines) {
    try {
      const msg = JSON.parse(line);
      if (msg.class === "TPV" && msg.lat && msg.lon) {
        const gpsData = {
          lat: msg.lat,
          lon: msg.lon,
          speed: msg.speed || 0,
          track: msg.track || 0,
          time: msg.time || new Date().toISOString(),
        };

        // --- Broadcast via WebSocket ---
        io.emit("gps", gpsData);

        // --- Append to daily track ---
        fs.appendFileSync(
          getTrackFilename(),
          `${gpsData.time},${gpsData.lat},${gpsData.lon},${gpsData.speed},${gpsData.track}\n`
        );

        // --- Raw log ---
        fs.appendFileSync(getRawFilename(), line + "\n");
	console.log("printed to raw log");
      }
    } catch (err) {
      fs.appendFileSync(getRawFilename(), "ERROR parsing line: " + line + "\n");
    }
  }
});

gpsd.stderr.on("data", (data) => {
  console.error("gpspipe error:", data.toString());
});

// --- Start server ---
server.listen(PORT, "0.0.0.0", () => {
  console.log(`[INFO] Server running at http://0.0.0.0:${PORT}`);
});


// gps_server.js
const express = require("express");
const fs = require("fs");
const path = require("path");
const http = require("http");
const socketIo = require("socket.io");
const { spawn } = require("child_process");
//const chalk = require("chalk");

const app = express();
const server = http.createServer(app);
const io = socketIo(server);

const PORT = 5000;
const LOG_DIR = path.join(__dirname, "logs");
if (!fs.existsSync(LOG_DIR)) fs.mkdirSync(LOG_DIR);

// Serve static files
app.use("/static", express.static(path.join(__dirname, "static")));
app.use("/tiles_osm", express.static(path.join(__dirname, "tiles_osm")));
app.use("/tiles_satellite", express.static(path.join(__dirname, "tiles_satellite")));

app.get("/", (req, res) => {
  res.sendFile(path.join(__dirname, "index.html"));
});
app.get("/favicon.ico", (req, res) => {
  res.sendFile(path.join(__dirname, "static", "favicon.ico"));
});

// --- Daily filenames ---
function getDateString() {
  return new Date().toISOString().split("T")[0];
}
function getTrackFilename() {
  return path.join(LOG_DIR, `track_${getDateString()}.csv`);
}
function getRawFilename() {
  return path.join(LOG_DIR, `raw_${getDateString()}.log`);
}

// --- Track endpoint ---
app.get("/track/:date", (req, res) => {
  const filename = path.join(LOG_DIR, `track_${req.params.date}.csv`);
  if (fs.existsSync(filename)) res.sendFile(filename);
  else res.status(404).send("Track file not found");
});

// --- Raw log endpoint ---
app.get("/raw/:date", (req, res) => {
  const filename = path.join(LOG_DIR, `raw_${req.params.date}.log`);
  if (!fs.existsSync(filename)) return res.json([]);

  const lines = fs.readFileSync(filename, "utf8").split("\n").filter(Boolean);
  const points = [];
  for (let line of lines) {
    try {
      const msg = JSON.parse(line);
      if (msg.class === "TPV" && msg.lat && msg.lon) {
        points.push({
          lat: msg.lat,
          lon: msg.lon,
          speed: msg.speed || 0,
          track: msg.track || 0,
          time: msg.time || new Date().toISOString(),
        });
      }
    } catch {
      // Ignore malformed lines
    }
  }
  res.json(points);
});

// --- GPSD stream ---
const gpsd = spawn("gpspipe", ["-w"]);
gpsd.stdout.on("data", (data) => {
  const lines = data.toString().split("\n").filter(Boolean);
  for (let line of lines) {
    try {
      const msg = JSON.parse(line);

      // --- Handle TPV messages ---
      if (msg.class === "TPV") {
        if (msg.mode < 2) {
          console.warn(`[WARN] No GPS fix (mode=${msg.mode || 0})`);
        } else {
          const gpsData = {
            lat: msg.lat,
            lon: msg.lon,
            speed: msg.speed || 0,
            track: msg.track || 0,
            time: msg.time || new Date().toISOString(),
          };

          // Log accuracy if available
          if (msg.epx && msg.epy) {
            const horizAcc = Math.sqrt(msg.epx ** 2 + msg.epy ** 2);
            if (horizAcc > 10) {
              console.warn(`[WARN] Low accuracy: ±${horizAcc.toFixed(1)} m`);
            }
          }

          io.emit("gps", gpsData);

          fs.appendFileSync(
            getTrackFilename(),
            `${gpsData.time},${gpsData.lat},${gpsData.lon},${gpsData.speed},${gpsData.track}\n`
          );

          fs.appendFileSync(getRawFilename(), line + "\n");
        }
      }

      // --- Handle SKY messages (optional, satellite count) ---
      if (msg.class === "SKY" && Array.isArray(msg.satellites)) {
        const used = msg.satellites.filter(s => s.used).length;
        const total = msg.satellites.length;
        console.log(`[INFO] Satellites: ${used}/${total} in use`);
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


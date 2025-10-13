// gps_server.js — PostgreSQL version

const express = require("express");
const fs = require("fs");
const path = require("path");
const http = require("http");
const socketIo = require("socket.io");
const { spawn } = require("child_process");
const { Pool } = require("pg");

const app = express();
const server = http.createServer(app);
const io = socketIo(server);

const PORT = 5000;
const DEBUG = process.argv.includes("DEBUG=1") || process.env.DEBUG === "1";

// --- PostgreSQL connection ---
const pool = new Pool({
  user: "pi",
  host: "localhost",
  database: "gps_tracker",
  password: "ch1rlt4n", // change this
  port: 5432,
});

pool.connect()
  .then(() => console.log("[INFO] Connected to PostgreSQL"))
  .catch(err => console.error("[ERROR] DB connection failed:", err.message));

// --- Serve static assets ---
app.use("/static", express.static(path.join(__dirname, "static")));
app.use("/tiles_osm", express.static(path.join(__dirname, "tiles_osm")));
app.use("/tiles_satellite", express.static(path.join(__dirname, "tiles_satellite")));
app.use("/tiles_dark", express.static(path.join(__dirname, "tiles_dark")));

app.get("/", (req, res) => {
  res.sendFile(path.join(__dirname, "index.html"));
});
app.get("/favicon.ico", (req, res) => {
  res.sendFile(path.join(__dirname, "static", "favicon.ico"));
});

// --- REST endpoints using Postgres ---
app.get("/raw/:date", async (req, res) => {
  try {
    const date = req.params.date;
    const { rows } = await pool.query(`
      SELECT message
      FROM gps_raw
      WHERE DATE(timestamp) = $1
      ORDER BY timestamp ASC
    `, [date]);

    const points = rows
      .map(r => r.message)
      .filter(m => m.class === "TPV" && m.lat && m.lon)
      .map(m => ({
        lat: m.lat,
        lon: m.lon,
        speed: m.speed || 0,
        track: m.track || 0,
        time: m.time || new Date().toISOString(),
      }));

    res.json(points);
  } catch (err) {
    console.error("[ERROR] /raw query failed:", err.message);
    res.status(500).json({ error: "DB query failed" });
  }
});

app.get("/track/:date", async (req, res) => {
  try {
    const date = req.params.date;
    const { rows } = await pool.query(`
      SELECT timestamp, lat, lon, speed, track
      FROM gps_points
      WHERE DATE(timestamp) = $1
      ORDER BY timestamp ASC
    `, [date]);
    res.json(rows);
  } catch (err) {
    console.error("[ERROR] /track query failed:", err.message);
    res.status(500).json({ error: "DB query failed" });
  }
});


// --- get gps_raw record count ---
app.get("/count/raw", async (req, res) => {
  try {
    const { rows } = await pool.query("SELECT COUNT(*) FROM gps_raw");
    res.json({ count: Number(rows[0].count) });
  } catch (err) {
    console.error("[ERROR] /count/raw failed:", err.message);
    res.status(500).json({ error: "DB query failed" });
  }
});

// --- GPSD live stream ---
const gpsd = spawn("gpspipe", ["-w"]);

let lastFixMode = 0;
let lastAccuracyWarn = false;
let lastSatInfo = { used: 0, total: 0 };

gpsd.stdout.on("data", async (data) => {
  const lines = data.toString().split("\n").filter(Boolean);
  for (let line of lines) {
    try {
      const msg = JSON.parse(line);

      if (DEBUG) console.log("[DEBUG]", msg.class, msg.mode || "");

      // --- Store every message in gps_raw ---
      await pool.query(
        "INSERT INTO gps_raw (message) VALUES ($1)",
        [msg]
      );
      // Notify clients that a new record was added
      io.emit("raw_count_update");

      // --- Handle TPV messages ---
      if (msg.class === "TPV") {
        const mode = msg.mode || 0;

        // Detect fix status change
        if (!DEBUG && mode !== lastFixMode) {
          if (mode < 2) {
            console.warn(`[WARN] GPS lost — no fix (mode=${mode})`);
          } else if (mode === 2) {
            console.log(`[INFO] GPS 2D fix acquired`);
          } else if (mode === 3) {
            console.log(`[INFO] GPS 3D fix acquired`);
          }
          lastFixMode = mode;
        }

        // If valid fix, log to gps_points
        if (mode >= 2 && msg.lat && msg.lon) {
          const horizAcc = msg.epx && msg.epy ? Math.sqrt(msg.epx ** 2 + msg.epy ** 2) : null;

          await pool.query(
            `INSERT INTO gps_points (timestamp, lat, lon, speed, track, accuracy, fix_mode)
             VALUES ($1, $2, $3, $4, $5, $6, $7)`,
            [
              msg.time || new Date().toISOString(),
              msg.lat,
              msg.lon,
              msg.speed || 0,
              msg.track || 0,
              horizAcc,
              msg.mode,
            ]
          );

          // Accuracy monitoring
          if (!DEBUG && horizAcc) {
            const lowAccuracy = horizAcc > 10;
            if (lowAccuracy && !lastAccuracyWarn) {
              console.warn(`[WARN] Low accuracy: ±${horizAcc.toFixed(1)} m`);
              lastAccuracyWarn = true;
            } else if (!lowAccuracy && lastAccuracyWarn) {
              console.log(`[INFO] Accuracy restored: ±${horizAcc.toFixed(1)} m`);
              lastAccuracyWarn = false;
            }
          }

          // Emit live data to web client
          io.emit("gps", {
  	    lat: Number(msg.lat.toFixed(8)),  // 8 decimal places (~1 cm precision)
  	    lon: Number(msg.lon.toFixed(8)),
  	    speed: msg.speed || 0,
  	    track: msg.track || 0,
  	    time: msg.time || new Date().toISOString(),
	    fix_mode: msg.mode || 0,
	    accuracy: horizAcc
	  });
	}
      }

      // --- Handle SKY messages ---
      if (msg.class === "SKY" && Array.isArray(msg.satellites)) {
        const used = msg.satellites.filter(s => s.used).length;
        const total = msg.satellites.length;
        if (DEBUG || used !== lastSatInfo.used || total !== lastSatInfo.total) {
          console.log(`[INFO] Satellites: ${used}/${total} in use`);
          lastSatInfo = { used, total };
        }
      }

    } catch (err) {
      if (DEBUG) console.error("[DEBUG] Parse error:", err.message);
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


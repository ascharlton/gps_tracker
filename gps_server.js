// server.js - Consolidated GPS, Sonar, DB, and Single Raw WebSocket Server

// TODO 
// check and create gps tables before running program
// post console log of gps tbale setup

const express = require("express");
const fs = require("fs");
const dotenv = require('dotenv') // .env file
const path = require("path");
const http = require("http");
const { Server } = require("socket.io"); // For GPS/Map Updates
const WebSocket = require("ws");       // For High-Speed Binary Sonar Stream
const { spawn } = require("child_process");
const { Pool } = require("pg");
const { SerialPort } = require('serialport'); 

const app = express();
const server = http.createServer(app);

// Dual WebSocket Setup
const io = new Server(server); // Socket.IO for GPS/Map
const wss = new WebSocket.Server({ noServer: true }); // Raw WS for Sonar Plot

// --- FIX: Storage for active raw WebSocket clients ---
const rawWsClients = new Set(); 

dotenv.config() // using .env

wss.on('connection', (ws) => {
    rawWsClients.add(ws);
    console.log('[RAW WS] New client connected for sonar plot.');
    ws.on('close', () => {
        rawWsClients.delete(ws);
        console.log('[RAW WS] Client disconnected.');
    });
    // Set binary type expectation
    ws.binaryType = 'arraybuffer';
});

// Handle raw WebSocket upgrade requests
server.on('upgrade', (request, socket, head) => {
    // FIX: Only handle raw WS upgrade if the path is explicitly for the sonar stream
    if (request.url === '/sonar_raw_stream') { 
        wss.handleUpgrade(request, socket, head, (ws) => {
            wss.emit('connection', ws, request);
        });
    } else {
        // If not the raw stream, let socket.io handle it or destroy the socket
        // Note: Socket.IO handles its own upgrade, so this block is mainly for debugging unhandled requests.
        // The default behavior will allow socket.io to connect on its path.
    }
});


const PORT = 5000;
const DEBUG = process.argv.includes("DEBUG=1") || process.env.DEBUG === "1";

// --- GLOBAL GPS STATE (for Sonar use) ---
let currentGpsData = { 
  lat: null, 
  lon: null, 
  initialized: false 
};

function getGpsCoordinates() {
    // Return a copy of the current GPS coordinates
    if (!currentGpsData.initialized) {
        return { lat: null, lon: null };
    }
    return { 
      lat: currentGpsData.lat, 
      lon: currentGpsData.lon 
    };
}

// --- 1. POSTGRESQL CONNECTION (GPS & SONAR) ---

const CREATE_SONAR_TABLE_SQL = `
CREATE TABLE IF NOT EXISTS sonar_readings (
    id SERIAL PRIMARY KEY,
    timestamp TIMESTAMPTZ NOT NULL,
    latitude DOUBLE PRECISION,
    longitude DOUBLE PRECISION,
    max_value INTEGER NOT NULL,
    max_sample_index INTEGER NOT NULL,
    max_distance_cm DOUBLE PRECISION 
);`;

const CREATE_GPS_RAW_TABLE_SQL = `
CREATE TABLE IF NOT EXISTS gps_raw (
    id SERIAL PRIMARY KEY,
    timestamp TIMESTAMPTZ NOT NULL,
    message jsonb
);`;

const DBPASS = process.env.DBPASS;

// GPS Tracker Database
const GPS_PG_CONFIG = {
  user: "pi",
  host: "localhost",
  database: "gps_tracker",
  password: DBPASS,
  port: 5432,
};

// Sonar Readings Database
const SONAR_PG_CONFIG = {
    user: 'pi',
    host: 'localhost',
    database: 'sonar',
    password:  DBPASS,
    port: 5432, 
};

const gpsPool = new Pool(GPS_PG_CONFIG);
gpsPool.connect().then(() => console.log("[GPS DB] Connected to GPS PostgreSQL"))
  .catch(err => console.error("[ERROR] GPS DB connection failed:", err.message));

const sonarPool = new Pool(SONAR_PG_CONFIG);
sonarPool.connect().then(() => console.log("[SONAR DB] Connected to Sonar PostgreSQL"))
  .catch(err => console.error("[ERROR] Sonar DB connection failed:", err.message));

// --- 2. SONAR LOGGING GLOBALS & CONSTANTS ---

// This global variable holds the latest, smoothed depth measurement (in CM)
let lastSmoothedDistance = null; 
let comBuffer = Buffer.alloc(0);
let lastDbWriteTimestamp = 0; 
/**
 * Array to temporarily hold processed sonar packets.
 * This collects all data between throttling intervals and socket emits.
 */
const collectedSonarData = []; 
// Serial Port Config
const COM_PORT_PATH = '/dev/ttyACM0'; // Sonar
const BAUD_RATE = 250000;
// Sonar Physical Constants 
const SAMPLE_RESOLUTION = 0.2178; // cm/sample
const ORIGINAL_SAMPLE_COUNT = 1800;
const NUM_SAMPLES = ORIGINAL_SAMPLE_COUNT;
const NOISE_FLOOR_RANGE = 100; 
const EMA_ALPHA = 0.1;
// Serial Packet Structure
const HEADER_BYTE = 0xAA;
const NUM_METADATA_BYTES = 6;
const SAMPLES_BYTE_SIZE = NUM_SAMPLES * 2;
const PACKET_SIZE = 1 + NUM_METADATA_BYTES + SAMPLES_BYTE_SIZE + 1; 
// Database Throttling - time delay to write sonar data to database
const DB_WRITE_INTERVAL_MS = 3000; // 3 seconds interval for logging to DB



// --- 3. HELPER FUNCTIONS FOR SONAR FRAME PROCESSING ---

// get the noisefloor for calculating the edge of the xdcr deadzone
function calculateNoiseFloor(samples) {
    const start = samples.length - NOISE_FLOOR_RANGE;
    if (start < 0) return 0;
    const tailSamples = samples.slice(start);
    const sum = tailSamples.reduce((acc, val) => acc + val, 0);
    return sum / NOISE_FLOOR_RANGE;
}

// find edge of dead zone, threshold is just above the noise floor which indicates where the noise floor begins   
function findBlindZoneEnd(samples, noiseFloor) {
    const searchLimit = 500; 
    const threshold = noiseFloor * 1.2; 
    for (let i = 0; i < Math.min(samples.length, searchLimit); i++) { 
        if (samples[i] <= threshold) { // first value that is below threshold == noise floor starts
            return i;
        }
    }
    return 150; 
}

// We want to keep track of the main reflection not any secondary reflection or noise
function findPrimaryReflection(samples) {
    if (samples.length === 0) {
        return { value: 0, index: -1, distance: 0, blindZoneEndIndex: 0 };
    }
    const noiseFloor = calculateNoiseFloor(samples); 
    const startIndex = findBlindZoneEnd(samples, noiseFloor);
    let maxValue = 0;
    let maxIndex = -1;
    for (let i = startIndex; i < samples.length; i++) {
        if (samples[i] > maxValue) {
            maxValue = samples[i];
            maxIndex = i;
        }
    }
    const rawDistanceCm = maxIndex * SAMPLE_RESOLUTION;
    return { 
        value: maxValue, 
        index: maxIndex, 
        distance: rawDistanceCm,
        blindZoneEndIndex: startIndex
    };
}

function applyExponentialSmoothing(currentDistance) {
    if (lastSmoothedDistance === null) {
        lastSmoothedDistance = currentDistance;
        return currentDistance;
    }
    const smoothedDistance = 
        (EMA_ALPHA * currentDistance) + 
        ((1 - EMA_ALPHA) * lastSmoothedDistance);
    lastSmoothedDistance = smoothedDistance;
    return smoothedDistance;
}

// --- 4. SONAR DATA ACQUISITION & PARSING ---
function serialBufferHandler(data) {
    comBuffer = Buffer.concat([comBuffer, data]);

    while (comBuffer.length >= PACKET_SIZE) {
        const headerIndex = comBuffer.indexOf(HEADER_BYTE);
        
        if (headerIndex === -1) {
            comBuffer = Buffer.alloc(0);
            console.warn(`[SONAR WARN] Dropping entire buffer. No header found.`);
            break;
        }
        if (headerIndex > 0) {
            console.warn(`[SONAR WARN] Discarding ${headerIndex} junk bytes before header.`);
            comBuffer = comBuffer.slice(headerIndex);
            if (comBuffer.length < PACKET_SIZE) break; 
        }

        const packet = comBuffer.slice(0, PACKET_SIZE);
        const payload = packet.slice(1, PACKET_SIZE - 1); 
        const samplesBuffer = payload.slice(NUM_METADATA_BYTES);
        const rawSamples = [];
        for (let i = 0; i < NUM_SAMPLES; i++) {
            rawSamples.push(samplesBuffer.readUInt16BE(i * 2)); 
        }
        //console.log(`[SONAR] processing sonar packet`);
        processDataPacket(rawSamples); 
        comBuffer = comBuffer.slice(PACKET_SIZE);                    
    }
}

/**
 * Main data processing and logging function, runs on every sonar frame.
 */
async function processDataPacket(rawSamples) {
    const timestamp = new Date();
    const reflection = findPrimaryReflection(rawSamples);
    const smoothedDistance = applyExponentialSmoothing(reflection.distance);
    
    // Skip processing and collection if peak is too low
    if (reflection.value < 50) return; 

    // --- FIX: Emit single point (Smoothed Distance & Peak) as BINARY over RAW WS ---
    const distMM = Math.round(smoothedDistance * 10); // Distance in cm to mm (Uint16)
    const peakValue = reflection.value > 255 ? 255 : reflection.value; // Clamp peak (Uint8)

    // Create a 3-byte Buffer (2 bytes for distance in mm, 1 byte for peak value)
    const buffer = Buffer.alloc(3);
    buffer.writeUInt16BE(distMM, 0); // Big Endian
    buffer.writeUInt8(peakValue, 2);

    for (const client of rawWsClients) {
        if (client.readyState === WebSocket.OPEN) {
            client.send(buffer);
        }
    }

    // 1. ALWAYS collect the processed result, storing its timestamp and the GPS data 
    // available at the moment of processing.
    collectedSonarData.push({
        timestamp: timestamp,
        reflection: reflection,
        smoothedDistance: smoothedDistance,
        gps: getGpsCoordinates() 
    });

    // --- Throttle Check (3 seconds) ---
    const currentTime = timestamp.getTime();
    if (currentTime - lastDbWriteTimestamp < DB_WRITE_INTERVAL_MS) {
        // Data is collected, but we skip DB write/emit until the interval passes
        if (DEBUG) console.log(`[DEBUG] Real-time depth (cm): ${smoothedDistance.toFixed(1)} (Data Collected, Skipping DB/Socket)`);
        return; 
    }
    
    // 2. If 3 seconds have passed AND we have collected data, proceed with DB write
    if (collectedSonarData.length === 0) {
        if (DEBUG) console.log("[DEBUG] Interval passed, but no sonar data collected to write.");
        return;
    }
    
    // Get the latest processed data from the collection for DB write
    const latestData = collectedSonarData[collectedSonarData.length - 1];
    
    const { lat, lon } = latestData.gps;

    if (!lat || !lon) {
        console.log("[SONAR DB] Skipping DB write: Latest collected data lacks GPS info. Will retry next interval.");
        return; 
    }
    
    // If 3 seconds have passed AND GPS is available, proceed with DB write
    lastDbWriteTimestamp = currentTime;

    const values = [
        latestData.timestamp,
        lat,
        lon,
        latestData.reflection.value,
        latestData.reflection.index,
        latestData.smoothedDistance
    ];

    const INSERT_SQL = `
    INSERT INTO sonar_readings (timestamp, latitude, longitude, max_value, max_sample_index, max_distance_cm)
    VALUES ($1, $2, $3, $4, $5, $6);`;

    try {
        await sonarPool.query(INSERT_SQL, values);
        // Note: We no longer clear the array here, as the GPS pipe will handle that.
        console.log(`[SONAR DB] THROTTLED DB WRITE | Latest Smoothed Depth: ${latestData.smoothedDistance.toFixed(2)} cm`);
        
    } catch (error) {
        console.error('[SONAR DB ERROR] Sonar database batch insertion failed:', error.message);
    }
}

// SONAR USB PORT LISTENER
function startSonarPort() {
    if (typeof SerialPort === 'undefined') {
        console.warn("[SONAR WARN] SerialPort not initialized.");
        return;
    }
    
    const port = new SerialPort({ 
         path: COM_PORT_PATH, 
         baudRate: BAUD_RATE 
    });
    
    port.on('error', (err) => { console.error('[COM ERROR]', err.message); });
    port.on('data', serialBufferHandler); 
    
    console.log(`[SONAR INFO] Serial port initialized on ${COM_PORT_PATH} at ${BAUD_RATE} Bps.`);
}

// --- 5. HTTP SERVER SETUP & ENDPOINTS ---
app.use("/static", express.static(path.join(__dirname, "static")));
app.use("/tiles_osm", express.static(path.join(__dirname, "tiles_osm")));
app.use("/tiles_satellite", express.static(path.join(__dirname, "tiles_satellite")));
app.use("/tiles_dark", express.static(path.join(__dirname, "tiles_dark")));
app.get("/", (req, res) => {
  res.sendFile(path.join(__dirname, "index.html"));
});
// FIX: Add route to serve sonar-plot.html
app.get("/sonar-plot.html", (req, res) => {
  res.sendFile(path.join(__dirname, "sonar-plot.html"));
});

app.get("/favicon.ico", (req, res) => {
  res.sendFile(path.join(__dirname, "static", "favicon.ico"));
});

// --- API ENDPOINT: Fetch All Sonar Data ---
app.get("/depth/all", async (req, res) => {
  try {
    const { rows } = await sonarPool.query(`
      SELECT latitude, longitude, max_distance_cm, timestamp
      FROM sonar_readings
      ORDER BY timestamp DESC
    `);
    res.json(rows.map(row => ({
      lat: row.latitude,
      lon: row.longitude,
      depth_cm: row.max_distance_cm,
      time: row.timestamp.toISOString()
    })));
  } catch (err) {
    console.error("[HTTP ERROR] /depth/all query failed:", err.message);
    res.status(500).json({ error: "DB query failed" });
  }
});

// --- EXISTING REST endpoints using Postgres (using 'gpsPool' for gps_tracker) ---

app.get("/raw/:date", async (req, res) => {
  try {
    const date = req.params.date;
    const { rows } = await gpsPool.query(`
      SELECT message
      FROM gps_raw
      WHERE DATE(timestamp) = $1
      ORDER BY timestamp ASC
    `, [date]);

    const points = rows
      .map(r => (typeof r.message === "string" ? JSON.parse(r.message) : r.message))
      .filter(m => m.class === "TPV" && m.lat && m.lon)
      .map(m => ({
        lat: m.lat,
        lon: m.lon,
        speed: m.speed || 0,
        track: m.track || 0,
        time: m.time || new Date().toISOString(),
        alt: m.alt || null,
      }));

    res.json(points);
  } catch (err) {
    console.error("[HTTP ERROR] /raw query failed:", err.message);
    res.status(500).json({ error: "DB query failed" });
  }
});

app.get("/track/:date", async (req, res) => {
  try {
    const date = req.params.date;
    const { rows } = await gpsPool.query(`
      SELECT timestamp, lat, lon, speed, track
      FROM gps_points
      WHERE DATE(timestamp) = $1
      ORDER BY timestamp ASC
    `, [date]);
    res.json(rows);
  } catch (err) {
    console.error("[HTTP ERROR] /track query failed:", err.message);
    res.status(500).json({ error: "DB query failed" });
  }
});

app.get("/waypoints", async (req, res) => {
  try {
    const { rows } = await gpsPool.query("SELECT id, remarks, lat, lon FROM gps_waypoints ORDER BY id ASC");
    res.json(rows);
  } catch (err) {
    console.error("[HTTP ERROR] /waypoints failed:", err.message);
    res.status(500).json({ error: "DB query failed" });
  }
});

app.post("/waypoints", express.json(), async (req, res) => {
  try {
    const { remarks, lat, lon, description } = req.body;
    if (!lat || !lon) return res.status(400).json({ error: "lat/lon required" });
    await gpsPool.query(
      "INSERT INTO gps_waypoints (remarks, lat, lon) VALUES ($1, $2, $3)",
      [remarks || "Waypoint", lat, lon || null]
    );
    res.json({ status: "ok" });
  } catch (err) {
    console.error("[HTTP ERROR] /waypoints POST failed:", err.message);
    res.status(500).json({ error: "DB query failed" });
  }
});

app.get("/count/raw", async (req, res) => {
  try {
    const { rows } = await gpsPool.query("SELECT COUNT(*) FROM gps_raw");
    res.json({ count: Number(rows[0].count) });
  } catch (err) {
    console.error("[HTTP ERROR] /count/raw failed:", err.message);
    res.status(500).json({ error: "DB query failed" });
  }
});

// --- GPSD live stream with auto-reconnect ---
let gpsd = null;
let lastFixMode = 0;
let lastAccuracyWarn = false;
let lastSatInfo = { used: 0, total: 0 };

function startGpsPipe() {
  console.log("[GPS INFO] Starting gpspipe...");
  gpsd = spawn("gpspipe", ["-w"]);

  gpsd.stdout.on("data", async (data) => {
    const lines = data.toString().split("\n").filter(Boolean);
    for (let line of lines) {
      try {
        const msg = JSON.parse(line);
        if (DEBUG) console.log("[DEBUG]", msg.class, msg.mode || "");

        // --- Store every message in gps_raw ---
        await gpsPool.query("INSERT INTO gps_raw (message) VALUES ($1)", [msg]);
        io.emit("raw_count_update");
        console.info(`[GPS DB] updating gps_raw`);

        // --- Handle TPV messages ---
        if (msg.class === "TPV") {
          const mode = msg.mode || 0;

          if (!DEBUG && mode !== lastFixMode) {
            if (mode < 2) console.warn(`[WARN] GPS lost — no fix (mode=${mode})`);
            else console.log(`[GPS INFO] ${mode === 2 ? "2D" : "3D"} fix acquired`);
            lastFixMode = mode;
          }

          if (mode >= 2 && msg.lat && msg.lon) {
            // Update global state for sonar logging
            currentGpsData.lat = msg.lat;
            currentGpsData.lon = msg.lon;
            currentGpsData.initialized = true;

            const horizAcc = msg.epx && msg.epy ? Math.sqrt(msg.epx ** 2 + msg.epy ** 2) : null;

            // --- Sonar Synchronization ---
            const currentDepthCm = lastSmoothedDistance !== null ? lastSmoothedDistance : 0;
            const currentDepthMeters = currentDepthCm / 100.0; 

            await gpsPool.query(
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
            console.log(`[GPS DB] updating gps_points`);

            // Accuracy monitoring CHECK if NEEDED
            if (!DEBUG && horizAcc) {
              const lowAccuracy = horizAcc > 10;
              if (lowAccuracy && !lastAccuracyWarn) {
                  console.warn(`[GPS WARN] Low accuracy: ±${horizAcc.toFixed(1)} m`);
                  lastAccuracyWarn = true;
              } else if (!lowAccuracy && lastAccuracyWarn) {
                  console.log(`[GPS INFO] Accuracy restored: ±${horizAcc.toFixed(1)} m`);
                  lastAccuracyWarn = false;
              }
            }
            
            // --- NEW: Emit Collected Sonar Data (Batch) ---
            if (collectedSonarData.length > 0) {
              const batchToEmit = collectedSonarData.map(data => ({
                  time: data.timestamp.toISOString(),
                  depth_cm: data.smoothedDistance,
                  lat: data.gps.lat,
                  lon: data.gps.lon, 
              }));
              // send to web client
              io.emit("sonar_batch", batchToEmit);
              console.log(`[EMIT] sonar_batch: ${batchToEmit.length} collected sonar packets via 'sonar_batch'.`);
              // Clear the array after successfully emitting the batch
              collectedSonarData.length = 0; 
            }

            // Emit unified GPS and Depth data (single point for map marker/header display)
            io.emit("gps", {
              lat: Number(msg.lat.toFixed(8)),
              lon: Number(msg.lon.toFixed(8)),
              alt: msg.alt || null, // Include altitude from TPV
              speed: msg.speed || 0,
              track: msg.track || 0,
              time: msg.time || new Date().toISOString(),
              fix_mode: msg.mode || 0,
              accuracy: horizAcc,
              status: msg.status ?? null, // add GPSD status field if present
              depth_m: currentDepthMeters // Synchronized Sonar Depth (meters)
            });
            console.log(`[EMIT] gps: ${msg.lat}, ${msg.lon}, ${msg.lon}, ${msg.alt}, ${msg.speed}, ${msg.track}, ${msg.time}, ${msg.mode}, ${msg.horizAcc}, ${msg.status}, ${currentDepthMeters}`);
          }
        }

        // --- Handle SKY messages ---
        if (msg.class === "SKY" && Array.isArray(msg.satellites)) {
            const used = msg.satellites.filter(s => s.used).length;
            const total = msg.satellites.length;
        if (DEBUG || used !== lastSatInfo.used || total !== lastSatInfo.total) {
            //console.log(`[INFO] Satellites: ${used}/${total} in use`);
            lastSatInfo = { used, total };
          }
          io.emit("satellite_update", lastSatInfo);
          console.log(`[EMIT] satellite_update: used: ${lastSatInfo.used}`);
        }

      } catch (err) {
        if (DEBUG) console.error("[DEBUG] Parse error:", err.message);
      }
    }
  });

  gpsd.stderr.on("data", (data) => {
    console.error("[gpspipe ERROR]", data.toString());
  });

  gpsd.on("close", (code) => {
    console.warn(`[GPS WARN] gpspipe exited (code ${code}). Retrying in 5 seconds...`);
    setTimeout(startGpsPipe, 5000); // Retry automatically
  });
}

// --- Main Execution ---
async function main() {
    startGpsPipe();
    try {
        // Ensure sonar table exists before starting the listener 
        await sonarPool.query(CREATE_SONAR_TABLE_SQL);
        console.log('[SONAR DB] Sonar table setup complete.');
        // Ensure gps_raw table exists before starting the listener 
        await gpsPool.query(CREATE_GPS_RAW_TABLE_SQL);
        console.log('[GPS DB] gps_raw table setup complete.');
    } catch (err) {
        console.error("CRITICAL ERROR: Failed to setup Sonar database table.", err.message);
        // Do not exit, allow the GPS tracking part to continue
    }
    startSonarPort(); 

    // Start server
    server.listen(PORT, "0.0.0.0", () => {
      console.log(`\n🚀 Consolidated Server running on http://0.0.0.0:${PORT}`);
    });
}

main().catch(err => {
    console.error('An unrecoverable error occurred in main:', err);
});

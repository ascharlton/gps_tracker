// sonar-perf-monitor.js
// Node.js script for local serial data processing, signal filtering, and fixed terminal plot.

const { SerialPort } = require('serialport'); 
const { performance } = require('perf_hooks');
const readline = require('readline'); 

// --- CONFIGURATION PARAMETERS (Algorithm) ---
let VALUE_THRESHOLD = 50;          
let PERSISTENCE_THRESHOLD = 5;     
let EMA_ALPHA = 0.1;               
const DISTANCE_TOLERANCE = 5;      

// --- SERIAL CONFIGURATION ---
const COM_PORT_PATH = '/dev/ttyACM0'; 
const BAUD_RATE = 250000;
// Sonar Physical Constants 
const SAMPLE_RESOLUTION = 0.2178; // cm/sample
const MAX_PLOT_DISTANCE_CM = 500; // Max depth for plot normalization
const ORIGINAL_SAMPLE_COUNT = 1800;
const NUM_SAMPLES = ORIGINAL_SAMPLE_COUNT;
const NOISE_FLOOR_RANGE = 100; 
// Serial Packet Structure
const HEADER_BYTE = 0xAA;
const NUM_METADATA_BYTES = 6;
const SAMPLES_BYTE_SIZE = NUM_SAMPLES * 2;
const PACKET_SIZE = 1 + NUM_METADATA_BYTES + SAMPLES_BYTE_SIZE + 1; 

// --- TERMINAL PLOTTING CONSTANTS ---
// The fixed area at the bottom of the screen
const PLOT_HEIGHT = 3; 

// --- STATE MANAGEMENT ---
let lastSmoothedDistance = null; 
let comBuffer = Buffer.alloc(0);
let pointIndex = 0; 
let persistentSignals = new Map(); 

// --- RATE TRACKING ---
let lastRateTime = performance.now();
let lastRateIndex = 0;
let framesPerSecond = 0.0;
let totalLatencyUs = 0;
let latencyCount = 0;

// --- CORE ALGORITHM FUNCTIONS (Unchanged) ---

function calculateNoiseFloor(samples) {
    const start = samples.length - NOISE_FLOOR_RANGE;
    if (start < 0) return 0;
    const tailSamples = samples.slice(start);
    const sum = tailSamples.reduce((acc, val) => acc + val, 0);
    return sum / NOISE_FLOOR_RANGE;
}

function findBlindZoneEnd(samples, noiseFloor) {
    const searchLimit = 500; 
    const threshold = noiseFloor * 1.2; 
    for (let i = 0; i < Math.min(samples.length, searchLimit); i++) { 
        if (samples[i] <= threshold) { 
            return i;
        }
    }
    return 150; 
}

function trackAndFilterSignals(samples) {
    const noiseFloor = calculateNoiseFloor(samples); 
    const startIndex = findBlindZoneEnd(samples, noiseFloor);
    let strongestIndex = -1;
    let strongestPeak = 0;
    
    const potentialPeaks = [];
    for (let i = startIndex; i < samples.length; i++) {
        const value = samples[i];
        if (value > VALUE_THRESHOLD) { 
            potentialPeaks.push({
                index: i,
                distance: i * SAMPLE_RESOLUTION,
                peak: value
            });
            if (value > strongestPeak) { 
                strongestPeak = value;
                strongestIndex = i;
            }
        }
    }
    
    const newPersistentSignals = new Map();
    const matchedPeaks = new Set();
    
    for (const [id, signal] of persistentSignals.entries()) {
        let matchFound = false;
        
        for (let i = 0; i < potentialPeaks.length; i++) {
            const peak = potentialPeaks[i];
            
            if (Math.abs(peak.index - signal.index) <= DISTANCE_TOLERANCE) {
                
                const updatedSignal = {
                    id: id,
                    index: peak.index, 
                    distance: peak.distance, 
                    peak: peak.peak, 
                    persistence: Math.min(signal.persistence + 1, PERSISTENCE_THRESHOLD + 5), 
                    lastSeenFrame: pointIndex
                };
                newPersistentSignals.set(id, updatedSignal);
                matchedPeaks.add(peak);
                matchFound = true;
                break; 
            }
        }
        
        if (!matchFound) {
            const decayRate = signal.persistence > PERSISTENCE_THRESHOLD ? 2 : 1; 
            const newPersistence = signal.persistence - decayRate;
            
            if (newPersistence > 0) {
                 const decayedSignal = { ...signal, persistence: newPersistence, lastSeenFrame: pointIndex };
                 newPersistentSignals.set(id, decayedSignal);
            }
        }
    }
    
    for (const peak of potentialPeaks) {
        if (!matchedPeaks.has(peak)) {
            const newId = `sig_${pointIndex}_${peak.index}`; 
            newPersistentSignals.set(newId, {
                id: newId,
                index: peak.index,
                distance: peak.distance,
                peak: peak.peak,
                persistence: 1, 
                lastSeenFrame: pointIndex
            });
        }
    }
    
    persistentSignals = newPersistentSignals;
    
    const wantedSignals = Array.from(persistentSignals.values())
        .filter(s => s.persistence >= PERSISTENCE_THRESHOLD)
        .map(s => ({ d: s.distance.toFixed(1), p: s.peak, id: s.id }));

    return { 
        wantedSignals: wantedSignals, 
        strongestDistance: strongestIndex === -1 ? 0 : strongestIndex * SAMPLE_RESOLUTION,
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

function calculateDataRate() {
    const currentTime = performance.now();
    const timeDeltaMs = currentTime - lastRateTime;
    const framesDelta = pointIndex - lastRateIndex;

    if (timeDeltaMs >= 1000) { 
        framesPerSecond = (framesDelta * 1000) / timeDeltaMs;
        lastRateTime = currentTime;
        lastRateIndex = pointIndex;
        return true;
    }
    return false;
}

// --- CONSOLE PLOT AND METRICS ---

// ANSI escape codes for colors
const ANSI_GREEN = '\x1b[32m';
const ANSI_RED = '\x1b[31m';
const ANSI_YELLOW = '\x1b[33m';
const ANSI_BLUE = '\x1b[34m';
const ANSI_RESET = '\x1b[0m';

/**
 * Logs a concise summary (the scrolling section) and updates the fixed plot area.
 */
function updateConsoleDashboard(latencyUs, signals, smoothedDistance) {
    totalLatencyUs += latencyUs;
    latencyCount++;
    const avgLatencyUs = totalLatencyUs / latencyCount;
    const shouldLogRate = calculateDataRate();

    // 1. Log History (Scrolling Section)
    
    if (shouldLogRate) {
        // Detailed metrics (FPS, Avg Latency, Thresholds) logged infrequently
        console.log(`${ANSI_BLUE}[RATE]${ANSI_RESET} FPS: ${framesPerSecond.toFixed(1)} | Avg Latency: ${avgLatencyUs.toFixed(2)} µs | V=${VALUE_THRESHOLD}, P=${PERSISTENCE_THRESHOLD}, A=${EMA_ALPHA}`);
    }
    
    // Per-frame summary (The scrolling log)
    console.log(`FRAME ${pointIndex}: ${ANSI_YELLOW}Primary: ${smoothedDistance.toFixed(1)} cm${ANSI_RESET} | Signals: ${signals.length} | Latency: ${latencyUs.toFixed(2)} µs`);

    // 2. Fixed Plot Area Update
    
    // Move cursor up PLOT_HEIGHT lines (back to the start of the fixed area)
    readline.moveCursor(process.stdout, 0, -PLOT_HEIGHT);
    
    // Clear the fixed area below the cursor for a clean redraw
    readline.clearScreenDown(process.stdout);
    
    // --- PLOT CONTENT ---
    // Line 1: Header
    process.stdout.write('--- REAL-TIME SONAR PLOT (0cm to 500cm) ---\n');

    // Line 2: Bar Plot
    const normalizedDistance = Math.min(smoothedDistance, MAX_PLOT_DISTANCE_CM);
    const scaleFactor = 60; // Max width of the bar (characters)
    const barLength = Math.round((normalizedDistance / MAX_PLOT_DISTANCE_CM) * scaleFactor);
    
    const depthBar = ANSI_GREEN + '[' + '='.repeat(barLength) + ANSI_RESET;
    const padding = ' '.repeat(scaleFactor - barLength);
    const endBracket = ']';

    let signalIndicator = '';
    if (signals.length > 0) {
        signalIndicator = `${ANSI_RED} ${signals.length}X ${ANSI_RESET}`;
    } else {
        signalIndicator = `   `;
    }

    const plotLine = 
        `${depthBar}${padding}${endBracket} ${signalIndicator}` + 
        `${ANSI_YELLOW} ${normalizedDistance.toFixed(1)} cm ${ANSI_RESET}`;
    
    process.stdout.write(plotLine + '\n'); 
    
    // Line 3: Separator/Spacer (This line sets the cursor position for the next console.log)
    process.stdout.write('--------------------------------------------------------------------------------\n');
    
    // NOTE: The cursor is now positioned correctly for the next console.log to appear in the scrolling section.
}

// --- SERIAL HANDLER ---

function serialBufferHandler(data) {
    comBuffer = Buffer.concat([comBuffer, data]);

    while (comBuffer.length >= PACKET_SIZE) {
        const headerIndex = comBuffer.indexOf(HEADER_BYTE);
        
        if (headerIndex === -1) {
            comBuffer = Buffer.alloc(0);
            break;
        }
        if (headerIndex > 0) {
            comBuffer = comBuffer.slice(headerIndex);
            if (comBuffer.length < PACKET_SIZE) break; 
        }

        const packet = comBuffer.slice(0, PACKET_SIZE);
        const samplesBuffer = packet.slice(1 + NUM_METADATA_BYTES, PACKET_SIZE - 1);
        const rawSamples = [];
        for (let i = 0; i < NUM_SAMPLES; i++) {
            rawSamples.push(samplesBuffer.readUInt16BE(i * 2)); 
        }
        processDataPacket(rawSamples); 
        comBuffer = comBuffer.slice(PACKET_SIZE);                    
    }
}

function processDataPacket(rawSamples) {
    // START Timer for Latency Measurement
    const startTime = performance.now();

    // 1. Core filtering
    const result = trackAndFilterSignals(rawSamples);
    
    // 2. Smoothing
    const smoothedDistance = applyExponentialSmoothing(result.strongestDistance);

    // 3. Increment counter
    pointIndex++; 

    // END Timer
    const endTime = performance.now();
    const latencyUs = (endTime - startTime) * 1000; // Convert ms to µs

    // 4. Console Output
    updateConsoleDashboard(latencyUs, result.wantedSignals, smoothedDistance);
}

// --- MAIN EXECUTION ---

function startComPortListener() {
    try {
        const port = new SerialPort({ 
             path: COM_PORT_PATH, 
             baudRate: BAUD_RATE
        });
        
        port.on('open', () => {
             console.log(`\n[INIT] Serial port opened successfully on ${COM_PORT_PATH} at ${BAUD_RATE} Bps.`);
             console.log(`[INIT] Starting performance monitor. Max Depth Plot: ${MAX_PLOT_DISTANCE_CM} cm. Press Ctrl+C to stop.\n`);
             
             // Print PLOT_HEIGHT blank lines to push the cursor down and reserve the fixed area
             for (let i = 0; i < PLOT_HEIGHT; i++) {
                 console.log('');
             }
        });

        port.on('error', (err) => { 
            console.error('\n[FATAL COM ERROR]', err.message);
            console.log(`[HINT] Check that your device is connected to ${COM_PORT_PATH} and powered on.`);
            process.exit(1); 
        });

        port.on('data', serialBufferHandler); 
        
    } catch (err) {
        console.error('\n[SETUP ERROR] Failed to initialize serial port:', err.message);
        console.log(`[HINT] Ensure the 'serialport' package is installed.`);
    }
}

startComPortListener();

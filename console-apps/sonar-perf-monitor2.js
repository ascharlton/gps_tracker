// sonar-perf-monitor.js
// Node.js script for local serial data processing, signal filtering, and performance monitoring.

const { SerialPort } = require('serialport'); 
const { performance } = require('perf_hooks');
const readline = require('readline'); // Used for single-line plot updates

// --- CONFIGURATION PARAMETERS (Algorithm) ---
let VALUE_THRESHOLD = 50;          // Minimum value for a sample to be considered a 'potential' signal
let PERSISTENCE_THRESHOLD = 5;     // Minimum number of consecutive frames a signal must be seen to be plotted
let EMA_ALPHA = 0.1;               // Smoothing factor for the Primary Signal line
const DISTANCE_TOLERANCE = 5;      // Max distance (in index units) a signal can shift between frames to be considered the same object

// --- SERIAL CONFIGURATION ---
const COM_PORT_PATH = '/dev/ttyACM0'; 
const BAUD_RATE = 250000;
// Sonar Physical Constants 
const SAMPLE_RESOLUTION = 0.2178; // cm/sample
const MAX_PLOT_DISTANCE_CM = 500; // Max depth for plot normalization (used for the bar width)
const ORIGINAL_SAMPLE_COUNT = 1800;
const NUM_SAMPLES = ORIGINAL_SAMPLE_COUNT;
const NOISE_FLOOR_RANGE = 100; 
// Serial Packet Structure
const HEADER_BYTE = 0xAA;
const NUM_METADATA_BYTES = 6;
const SAMPLES_BYTE_SIZE = NUM_SAMPLES * 2;
const PACKET_SIZE = 1 + NUM_METADATA_BYTES + SAMPLES_BYTE_SIZE + 1; 

// --- STATE MANAGEMENT ---
let lastSmoothedDistance = null; 
let comBuffer = Buffer.alloc(0);
let pointIndex = 0; // Total frames received
// Key: Unique signal ID, Value: { index, distance, peak, persistence, lastSeenFrame }
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
    
    // 1. Find all potential peaks (above value threshold)
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
    
    // 2. Track existing signals (Persistence Check & Decay)
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
        
        // No match found: Decay persistence
        if (!matchFound) {
            const decayRate = signal.persistence > PERSISTENCE_THRESHOLD ? 2 : 1; 
            const newPersistence = signal.persistence - decayRate;
            
            if (newPersistence > 0) {
                 const decayedSignal = { ...signal, persistence: newPersistence, lastSeenFrame: pointIndex };
                 newPersistentSignals.set(id, decayedSignal);
            }
        }
    }
    
    // 3. Track new signals (New Object Detection)
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
    
    // 4. Update and Filter State
    persistentSignals = newPersistentSignals;
    
    // Extract only the signals that have met the minimum persistence
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

    if (timeDeltaMs >= 1000) { // Update rate at least every second
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
const ANSI_RESET = '\x1b[0m';

/**
 * Creates a single-line ASCII visualization of the data.
 */
function consolePlotAndUpdate(signals, smoothedDistance) {
    // 1. Plot Logic
    const normalizedDistance = Math.min(smoothedDistance, MAX_PLOT_DISTANCE_CM);
    const scaleFactor = 60; // Max width of the bar (characters)
    const barLength = Math.round((normalizedDistance / MAX_PLOT_DISTANCE_CM) * scaleFactor);
    
    const depthBar = ANSI_GREEN + 'Depth: [' + '='.repeat(barLength) + ANSI_RESET;
    const padding = ' '.repeat(scaleFactor - barLength);
    const endBracket = ']';

    // 2. Persistent Signal Indicator
    let signalIndicator = '';
    if (signals.length > 0) {
        signalIndicator = `${ANSI_RED} X${signals.length} ${ANSI_RESET}`;
    } else {
        signalIndicator = `   `;
    }

    // 3. Final Line Output
    const plotLine = 
        `${depthBar}${padding}${endBracket} ${signalIndicator}` + 
        `${ANSI_YELLOW} ${normalizedDistance.toFixed(1)} cm ${ANSI_RESET}` +
        ` (F: ${pointIndex}, FPS: ${framesPerSecond.toFixed(1)})`;
        
    // Use readline to clear the current line and write the new one
    readline.cursorTo(process.stdout, 0);
    process.stdout.write(plotLine);
}

/**
 * Logs detailed performance metrics less frequently.
 */
function logDetailedMetrics(latencyUs) {
    totalLatencyUs += latencyUs;
    latencyCount++;
    const avgLatencyUs = totalLatencyUs / latencyCount;

    // Move to the next line before printing the log
    process.stdout.write('\n'); 
    console.log(`
[METRICS] 
  - Processing Latency: ${latencyUs.toFixed(2)} µs (Avg: ${avgLatencyUs.toFixed(2)} µs)
  - Value Threshold: ${VALUE_THRESHOLD} | Persistence: ${PERSISTENCE_THRESHOLD} | EMA Alpha: ${EMA_ALPHA}
`);
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

    // 1. Core filtering: Identify the wanted signals
    const result = trackAndFilterSignals(rawSamples);
    
    // 2. Smoothing
    const smoothedDistance = applyExponentialSmoothing(result.strongestDistance);

    // 3. Increment counter and check FPS
    pointIndex++; 
    const shouldLogRate = calculateDataRate(); 

    // END Timer
    const endTime = performance.now();
    const latencyUs = (endTime - startTime) * 1000; // Convert ms to µs

    // 4. Console Output
    consolePlotAndUpdate(result.wantedSignals, smoothedDistance);
    
    if (shouldLogRate) { 
        logDetailedMetrics(latencyUs);
    }
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
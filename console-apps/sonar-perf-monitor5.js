// sonar-perf-monitor.js
// Node.js script for fixed terminal statistics and a scrolling X-Y plot.

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
const MAX_PLOT_DISTANCE_CM = 500; // Max depth for plot normalization (Y-axis ceiling)
const ORIGINAL_SAMPLE_COUNT = 1800;
const NUM_SAMPLES = ORIGINAL_SAMPLE_COUNT;
const NOISE_FLOOR_RANGE = 100; 
// Serial Packet Structure
const HEADER_BYTE = 0xAA;
const NUM_METADATA_BYTES = 6;
const SAMPLES_BYTE_SIZE = NUM_SAMPLES * 2;
const PACKET_SIZE = 1 + NUM_METADATA_BYTES + SAMPLES_BYTE_SIZE + 1; 

// --- TERMINAL PLOTTING CONSTANTS ---
const TERMINAL_WIDTH = 100; 
const PLOT_HEIGHT_LINES = 20; // Y-axis resolution (Distance)
const PLOT_HISTORY_FRAMES = 80; // X-axis history length (Frames)
const HEADER_LINES = 4;

// --- STATE MANAGEMENT ---
let lastSmoothedDistance = null; 
let comBuffer = Buffer.alloc(0);
let pointIndex = 0; 
let persistentSignals = new Map(); 

// History buffer stores { primaryDistance: number, persistentDistances: number[], frameIndex: number }
let plotHistory = []; 

// --- RATE TRACKING ---
let lastRateTime = performance.now();
let lastRateIndex = 0;
let framesPerSecond = 0.0;
let totalLatencyUs = 0;
let latencyCount = 0;

// ANSI escape codes for colors
const ANSI_GREEN = '\x1b[32m';
const ANSI_RED = '\x1b[31m';
const ANSI_YELLOW = '\x1b[33m';
const ANSI_BLUE = '\x1b[34m';
const ANSI_RESET = '\x1b[0m';
const ANSI_BOLD = '\x1b[1m';
const ANSI_CLEAR_LINE = '\x1b[K'; 

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
    
    persistentSignals = newPersistentSignals;
    
    // 4. Extract all signals that meet the persistence threshold
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

    // Update rate at least every second
    if (timeDeltaMs >= 1000) { 
        framesPerSecond = (framesDelta * 1000) / timeDeltaMs;
        lastRateTime = currentTime;
        lastRateIndex = pointIndex;
    }
}

// --- TERMINAL PLOTTING FUNCTIONS ---

function drawFixedHeader(avgLatencyUs, latestDistance, latestSignals) {
    readline.cursorTo(process.stdout, 0, 0); 
    
    const line1 = `${ANSI_BOLD}${ANSI_BLUE}--- SONAR PERFORMANCE MONITOR ---${ANSI_RESET} (Frame ${pointIndex}, Ctrl+C to stop)`;
    const line2 = `Rate: ${ANSI_YELLOW}${framesPerSecond.toFixed(1)} FPS${ANSI_RESET} | Latency: ${ANSI_YELLOW}${avgLatencyUs.toFixed(2)} µs${ANSI_RESET} | Primary: ${ANSI_GREEN}${latestDistance.toFixed(1)} cm${ANSI_RESET} | Signals: ${ANSI_RED}${latestSignals}${ANSI_RESET}`;
    const line3 = `Config: ${ANSI_BOLD}V=${VALUE_THRESHOLD}${ANSI_RESET}, ${ANSI_BOLD}P=${PERSISTENCE_THRESHOLD}${ANSI_RESET}, ${ANSI_BOLD}A=${EMA_ALPHA}${ANSI_RESET}`;
    const line4 = `--------------------------------------------------------------------------------${ANSI_CLEAR_LINE}`;
    
    process.stdout.write(line1.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(line2.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(line3.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(line4.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
}

/**
 * Draws the scrolling X-Y plot below the header, showing multiple persistent signals.
 */
function drawScrollingPlot() {
    let output = '';
    
    for (let y = 0; y < PLOT_HEIGHT_LINES; y++) {
        let line = '';
        
        // Y-axis label/ruler (Distance)
        const distanceLabel = MAX_PLOT_DISTANCE_CM * (1 - y / PLOT_HEIGHT_LINES);
        line += `${distanceLabel.toFixed(0).padStart(4)} |`;

        // X-axis (Frames History)
        for (let x = 0; x < PLOT_HISTORY_FRAMES; x++) {
            const frame = plotHistory[x];
            let char = ' ';
            
            if (frame) {
                // Determine character precedence: Persistent (X) > Primary (@) > Space ( )
                
                // 1. Check Persistent Signals (Red 'X')
                let persistentChar = ' ';
                for (const dist of frame.persistentDistances) {
                    // Y-coordinate (row index) that this persistent signal should land on
                    const normalizedYPersistent = Math.floor((MAX_PLOT_DISTANCE_CM - dist) / MAX_PLOT_DISTANCE_CM * PLOT_HEIGHT_LINES);
                    
                    if (normalizedYPersistent === y) {
                        persistentChar = ANSI_RED + 'X' + ANSI_RESET;
                        break; // Found a persistent signal at this row, red 'X' takes precedence
                    }
                }

                // 2. Check Primary Signal (Green '@')
                const normalizedYPrimary = Math.floor((MAX_PLOT_DISTANCE_CM - frame.primaryDistance) / MAX_PLOT_DISTANCE_CM * PLOT_HEIGHT_LINES);
                
                if (normalizedYPrimary === y) {
                    // Only plot primary if no persistent signal is already at this location
                    char = persistentChar === ' ' ? (ANSI_GREEN + '@' + ANSI_RESET) : persistentChar; 
                } else {
                    // If primary is not here, only use the persistent character if found
                    char = persistentChar;
                }
            }
            line += char;
        }
        
        output += line + '|' + ANSI_CLEAR_LINE + '\n';
    }

    // X-axis ruler (Frame number) - Logic remains the same
    let xRuler = ' '.repeat(5) + '+' + '-'.repeat(PLOT_HISTORY_FRAMES) + '+';
    let xLabels = ' '.repeat(5) + '|';
    
    for (let i = 0; i < PLOT_HISTORY_FRAMES; i++) {
        if (i % 10 === 0 && plotHistory[i]) {
            xLabels += `${(plotHistory[i].frameIndex % 10)}`;
        } else {
            xLabels += ' ';
        }
    }
    xLabels += `|  (Frame ${pointIndex} at right) ${ANSI_CLEAR_LINE}`;
    
    process.stdout.write(output);
    process.stdout.write(xRuler + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(xLabels + ANSI_CLEAR_LINE + '\n');
}

/**
 * Main function to update the terminal dashboard.
 */
function updateConsoleDashboard(latencyUs, signals, smoothedDistance) {
    // 1. Update metrics
    totalLatencyUs += latencyUs;
    latencyCount++;
    const avgLatencyUs = totalLatencyUs / latencyCount;
    calculateDataRate(); // Update FPS

    // 2. Redraw fixed header (y=0)
    drawFixedHeader(avgLatencyUs, smoothedDistance, signals);

    // 3. Move cursor below header and draw plot (y=HEADER_LINES)
    readline.cursorTo(process.stdout, 0, HEADER_LINES);
    drawScrollingPlot();
    
    // 4. Ensure cursor is at the bottom of the screen
    readline.cursorTo(process.stdout, 0, HEADER_LINES + PLOT_HEIGHT_LINES + 3);
}

// --- SERIAL HANDLER (Unchanged) ---

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

    // 4. Update plot history (CHANGED TO STORE MULTIPLE DISTANCES)
    // Map the string distances back to numbers for the plot
    const persistentDistances = result.wantedSignals.map(s => parseFloat(s.d));

    if (plotHistory.length >= PLOT_HISTORY_FRAMES) {
        plotHistory.shift(); 
    }
    plotHistory.push({
        primaryDistance: smoothedDistance,
        persistentDistances: persistentDistances, // Store the list of persistent distances
        frameIndex: pointIndex
    });

    // 5. Console Output
    updateConsoleDashboard(latencyUs, persistentDistances.length, smoothedDistance);
}

// --- MAIN EXECUTION ---

function startComPortListener() {
    try {
        const port = new SerialPort({ 
             path: COM_PORT_PATH, 
             baudRate: BAUD_RATE
        });
        
        port.on('open', () => {
             // Clear the screen once at startup
             process.stdout.write('\x1b[2J\x1b[0;0H'); 
             console.log(`[INIT] Serial port opened successfully on ${COM_PORT_PATH} at ${BAUD_RATE} Bps.`);
             console.log(`[INIT] Plotting Depth vs. Frame. Max Depth Plot: ${MAX_PLOT_DISTANCE_CM} cm.`);
        });

        port.on('error', (err) => { 
            process.stdout.write('\x1b[2J\x1b[0;0H'); // Clear on fatal error
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

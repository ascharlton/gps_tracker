// sonar-perf-monitor.js
// Node.js script for fixed terminal statistics and a scrolling X-Y plot with dynamic tolerance.

const { SerialPort } = require('serialport'); 
const { performance } = require('perf_hooks');
const readline = require('readline'); 

// --- CONFIGURATION PARAMETERS (Algorithm) ---
let VALUE_THRESHOLD = 50;          // Min value for a sample to be considered a 'potential' signal
let PERSISTENCE_THRESHOLD = 5;     // Min frames required for a signal to be 'persistent'
let EMA_ALPHA = 0.1;               // Smoothing factor for the Primary Signal line
const BASE_DISTANCE_TOLERANCE = 5; // Base index tolerance for signal matching (before dynamic adjustment)

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

// History buffer stores { primaryDistance: number, minSecondaryDistance: number, frameIndex: number, totalPersistentCount: number }
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

// --- DYNAMIC TOLERANCE FUNCTION ---

/**
 * Calculates a dynamic distance tolerance based on the signal's peak strength.
 * Stronger signals get a wider tolerance for matching across frames.
 */
function getDynamicTolerance(peakValue) {
    const PEAK_MAX = 255;
    const MAX_TOLERANCE_BOOST = 15; // Max additional index tolerance
    
    // Calculate a boost factor based on peak strength above the base threshold
    const normalizedPeak = Math.max(0, peakValue - VALUE_THRESHOLD);
    
    // Scale the boost from 0 up to MAX_TOLERANCE_BOOST
    // Divisor limits the maximum peak used for scaling to prevent over-tolerance
    const boost = Math.floor((normalizedPeak / (PEAK_MAX - VALUE_THRESHOLD)) * MAX_TOLERANCE_BOOST);

    return BASE_DISTANCE_TOLERANCE + boost;
}

// --- CORE ALGORITHM FUNCTIONS ---

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
            
            // --- IMPLEMENT DYNAMIC TOLERANCE ---
            // Use the *new* peak's strength to determine the required match tolerance
            const requiredTolerance = getDynamicTolerance(peak.peak);
            
            if (Math.abs(peak.index - signal.index) <= requiredTolerance) {
                
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
    
    // 4. Summarize and Filter State for Plotting
    const primaryDistance = strongestIndex === -1 ? 0 : strongestIndex * SAMPLE_RESOLUTION;
    let minSecondaryDistance = 0; 
    let totalPersistentCount = 0;
    
    // Use the maximum dynamic tolerance possible for this comparison, converted to cm
    const PRIMARY_DISTANCE_TOLERANCE_CM = getDynamicTolerance(255) * SAMPLE_RESOLUTION;

    for (const signal of persistentSignals.values()) {
        if (signal.persistence >= PERSISTENCE_THRESHOLD) {
            totalPersistentCount++;
            
            // Check if this signal is close enough to the primary signal to be considered the same object
            const isNearPrimary = Math.abs(signal.distance - primaryDistance) <= PRIMARY_DISTANCE_TOLERANCE_CM;
            
            if (!isNearPrimary) {
                // Found a valid, non-primary persistent signal
                if (minSecondaryDistance === 0 || signal.distance < minSecondaryDistance) {
                    minSecondaryDistance = signal.distance;
                }
            }
        }
    }

    return { 
        totalPersistentCount: totalPersistentCount, 
        primaryDistance: primaryDistance,
        minSecondaryDistance: minSecondaryDistance // Closest non-primary persistent signal for the plot
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
    }
}

// --- TERMINAL PLOTTING FUNCTIONS ---

function drawFixedHeader(avgLatencyUs, latestDistance, totalPersistentCount) {
    // Move cursor to top-left
    readline.cursorTo(process.stdout, 0, 0); 
    
    // Ensure FPS is updated frequently for the header
    calculateDataRate();
    
    const line1 = `${ANSI_BOLD}${ANSI_BLUE}--- SONAR PERFORMANCE MONITOR ---${ANSI_RESET} (Frame ${pointIndex}, Ctrl+C to stop)`;
    // Note: totalPersistentCount includes the primary signal if it meets the persistence threshold
    const line2 = `Rate: ${ANSI_YELLOW}${framesPerSecond.toFixed(1)} FPS${ANSI_RESET} | Latency: ${ANSI_YELLOW}${avgLatencyUs.toFixed(2)} µs${ANSI_RESET} | Primary: ${ANSI_GREEN}${latestDistance.toFixed(1)} cm${ANSI_RESET} | Persistent Signals (Total): ${ANSI_RED}${totalPersistentCount}${ANSI_RESET}`;
    const line3 = `Config: ${ANSI_BOLD}V=${VALUE_THRESHOLD}${ANSI_RESET}, ${ANSI_BOLD}P=${PERSISTENCE_THRESHOLD}${ANSI_RESET}, ${ANSI_BOLD}A=${EMA_ALPHA}${ANSI_RESET} | Dyn Tolerance: ${BASE_DISTANCE_TOLERANCE} to ${getDynamicTolerance(255)}`;
    const line4 = `--------------------------------------------------------------------------------${ANSI_CLEAR_LINE}`;
    
    process.stdout.write(line1.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(line2.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(line3.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(line4.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
}

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
                // 1. Check Secondary Signal (Red 'X') - Closest non-primary persistent reflection
                if (frame.minSecondaryDistance > 0) {
                    // Y-coordinate (row index) that this secondary signal should land on
                    const normalizedYSecondary = Math.floor((MAX_PLOT_DISTANCE_CM - frame.minSecondaryDistance) / MAX_PLOT_DISTANCE_CM * PLOT_HEIGHT_LINES);
                    
                    if (normalizedYSecondary === y) {
                        char = ANSI_RED + 'X' + ANSI_RESET;
                    }
                }

                // 2. Check Primary Signal (Green '@') - Strongest reflection
                const normalizedYPrimary = Math.floor((MAX_PLOT_DISTANCE_CM - frame.primaryDistance) / MAX_PLOT_DISTANCE_CM * PLOT_HEIGHT_LINES);
                
                if (normalizedYPrimary === y) {
                    // Only plot primary if no secondary signal is already at this location
                    char = (char === ' ') ? (ANSI_GREEN + '@' + ANSI_RESET) : char; 
                }
            }
            line += char;
        }
        
        output += line + '|' + ANSI_CLEAR_LINE + '\n';
    }

    // X-axis ruler (Frame number)
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
function updateConsoleDashboard(latencyUs, totalPersistentCount, smoothedDistance, minSecondaryDistance) {
    // 1. Update metrics
    totalLatencyUs += latencyUs;
    latencyCount++;
    const avgLatencyUs = totalLatencyUs / latencyCount;

    // 2. Update history buffer
    if (plotHistory.length >= PLOT_HISTORY_FRAMES) {
        plotHistory.shift(); 
    }
    plotHistory.push({
        primaryDistance: smoothedDistance,
        minSecondaryDistance: minSecondaryDistance,
        frameIndex: pointIndex,
        totalPersistentCount: totalPersistentCount
    });

    // 3. Redraw fixed header (y=0)
    drawFixedHeader(avgLatencyUs, smoothedDistance, totalPersistentCount);

    // 4. Move cursor below header and draw plot (y=HEADER_LINES)
    readline.cursorTo(process.stdout, 0, HEADER_LINES);
    drawScrollingPlot();
    
    // 5. Ensure cursor is at the bottom of the screen
    readline.cursorTo(process.stdout, 0, HEADER_LINES + PLOT_HEIGHT_LINES + 3);
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
    const smoothedDistance = applyExponentialSmoothing(result.primaryDistance);

    // 3. Increment counter
    pointIndex++; 

    // END Timer
    const endTime = performance.now();
    const latencyUs = (endTime - startTime) * 1000; // Convert ms to µs

    // 4. Console Output
    updateConsoleDashboard(
        latencyUs, 
        result.totalPersistentCount, 
        smoothedDistance, 
        result.minSecondaryDistance
    );
}

// --- MAIN EXECUTION ---

function startComPortListener() {
    // ... (Serial port initialization logic remains the same)
    try {
        const port = new SerialPort({ 
             path: COM_PORT_PATH, 
             baudRate: BAUD_RATE
        });
        
        port.on('open', () => {
             process.stdout.write('\x1b[2J\x1b[0;0H'); 
             console.log(`[INIT] Serial port opened successfully on ${COM_PORT_PATH} at ${BAUD_RATE} Bps.`);
             console.log(`[INIT] Plotting Depth vs. Frame. Max Depth Plot: ${MAX_PLOT_DISTANCE_CM} cm.`);
        });

        port.on('error', (err) => { 
            process.stdout.write('\x1b[2J\x1b[0;0H'); 
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
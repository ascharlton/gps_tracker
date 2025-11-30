// sonar-perf-monitor.js
// Node.js script for fixed terminal statistics, median tracking, and expanded X-Y plot.
// This version uses aggressive pre-filtering (consolidation) to ensure one physical reflection 
// is tracked as one stable signal ID, and plots signals using their index (1, 2, 3...) for clarity.
// NEW: The newest frame is indicated by a '>' symbol.

const { SerialPort } = require('serialport'); 
const { performance } = require('perf_hooks');
const readline = require('readline'); 

// --- CONFIGURATION PARAMETERS (Algorithm) ---
let VALUE_THRESHOLD = 20;          // Min value for a sample to be considered a 'potential' signal
let PERSISTENCE_THRESHOLD = 10;    // Min frames required for a signal to be 'persistent' (increased for stability)
let EMA_ALPHA = 0.1;               // Smoothing factor for the Stable Signal line (Header Only)
const BASE_DISTANCE_TOLERANCE = 5; // Base index tolerance (minimum tolerance)
const CONSOLIDATION_TOLERANCE = 5; // New: Max indices between peaks to be considered one reflection (~1.1 cm)

// --- SERIAL CONFIGURATION ---
const COM_PORT_PATH = '/dev/ttyACM0'; 
const BAUD_RATE = 250000;
// Sonar Physical Constants 
const SAMPLE_RESOLUTION = 0.2178; // cm/sample
const MAX_PLOT_DISTANCE_CM = 240; // FIXED: Plot ceiling (0 to 200 cm)
const ORIGINAL_SAMPLE_COUNT = 1800;
const NUM_SAMPLES = ORIGINAL_SAMPLE_COUNT;
const NOISE_FLOOR_RANGE = 100; 
// Serial Packet Structure
const HEADER_BYTE = 0xAA;
const NUM_METADATA_BYTES = 6;
const SAMPLES_BYTE_SIZE = NUM_SAMPLES * 2;
const PACKET_SIZE = 1 + NUM_METADATA_BYTES + SAMPLES_BYTE_SIZE + 1; 

// --- TERMINAL PLOTTING CONSTANTS ---
const TERMINAL_WIDTH = 200; 
const PLOT_HEIGHT_LINES = 60;     // Doubled plot height for more resolution
const PLOT_HISTORY_FRAMES = 160; 
const HEADER_LINES = 6;
const SIGNAL_SUMMARY_LINES = 6;   // Lines reserved for signal details

// --- STATE MANAGEMENT ---
let lastSmoothedDistance = null; 
let comBuffer = Buffer.alloc(0);
let pointIndex = 0; 
let persistentSignals = new Map(); 

// History buffer stores { stableHeaderDistance: number, allPersistentMedianDistances: { medianDistance: number, plotIndex: number }[], frameIndex: number, totalPersistentCount: number }
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

// --- DYNAMIC TOLERANCE & MEDIAN FUNCTIONS ---

function getDynamicTolerance(peakValue) {
    const PEAK_MAX = 255;
    const MAX_TOLERANCE_BOOST = 15; 
    
    // Normalize the peak strength above the base threshold (50)
    const normalizedPeak = Math.max(0, peakValue - VALUE_THRESHOLD);
    
    // Scale the boost from 0 up to MAX_TOLERANCE_BOOST
    const boost = Math.floor((normalizedPeak / (PEAK_MAX - VALUE_THRESHOLD)) * MAX_TOLERANCE_BOOST);

    return BASE_DISTANCE_TOLERANCE + boost;
}

/**
 * Calculates the median of an array of numbers.
 */
function calculateMedian(distances) {
    if (distances.length === 0) return 0;
    const sorted = [...distances].sort((a, b) => a - b);
    const mid = Math.floor(sorted.length / 2);
    if (sorted.length % 2 === 0) {
        return (sorted[mid - 1] + sorted[mid]) / 2;
    }
    return sorted[mid];
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
        }
    }
    
    // --- Aggressive Peak Consolidation (to ensure 1 physical reflection = 1 peak) ---
    const consolidatedPeaks = [];
    // Sort peaks by index (distance) for easy consolidation
    const sortedPotentialPeaks = potentialPeaks.sort((a, b) => a.index - b.index);
    
    let currentGroup = null;
    
    for (const peak of sortedPotentialPeaks) {
        if (!currentGroup) {
            // Start the first group
            currentGroup = {
                indices: [peak.index],
                peaks: [peak]
            };
        } else if (peak.index - currentGroup.indices[currentGroup.indices.length - 1] <= CONSOLIDATION_TOLERANCE) {
            // Peak is within CONSOLIDATION_TOLERANCE of the last index in the group, consolidate it
            currentGroup.indices.push(peak.index);
            currentGroup.peaks.push(peak);
        } else {
            // New group starts. Finalize the current group by selecting the dominant (max peak) reflection.
            const dominantPeak = currentGroup.peaks.reduce((max, p) => (p.peak > max.peak ? p : max), currentGroup.peaks[0]);
            consolidatedPeaks.push(dominantPeak);
            
            // Start the new group
            currentGroup = {
                indices: [peak.index],
                peaks: [peak]
            };
        }
    }
    
    // Finalize the last group
    if (currentGroup && currentGroup.peaks.length > 0) {
        const dominantPeak = currentGroup.peaks.reduce((max, p) => (p.peak > max.peak ? p : max), currentGroup.peaks[0]);
        consolidatedPeaks.push(dominantPeak);
    }
    
    // Use the consolidated list for all subsequent tracking logic
    const finalPeaks = consolidatedPeaks;
    
    // ------------------------------------------------------------------------------------
    
    const newPersistentSignals = new Map();
    const matchedPeaks = new Set();
    
    // 1. Determine which incoming final peak maps to which existing persistent signal
    const peakToSignalMap = new Map(); // Key: finalPeak index (in finalPeaks array), Value: persistentSignal ID
    
    for (let i = 0; i < finalPeaks.length; i++) {
        const peak = finalPeaks[i];
        let closestSignalId = null;
        let minDiff = Infinity;

        for (const [id, signal] of persistentSignals.entries()) {
            // Tolerance is based on the existing signal's peak strength
            const requiredTolerance = getDynamicTolerance(signal.peak);
            const diff = Math.abs(peak.index - signal.index);
            
            // Match is only valid if within tolerance AND it's the closest match
            if (diff <= requiredTolerance && diff < minDiff) {
                minDiff = diff;
                closestSignalId = id;
            }
        }
        
        if (closestSignalId) {
            // Assign the incoming peak to the closest persistent signal
            peakToSignalMap.set(i, closestSignalId);
        }
    }
    
    // 2. Update persistent signals based on the assignments (one peak only updates one signal)
    const matchedSignalIds = new Set();
    
    for (let i = 0; i < finalPeaks.length; i++) {
        const signalId = peakToSignalMap.get(i);
        if (!signalId) continue;
        
        const peak = finalPeaks[i];
        
        // Ensure this persistent signal hasn't already been updated in this frame 
        if (matchedSignalIds.has(signalId)) {
             continue; 
        }

        const signal = persistentSignals.get(signalId);
        const requiredTolerance = getDynamicTolerance(signal.peak);

        // --- MEDIAN CALCULATION on Match ---
        const currentDistanceHistory = signal.distanceHistory ? signal.distanceHistory : [];
        const newDistanceHistory = [...currentDistanceHistory, peak.distance].slice(-Math.max(5, PERSISTENCE_THRESHOLD * 2));
        const medianDist = calculateMedian(newDistanceHistory);

        const updatedSignal = {
            id: signalId,
            index: peak.index, // IMPORTANT: Updated to the dominant peak's index
            distance: peak.distance, 
            peak: peak.peak, 
            persistence: Math.min(signal.persistence + 1, PERSISTENCE_THRESHOLD + 5), 
            lastSeenFrame: pointIndex,
            calculatedTolerance: requiredTolerance, 
            distanceHistory: newDistanceHistory,
            medianDistance: medianDist,
        };
        newPersistentSignals.set(signalId, updatedSignal);
        matchedSignalIds.add(signalId);
        matchedPeaks.add(peak); // Mark peak as used for new signal tracking
    }
    
    // 3. Decay non-matched signals 
    for (const [id, signal] of persistentSignals.entries()) {
        if (!matchedSignalIds.has(id)) {
            const decayRate = signal.persistence > PERSISTENCE_THRESHOLD ? 2 : 1; 
            const newPersistence = signal.persistence - decayRate;
            
            if (newPersistence > 0) {
                 const currentDistanceHistory = signal.distanceHistory ? signal.distanceHistory.slice(1) : []; 
                 const medianDist = calculateMedian(currentDistanceHistory);
                 
                 const decayedSignal = { 
                     ...signal, 
                     persistence: newPersistence, 
                     lastSeenFrame: pointIndex,
                     distanceHistory: currentDistanceHistory,
                     medianDistance: medianDist,
                     calculatedTolerance: signal.calculatedTolerance || BASE_DISTANCE_TOLERANCE 
                 };
                 newPersistentSignals.set(id, decayedSignal);
            }
        }
    }
    
    // 4. Track new signals (from unmatched peaks)
    for (const peak of finalPeaks) {
        // If the peak was not used to update an existing signal (matchedPeaks set)
        if (!matchedPeaks.has(peak)) { 
            const newId = `sig_${pointIndex}_${peak.index}`; 
            const calculatedTolerance = getDynamicTolerance(peak.peak);
            newPersistentSignals.set(newId, {
                id: newId,
                index: peak.index,
                distance: peak.distance,
                peak: peak.peak,
                persistence: 1, 
                lastSeenFrame: pointIndex,
                calculatedTolerance: calculatedTolerance, 
                distanceHistory: [peak.distance],
                medianDistance: peak.distance,
            });
        }
    }
    
    persistentSignals = newPersistentSignals;
    
    // 5. Summarize State for Plotting
    // Changed to store objects containing both distance and the plot index
    const persistentSignalsForPlotting = []; 
    let totalPersistentCount = 0;
    let stableHeaderDistance = 0; 
    
    // Sort signals by distance to ensure the header value is consistently the closest persistent signal
    const sortedSignals = Array.from(persistentSignals.values()).sort((a, b) => a.medianDistance - b.medianDistance);

    let signalIndex = 0;
    for (const signal of sortedSignals) {
        if (signal.persistence >= PERSISTENCE_THRESHOLD) {
            totalPersistentCount++;
            signalIndex++; // This is the Sig X index (1, 2, 3...)
            
            // Store distance AND the calculated plot index
            persistentSignalsForPlotting.push({
                medianDistance: signal.medianDistance,
                plotIndex: signalIndex 
            });

            // Use the closest persistent signal's median for the header's stable reading
            if (stableHeaderDistance === 0) {
                stableHeaderDistance = signal.medianDistance;
            }
        }
    }

    return { 
        totalPersistentCount: totalPersistentCount, 
        primaryValueForSmoothing: stableHeaderDistance, 
        // Renamed property to reflect it now holds objects
        allPersistentMedianDistances: persistentSignalsForPlotting, 
        detailedPersistentSignals: Array.from(persistentSignals.values())
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

function drawFixedHeader(avgLatencyUs, latestDistance, totalPersistentCount, detailedSignals) {
    readline.cursorTo(process.stdout, 0, 0); 
    
    calculateDataRate();
    
    // Calculate the overall max tolerance for display
    const maxTol = detailedSignals.length > 0 ? 
        Math.max(...detailedSignals.map(s => s.calculatedTolerance || BASE_DISTANCE_TOLERANCE)) : getDynamicTolerance(255);
    
    const line1 = `${ANSI_BOLD}${ANSI_BLUE}--- SONAR PERFORMANCE MONITOR (Y-Axis: 0-${MAX_PLOT_DISTANCE_CM}cm) ---${ANSI_RESET} (Frame ${pointIndex}, Ctrl+C to stop)`;
    const line2 = `Rate: ${ANSI_YELLOW}${framesPerSecond.toFixed(1)} FPS${ANSI_RESET} | Latency: ${ANSI_YELLOW}${avgLatencyUs.toFixed(2)} µs${ANSI_RESET} | Stable Signal (Median): ${ANSI_GREEN}${latestDistance.toFixed(1)} cm${ANSI_RESET} | Persistent Signals (Total): ${ANSI_RED}${totalPersistentCount}${ANSI_RESET}`;
    // Updated line 3 to report only MAX tolerance
    const line3 = `Config: ${ANSI_BOLD}V=${VALUE_THRESHOLD}${ANSI_RESET}, ${ANSI_BOLD}P=${PERSISTENCE_THRESHOLD}${ANSI_RESET}, ${ANSI_BOLD}A=${EMA_ALPHA}${ANSI_RESET} | Signal Tolerance: ${ANSI_RED}${maxTol} (Indices)${ANSI_RESET}`;
    const line4 = `--------------------------------------------------------------------------------${ANSI_CLEAR_LINE}`;
    
    process.stdout.write(line1.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(line2.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(line3.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(line4.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
}

/**
 * Draws the summary of all currently tracked signals with persistence >= 1.
 */
function drawPersistentSignalSummary(detailedSignals) {
    // Filter to signals detected at least once (P >= 1) and sort by Median Distance
    const detectedOnly = detailedSignals.filter(s => s.persistence >= 1).sort((a, b) => a.medianDistance - b.medianDistance);
    
    // Line 5: Summary Header
    process.stdout.write(`${ANSI_BOLD}ACTIVE SIGNAL DETAILS (P >= 1): (Persistent Threshold is P=${PERSISTENCE_THRESHOLD})${ANSI_RESET} ${ANSI_CLEAR_LINE}\n`);
    
    if (detectedOnly.length > 0) {
        // Line 6-9: Signal details (show max 4 signals)
        const signalsToShow = detectedOnly.slice(0, 4);

        signalsToShow.forEach((s, index) => {
            // Sig index is index + 1
            const persistenceColor = s.persistence >= PERSISTENCE_THRESHOLD ? ANSI_RED : ANSI_YELLOW;
            const line = 
                `${persistenceColor}[Sig ${index + 1}]${ANSI_RESET} ` + 
                `Curr: ${s.distance.toFixed(1)}cm | ` + 
                `Median: ${s.medianDistance ? s.medianDistance.toFixed(1) : '---'}cm | ` + 
                `Peak: ${s.peak.toString().padStart(3, ' ')} | ` + 
                `P: ${s.persistence.toString().padStart(2, ' ')} | ` + 
                `TOL: ${s.calculatedTolerance.toString().padStart(2, ' ')}`; 
            process.stdout.write(line + ANSI_CLEAR_LINE + '\n');
        });

        // Fill remaining lines if less than 4 signals were shown
        for (let i = signalsToShow.length; i < 4; i++) {
            process.stdout.write(ANSI_CLEAR_LINE + '\n');
        }
        
    } else {
         process.stdout.write(`No signals detected above threshold (${VALUE_THRESHOLD}).${ANSI_CLEAR_LINE}\n`);
         process.stdout.write(ANSI_CLEAR_LINE + '\n');
         process.stdout.write(ANSI_CLEAR_LINE + '\n');
         process.stdout.write(ANSI_CLEAR_LINE + '\n');
    }
    
    // Line 10: Separator line
    process.stdout.write('================================================================================\n');
}

function drawScrollingPlot() {
    let output = '';
    // The last frame is the newest data (far right of the plot)
    const lastFrameIndex = PLOT_HISTORY_FRAMES - 1; 
    
    for (let y = 0; y < PLOT_HEIGHT_LINES; y++) {
        let line = '';
        
        // Y-axis label/ruler (Distance: 0cm at bottom, 200cm at top)
        const distanceLabel = MAX_PLOT_DISTANCE_CM * (1 - y / PLOT_HEIGHT_LINES);
        line += `${distanceLabel.toFixed(0).padStart(4)} |`;

        for (let x = 0; x < PLOT_HISTORY_FRAMES; x++) {
            const frame = plotHistory[x];
            let char = ' ';
            
            if (frame) {
                // --- PLOT ALL PERSISTENT SIGNALS (Using Index or '>' for newest) ---
                for (const signalData of frame.allPersistentMedianDistances) {
                    const medianDist = signalData.medianDistance;
                    const plotIndex = signalData.plotIndex;
                    const plotChar = plotIndex.toString(); // e.g., '1', '2', '3'
                    
                    const normalizedY = Math.floor((MAX_PLOT_DISTANCE_CM - medianDist) / MAX_PLOT_DISTANCE_CM * PLOT_HEIGHT_LINES);
                    
                    // Only plot if within the fixed y-axis scale
                    if (normalizedY === y && medianDist <= MAX_PLOT_DISTANCE_CM) {
                        
                        if (x === lastFrameIndex) {
                            // NEW: Use '>' for the newest frame to indicate flow direction
                            char = ANSI_RED + '>' + ANSI_RESET;
                        } else {
                            // Use the signal index for historical data
                            char = ANSI_RED + plotChar + ANSI_RESET;
                        }
                        
                        break; 
                    }
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
    xLabels += `|  (Frame ${pointIndex} at right, > indicates newest data) ${ANSI_CLEAR_LINE}`;
    
    process.stdout.write(output);
    process.stdout.write(xRuler + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(xLabels + ANSI_CLEAR_LINE + '\n');
}

function updateConsoleDashboard(latencyUs, totalPersistentCount, smoothedDistance, allPersistentMedianDistances, detailedSignals) {
    
    totalLatencyUs += latencyUs;
    latencyCount++;
    const avgLatencyUs = totalLatencyUs / latencyCount;

    // 1. Update history buffer
    if (plotHistory.length >= PLOT_HISTORY_FRAMES) {
        plotHistory.shift(); 
    }
    plotHistory.push({
        stableHeaderDistance: smoothedDistance,
        // allPersistentMedianDistances is now an array of { medianDistance, plotIndex } objects
        allPersistentMedianDistances: allPersistentMedianDistances, 
        frameIndex: pointIndex,
        totalPersistentCount: totalPersistentCount
    });

    // 2. Redraw fixed header (y=0)
    drawFixedHeader(avgLatencyUs, smoothedDistance, totalPersistentCount, detailedSignals);

    // 3. Draw signal summary (y=HEADER_LINES)
    readline.cursorTo(process.stdout, 0, HEADER_LINES);
    drawPersistentSignalSummary(detailedSignals);

    // 4. Draw plot (y=HEADER_LINES + SIGNAL_SUMMARY_LINES)
    const plotStartLine = HEADER_LINES + SIGNAL_SUMMARY_LINES;
    readline.cursorTo(process.stdout, 0, plotStartLine);
    drawScrollingPlot();
    
    // 5. Ensure cursor is at the bottom of the screen
    readline.cursorTo(process.stdout, 0, plotStartLine + PLOT_HEIGHT_LINES + 3);
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
    const startTime = performance.now();

    const result = trackAndFilterSignals(rawSamples);
    
    // Apply smoothing to the stable header reading
    const smoothedDistance = applyExponentialSmoothing(result.primaryValueForSmoothing);

    pointIndex++; 

    const endTime = performance.now();
    const latencyUs = (endTime - startTime) * 1000; 

    updateConsoleDashboard(
        latencyUs, 
        result.totalPersistentCount, 
        smoothedDistance, 
        result.allPersistentMedianDistances, 
        result.detailedPersistentSignals 
    );
}

// --- MAIN EXECUTION ---

function startComPortListener() {
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
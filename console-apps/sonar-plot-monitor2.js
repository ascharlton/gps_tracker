// sonar-plot-monitor.js
// Node.js console application for plotting real-time sonar reflections.
//
// Features:
// 1. Packet parsing for TUSS4470-like data stream (Header + Metadata + 1800 samples).
// 2. Real-time tracking of persistent signals (reflections) using a consistency buffer.
// 3. Consolidation of adjacent consistent indices into a single median-averaged reflection point.
// 4. Scrolling X-Y console plot (Distance vs. Frame Index).

const { SerialPort } = require('serialport');
const { performance } = require('perf_hooks');
const readline = require('readline');

// --- CONFIGURATION PARAMETERS (ADJUSTABLE) ---
// Note: These should be tuned based on your specific sensor data characteristics.
const VALUE_THRESHOLD = 92;                 // 2. Min value for a sample to be considered a 'potential' peak (adjust to filter noise).
const CONSISTENCY_SAMPLES = 10;             // 1. Min frames required for an index to be considered 'consistent'.
const POSITION_TOLERANCE = 1;               // Range (+/-) within which a peak position is considered the same over time.
const CONSOLIDATION_TOLERANCE_INDICES = 1;  // Max indices between consistent points to group them into a single reflection.
let EMA_ALPHA = 0.1;                        // Smoothing factor for the main header distance (0.1 is moderate smoothing).
const DEAD_ZONE_END_INDEX = 300;            // NEW: Indices from 0 up to (but not including) this index are ignored.

// --- PLOTTING & VISUAL CONFIGURATION ---
const COM_PORT_PATH = '/dev/ttyACM0';       // CHANGE THIS to your serial port path (e.g., 'COM3' or '/dev/ttyS0')
const BAUD_RATE = 250000;
const MAX_PLOT_DISTANCE_CM = 350;           // Y-axis maximum distance (0 to 250 cm).
const MAX_PLOT_HISTORY_FRAMES = 50;         // X-axis history depth (50 columns/frames).
const PLOT_HEIGHT_LINES = 40;               // Plot vertical resolution.

// --- SONAR PHYSICAL CONSTANTS ---
const NUM_SAMPLES = 1800;
const SAMPLE_RESOLUTION = 0.19;           // cm/sample (derived from speed of sound and sample time)
const NUM_METADATA_BYTES = 6;               // Assuming 6 bytes of metadata after the header
const SAMPLES_BYTE_SIZE = NUM_SAMPLES * 2;  // 1800 samples * 2 bytes/sample
const PACKET_HEADER = 0xAA;
const PACKET_SIZE = 1 + NUM_METADATA_BYTES + SAMPLES_BYTE_SIZE + 1; // Header + Metadata + Samples + Checksum

// --- STATE MANAGEMENT ---
let comBuffer = Buffer.alloc(0);
let pointIndex = 0;
let lastSmoothedDistance = null;
let signalTracker = null; // Instance of SignalTracker
let plotHistory = [];     // Stores plot data for the scrolling graph

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
const ANSI_MAGENTA = '\x1b[35m';
const ANSI_RESET = '\x1b[0m';
const ANSI_BOLD = '\x1b[1m';
const ANSI_CLEAR_LINE = '\x1b[K';
const TERMINAL_WIDTH = 100;

// --- Signal Tracker Class ---

class SignalTracker {
    /**
     * Manages consistency checking across frames (Criterion 1)
     */
    constructor(bufferSize, threshold, tolerance) {
        this.buffer = [];
        this.bufferSize = bufferSize;
        this.threshold = threshold; // Value Threshold (Criterion 2)
        this.tolerance = tolerance; // Position Tolerance
        this.consistentIndices = new Set();
    }

    // 1. Finds all consistent indices (indices that appear in all frames within tolerance)
    updateAndGetConsistentIndices(currentValues) {
        // Find current peaks above threshold (Criterion 2)
        const currentPeakIndices = [];
        
        // START index check AFTER the DEAD_ZONE_END_INDEX
        for (let i = DEAD_ZONE_END_INDEX; i < currentValues.length; i++) {
            if (currentValues[i] >= this.threshold) {
                currentPeakIndices.push(i);
            }
        }

        // Update buffer (store the index list)
        this.buffer.push(currentPeakIndices);
        if (this.buffer.length > this.bufferSize) {
            this.buffer.shift();
        }

        if (this.buffer.length < this.bufferSize) {
            this.consistentIndices.clear();
            return this.consistentIndices;
        }

        // Check consistency across all buffered frames (Criterion 1)
        const newConsistent = new Set();

        for (const index of this.buffer[this.buffer.length - 1]) {
            let isConsistent = true;
            for (let i = 0; i < this.buffer.length - 1; i++) {
                const pastPeakIndices = this.buffer[i];

                const minIdx = Math.max(0, index - this.tolerance);
                const maxIdx = Math.min(NUM_SAMPLES - 1, index + this.tolerance);

                let isMatch = false;
                for (const pIdx of pastPeakIndices) {
                    if (pIdx >= minIdx && pIdx <= maxIdx) {
                        isMatch = true;
                        break;
                    }
                }
                if (!isMatch) {
                    isConsistent = false;
                    break;
                }
            }
            if (isConsistent) {
                newConsistent.add(index);
            }
        }

        this.consistentIndices = newConsistent;
        return this.consistentIndices;
    }
}

// --- CONSOLIDATION & MEDIAN AVERAGING ---

/**
 * Groups adjacent consistent indices into single, consolidated reflection points.
 * @param {Array<number>} allConsistentIndices - List of indices that passed consistency check.
 * @param {Array<number>} rawSamples - The raw amplitude values for the current frame.
 * @returns {Array<{indices: number[], avgIndex: number, avgPeak: number, firstIndex: number}>} Consolidated groups.
 */
function consolidateAdjacentIndices(allConsistentIndices, rawSamples) {
    if (allConsistentIndices.size === 0) return [];

    // Sort the indices to group them correctly
    const sortedIndices = Array.from(allConsistentIndices).sort((a, b) => a - b);
    const consolidatedGroups = [];
    let currentGroup = [];

    for (let i = 0; i < sortedIndices.length; i++) {
        const currentIndex = sortedIndices[i];

        if (currentGroup.length === 0) {
            currentGroup.push(currentIndex);
        } else {
            const lastIndexInGroup = currentGroup[currentGroup.length - 1];

            // Check for adjacency based on CONSOLIDATION_TOLERANCE_INDICES
            if (currentIndex - lastIndexInGroup <= CONSOLIDATION_TOLERANCE_INDICES) {
                currentGroup.push(currentIndex);
            } else {
                // Gap is too large, finalize current group and start a new one
                consolidatedGroups.push(currentGroup);
                currentGroup = [currentIndex];
            }
        }
    }
    // Push the last group
    if (currentGroup.length > 0) {
        consolidatedGroups.push(currentGroup);
    }

    // Process each group to calculate average index and peak value
    return consolidatedGroups.map(indices => {
        const sumIndex = indices.reduce((sum, index) => sum + index, 0);
        const sumPeak = indices.reduce((sum, index) => sum + rawSamples[index], 0);

        return {
            indices: indices,
            firstIndex: indices[0],
            avgIndex: sumIndex / indices.length,
            avgPeak: sumPeak / indices.length,
            // Convert index to distance
            avgDistanceCm: (sumIndex / indices.length) * SAMPLE_RESOLUTION,
        };
    });
}

// --- CORE SIGNAL TRACKING AND PLOT PREP ---

// Global map to track signals across frames (key is a persistent ID)
const persistentSignals = new Map();

/**
 * Calculates the median of an array of numbers.
 */
function calculateMedian(values) {
    if (values.length === 0) return 0;
    const sorted = [...values].sort((a, b) => a - b);
    const mid = Math.floor(sorted.length / 2);
    if (sorted.length % 2 === 0) {
        return (sorted[mid - 1] + sorted[mid]) / 2;
    }
    return sorted[mid];
}

/**
 * Updates the global persistentSignals map using the current frame's consolidated groups.
 * @param {Array<object>} consolidatedGroups - The groups from the current frame.
 * @returns {Array<object>} The final, stable, median-averaged reflection points for plotting.
 */
function updatePersistentSignals(consolidatedGroups) {
    const newPersistentSignals = new Map();
    const currentFrameIds = new Set();

    // 1. Match current consolidated groups to existing persistent signals
    for (const currentGroup of consolidatedGroups) {
        let matchedSignalId = null;
        let minDiff = Infinity;

        // Find the closest existing signal based on distance/index
        for (const [id, signal] of persistentSignals.entries()) {
            const diff = Math.abs(currentGroup.avgIndex - signal.avgIndex);
            if (diff <= POSITION_TOLERANCE * 2 && diff < minDiff) {
                minDiff = diff;
                matchedSignalId = id;
            }
        }

        const signalId = matchedSignalId || `sig_${pointIndex}_${currentGroup.firstIndex}`;

        // Get existing or set initial state
        const signal = persistentSignals.get(signalId) || {
            id: signalId,
            distanceHistory: [],
            medianDistance: 0,
        };

        // Update history and persistence
        const newDistanceHistory = [...signal.distanceHistory, currentGroup.avgDistanceCm].slice(-CONSISTENCY_SAMPLES * 2);
        const medianDist = calculateMedian(newDistanceHistory);

        const updatedSignal = {
            ...signal,
            avgIndex: currentGroup.avgIndex, // Keep track of the current average index
            avgDistanceCm: currentGroup.avgDistanceCm,
            avgPeak: currentGroup.avgPeak,
            distanceHistory: newDistanceHistory,
            medianDistance: medianDist,
            lastSeenFrame: pointIndex,
        };

        newPersistentSignals.set(signalId, updatedSignal);
        currentFrameIds.add(signalId);
    }

    // 2. Remove old signals (simple decay: keep only those seen in this frame)
    persistentSignals.clear();
    for (const [id, signal] of newPersistentSignals.entries()) {
        persistentSignals.set(id, signal);
    }

    // 3. Prepare final array for plotting (sorted by median distance)
    const finalSignals = Array.from(persistentSignals.values())
        .filter(s => s.medianDistance > 0 && s.medianDistance <= MAX_PLOT_DISTANCE_CM)
        .sort((a, b) => a.medianDistance - b.medianDistance);

    return finalSignals;
}


function applyExponentialSmoothing(currentDistance) {
    if (lastSmoothedDistance === null || currentDistance === 0) {
        lastSmoothedDistance = currentDistance;
        return currentDistance;
    }
    const smoothedDistance =
        (EMA_ALPHA * currentDistance) +
        ((1 - EMA_ALPHA) * lastSmoothedDistance);
    lastSmoothedDistance = smoothedDistance;
    return smoothedDistance;
}


// --- TERMINAL PLOTTING FUNCTIONS ---

function drawFixedHeader(latencyUs, totalPersistentCount, smoothedDistance, finalSignals) {
    // Clear screen and reset cursor position before drawing header
    process.stdout.write('\x1b[2J\x1b[0;0H');

    // Update FPS
    const currentTime = performance.now();
    const timeDeltaMs = currentTime - lastRateTime;
    const framesDelta = pointIndex - lastRateIndex;
    if (timeDeltaMs >= 1000) {
        framesPerSecond = (framesDelta * 1000) / timeDeltaMs;
        lastRateTime = currentTime;
        lastRateIndex = pointIndex;
    }

    totalLatencyUs += latencyUs;
    latencyCount++;
    const avgLatencyUs = totalLatencyUs / latencyCount;

    // Convert dead zone index to distance for display
    const deadZoneDistance = DEAD_ZONE_END_INDEX * SAMPLE_RESOLUTION;

    const line1 = `${ANSI_BOLD}${ANSI_BLUE}--- SONAR REAL-TIME REFLECTION MONITOR ---${ANSI_RESET} (Frame ${pointIndex}, Ctrl+C to stop)`;
    const line2 = `Rate: ${ANSI_YELLOW}${framesPerSecond.toFixed(1)} FPS${ANSI_RESET} | Latency: ${ANSI_YELLOW}${avgLatencyUs.toFixed(2)} µs${ANSI_RESET} | Primary Signal (Smoothed): ${ANSI_GREEN}${smoothedDistance.toFixed(1)} cm${ANSI_RESET} | Active Reflections: ${ANSI_RED}${totalPersistentCount}${ANSI_RESET}`;
    const line3 = `Tracking Config: ${ANSI_BOLD}V=${VALUE_THRESHOLD}${ANSI_RESET} (Value), ${ANSI_BOLD}C=${CONSISTENCY_SAMPLES}${ANSI_RESET} (Frames), ${ANSI_BOLD}T=${POSITION_TOLERANCE}${ANSI_RESET} (Pos Tol), ${ANSI_BOLD}G=${CONSOLIDATION_TOLERANCE_INDICES}${ANSI_RESET} (Group Tol) | Dead Zone: ${ANSI_YELLOW}0-${DEAD_ZONE_END_INDEX} indices (${deadZoneDistance.toFixed(1)} cm)${ANSI_RESET}`;
    const line4 = `Plot (X-Y: 0-${MAX_PLOT_HISTORY_FRAMES} Frames / 0-${MAX_PLOT_DISTANCE_CM} cm): ${ANSI_CLEAR_LINE}`;

    process.stdout.write(line1.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(line2.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(line3.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(line4.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');

    // Reflection Summary (Line 5+)
    process.stdout.write(`${ANSI_BOLD}${ANSI_MAGENTA}ACTIVE REFLECTIONS (Median Distance):${ANSI_RESET} (Total: ${totalPersistentCount})${ANSI_CLEAR_LINE}\n`);
    const summaryLine = finalSignals.map((s, index) => {
        return `[R${index + 1}] ${s.medianDistance.toFixed(1)}cm (Peak: ${s.avgPeak.toFixed(0)})`;
    }).join(' | ');

    process.stdout.write(summaryLine.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write('--------------------------------------------------------------------------------' + ANSI_CLEAR_LINE + '\n');
}

function drawScrollingPlot() {
    let output = '';
    const plotWidth = MAX_PLOT_HISTORY_FRAMES;
    // const plotStartLine = 7; // Header (4) + Summary (2) + Separator (1)

    // Y-axis to plot distance
    for (let y = 0; y < PLOT_HEIGHT_LINES; y++) {
        let line = '';

        // Calculate distance represented by this row (top is 250, bottom is 0)
        const distanceLabel = MAX_PLOT_DISTANCE_CM * (1 - y / PLOT_HEIGHT_LINES);
        line += `${distanceLabel.toFixed(0).padStart(4)} |`;

        // Loop through the visible portion of the history buffer (X-axis)
        for (let x = 0; x < plotWidth; x++) {
            const frame = plotHistory[x];
            let char = ' ';
            let minReflectionIndex = Infinity;

            if (frame) {
                // Check all persistent signals in this frame
                frame.finalSignals.forEach((signal, index) => {
                    const medianDist = signal.medianDistance;
                    const reflectionIndex = index + 1; // 1, 2, 3...

                    // SCALING LOGIC: map median distance to the plot row 'y'
                    const normalizedY = Math.floor((MAX_PLOT_DISTANCE_CM - medianDist) / MAX_PLOT_DISTANCE_CM * PLOT_HEIGHT_LINES);

                    // Check if this signal falls on the current row 'y'
                    if (normalizedY === y && medianDist <= MAX_PLOT_DISTANCE_CM) {
                        // Plot the lowest reflection index (R1 is primary)
                        if (reflectionIndex < minReflectionIndex) {
                            minReflectionIndex = reflectionIndex;
                        }
                    }
                });

                // Set the character based on the highest priority signal found
                if (minReflectionIndex !== Infinity) {
                    const plotChar = minReflectionIndex.toString();

                    if (x === plotWidth - 1) {
                        // Use '>' for the newest data point
                        char = ANSI_RED + '>' + ANSI_RESET;
                    } else {
                        // Plot the signal index (1, 2, 3)
                        char = ANSI_RED + plotChar + ANSI_RESET;
                    }
                }
            }
            
            // Draw a dashed line to indicate the dead zone boundary
            const deadZoneRow = Math.floor((MAX_PLOT_DISTANCE_CM - (DEAD_ZONE_END_INDEX * SAMPLE_RESOLUTION)) / MAX_PLOT_DISTANCE_CM * PLOT_HEIGHT_LINES);
            if (y === deadZoneRow) {
                // Overlay a yellow dash to mark the dead zone
                if (char === ' ') {
                    char = ANSI_YELLOW + '-' + ANSI_RESET;
                }
            }
            
            line += char;
        }

        output += line + '|' + ANSI_CLEAR_LINE + '\n';
    }

    // Print the Plot
    process.stdout.write(output);

    // X-axis ruler (Frame number)
    let xRuler = ' '.repeat(5) + '+' + '-'.repeat(plotWidth) + '+';
    let xLabels = ' '.repeat(5) + '|';

    // Draw frame index markers along the X-axis
    for (let i = 0; i < plotWidth; i++) {
        const frame = plotHistory[i];
        if (i % 10 === 0 && frame) { // Label every 10 frames
            // Use the last digit of the frame index to save space
            xLabels += `${(frame.frameIndex % 10)}`;
        } else {
            xLabels += ' ';
        }
    }
    xLabels += `|  (Frame ${pointIndex} at right, > indicates newest data) ${ANSI_CLEAR_LINE}`;

    process.stdout.write(xRuler + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(xLabels + ANSI_CLEAR_LINE + '\n');
}

function updateConsoleDashboard(latencyUs, finalSignals) {
    const totalPersistentCount = finalSignals.length;
    const primaryDistance = finalSignals.length > 0 ? finalSignals[0].medianDistance : 0;
    const smoothedDistance = applyExponentialSmoothing(primaryDistance);

    // 1. Update history buffer
    if (plotHistory.length >= MAX_PLOT_HISTORY_FRAMES) {
        plotHistory.shift();
    }
    plotHistory.push({
        smoothedDistance: smoothedDistance,
        finalSignals: finalSignals,
        frameIndex: pointIndex,
    });

    // 2. Redraw fixed header (y=0)
    drawFixedHeader(latencyUs, totalPersistentCount, smoothedDistance, finalSignals);

    // 3. Draw plot (starts at y=7)
    const plotStartLine = 7;
    readline.cursorTo(process.stdout, 0, plotStartLine);
    drawScrollingPlot();

    // 4. Ensure cursor is at the bottom of the screen
    readline.cursorTo(process.stdout, 0, plotStartLine + PLOT_HEIGHT_LINES + 3);
}

// --- SERIAL HANDLER ---

function serialBufferHandler(data) {
    comBuffer = Buffer.concat([comBuffer, data]);

    while (comBuffer.length >= PACKET_SIZE) {
        const headerIndex = comBuffer.indexOf(PACKET_HEADER);

        if (headerIndex === -1) {
            comBuffer = Buffer.alloc(0);
            break;
        }
        if (headerIndex > 0) {
            comBuffer = comBuffer.slice(headerIndex);
            if (comBuffer.length < PACKET_SIZE) break;
        }

        const packet = comBuffer.slice(0, PACKET_SIZE);
        // Note: Skipping metadata/checksum validation for brevity
        const samplesBuffer = packet.slice(1 + NUM_METADATA_BYTES, PACKET_SIZE - 1);
        const rawSamples = [];
        for (let i = 0; i < NUM_SAMPLES; i++) {
            // Read 2-byte (Uint16) Big Endian
            rawSamples.push(samplesBuffer.readUInt16BE(i * 2));
        }
        processDataPacket(rawSamples);
        comBuffer = comBuffer.slice(PACKET_SIZE);
    }
}

function processDataPacket(rawSamples) {
    const startTime = performance.now();

    // 1. Find consistent indices (Criterion 1 & 2)
    const consistentIndices = signalTracker.updateAndGetConsistentIndices(rawSamples);

    // 2. Consolidate adjacent consistent indices into groups (the single reflection point)
    const consolidatedGroups = consolidateAdjacentIndices(consistentIndices, rawSamples);

    // 3. Update persistent tracking and get median-averaged signals for plotting
    const finalSignals = updatePersistentSignals(consolidatedGroups);

    pointIndex++;

    const endTime = performance.now();
    const latencyUs = (endTime - startTime) * 1000;

    // 4. Update the console display
    updateConsoleDashboard(latencyUs, finalSignals);
}

// --- MAIN EXECUTION ---

function startComPortListener() {
    // Initialize the Signal Tracker instance globally
    signalTracker = new SignalTracker(CONSISTENCY_SAMPLES, VALUE_THRESHOLD, POSITION_TOLERANCE);

    try {
        const port = new SerialPort({
             path: COM_PORT_PATH,
             baudRate: BAUD_RATE
        });

        port.on('open', () => {
             process.stdout.write('\x1b[2J\x1b[0;0H');
             console.log(`[INIT] Serial port opened successfully on ${COM_PORT_PATH} at ${BAUD_RATE} Bps.`);
             console.log(`[INIT] Max Plot Depth: ${MAX_PLOT_DISTANCE_CM} cm. Plot History: ${MAX_PLOT_HISTORY_FRAMES} frames.`);
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
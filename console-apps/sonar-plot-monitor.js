
// sonar-plot-monitor.js

// Need to fix dead zone and reduce signals to single lines

// Node.js console application for plotting real-time sonar reflections.
//
// Features:
// 1. Dynamic Dead Zone Calculation based on noise floor.
// 2. Real-time tracking of persistent signals (reflections) using a two-tiered consistency buffer.
// 3. Consolidation of adjacent consistent indices into a single median-averaged reflection point, 
//    now based on the actual physical pulse width.
// 4. Scrolling X-Y console plot (Index vs. Frame Index).

const { SerialPort } = require('serialport');
const { performance } = require('perf_hooks');
const readline = require('readline');

// --- CONFIGURATION PARAMETERS (ADJUSTABLE) ---
// Note: These should be tuned based on your specific sensor data characteristics.
const VALUE_THRESHOLD_STRONG = 92;          // 70 (80)Min value for a sample to be considered a 'strong' reflection. (Used internally for tracking consistency tiers)
const VALUE_THRESHOLD_WEAK = 82;            // 30 (32)Min value for a sample to be considered a 'weak' potential peak (adjust to filter noise).
const CONSISTENCY_SAMPLES_STRONG = 5;       // 3 (5)Frames required for a STRONG signal to be considered persistent (fast track).
const CONSISTENCY_SAMPLES_WEAK = 20;        // 60 (30)Frames required for a WEAK signal to be considered persistent (slow track).
const POSITION_TOLERANCE = 1;               // 1 (1)Range (+/-) within which a peak position is considered the same over time.
const CONSOLIDATION_TOLERANCE_INDICES = 4;  // 5 (4)LEGACY: Max indices between consistent points to group them into a single reflection. (Replaced by noise floor logic for pulse extent)
const SIGNAL_EXTENT_NOISE_FACTOR = 1.3;     // 1.1 (1.01)NEW: Factor (e.g., 1.1 * NoiseFloor) for determining when a reflection pulse ends.
let EMA_ALPHA = 0.8;                        // 0.1 (0.4)Smoothing factor for the main header value (0.1 is moderate smoothing).
const NOISE_FLOOR_EMA_ALPHA = 0.1;         // NEW: Aggressive smoothing (slow change) for the running noise floor average.
const NOISE_SUPPRESSION_FACTOR = 0.2;       // NEW: Factor to multiply samples by if they are in the noise band (e.g., 0.1 to aggressively reduce them).

// --- DYNAMIC DEAD ZONE CONFIG ---
const NOISE_FLOOR_RANGE = 200;              // The number of tail samples to average for noise floor.
const BLIND_ZONE_SEARCH_LIMIT = 500;        // Max index to search for the end of the pulse ring-down.
const BLIND_ZONE_THRESHOLD_FACTOR = 0.9;    // Threshold for ending blind zone (1.2 * NoiseFloor).
const BLIND_ZONE_DEFAULT_END = 300;         // Fallback index if blind zone isn't found within search limit.

// --- PLOTTING & VISUAL CONFIGURATION ---
const COM_PORT_PATH = '/dev/ttyACM0';       // CHANGE THIS to your serial port path (e.g., 'COM3' or '/dev/ttyS0')
const BAUD_RATE = 250000;
const MAX_PLOT_INDEX = 1000;                // 1800 Y-axis maximum: Represents the total number of samples (Indices 0 to 1799).
const MAX_PLOT_HISTORY_FRAMES = 120;         // 50 (120)X-axis history depth (50 columns/frames).
const PLOT_HEIGHT_LINES = 40;               // 30 (40)Plot vertical resolution (reduced slightly for console clarity).

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
let lastSmoothedIndex = null;
let signalTracker = null; // Instance of SignalTracker
let plotHistory = [];     // Stores plot data for the scrolling graph
let dynamicDeadZoneIndex = BLIND_ZONE_DEFAULT_END; // Dynamic dead zone index for the current frame

// NEW NOISE FLOOR TRACKING STATE
let runningNoiseFloorEMA = 0.0;
let minNoiseFloor = Infinity;
let maxNoiseFloor = 0.0;

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
const ANSI_CYAN = '\x1b[36m';
const ANSI_WHITE = '\x1b[37m'; // Standard White
// Using 256 color code for a distinct orange, may fallback on some terminals
const ANSI_ORANGE = '\x1b[38;5;208m'; 
const ANSI_RESET = '\x1b[0m';
const ANSI_BOLD = '\x1b[1m';
const ANSI_CLEAR_LINE = '\x1b[K';
const TERMINAL_WIDTH = 100;

// Dedicated color for the dead zone line (not one of the signal colors)
const DEAD_ZONE_COLOR = ANSI_BLUE; 

// --- DYNAMIC DEAD ZONE IMPLEMENTATION ---

/**
 * Calculates the noise floor by averaging the last NOISE_FLOOR_RANGE samples.
 * @param {Array<number>} samples - The full array of raw amplitude samples.
 * @returns {number} The average noise floor value.
 */
function calculateNoiseFloor(samples) {
    const start = samples.length - NOISE_FLOOR_RANGE;
    if (start < 0) return 0;
    // Get the tail samples
    const tailSamples = samples.slice(start);
    const sum = tailSamples.reduce((acc, val) => acc + val, 0);
    return sum / NOISE_FLOOR_RANGE;
}

/**
 * Finds the index where the signal drops below the calculated noise threshold.
 * This marks the end of the high-amplitude transmitter ring-down/dead zone.
 * @param {Array<number>} samples - The full array of raw amplitude samples.
 * @param {number} noiseFloor - The baseline noise floor value.
 * @returns {number} The index where the blind zone ends.
 */
function findBlindZoneEnd(samples, noiseFloor) {
    const threshold = noiseFloor * BLIND_ZONE_THRESHOLD_FACTOR;
    
    // Search up to the configured limit
    for (let i = 0; i < Math.min(samples.length, BLIND_ZONE_SEARCH_LIMIT); i++) { 
        if (samples[i] <= threshold) { 
            return i;
        }
    }
    // Fallback if the signal never drops below the threshold in the search area
    return BLIND_ZONE_DEFAULT_END; 
}

/**
 * NEW: Determines the index at which a detected signal pulse drops back to the noise floor.
 * This is used to define the entire width of the object's reflection for better averaging.
 * @param {Array<number>} samples - The full array of raw amplitude samples.
 * @param {number} startIndex - The starting index of the reflection pulse (the first consistent index).
 * @param {number} noiseFloor - The baseline noise floor value.
 * @returns {number} The index where the signal pulse ends (inclusive).
 */
function findSignalPulseExtent(samples, startIndex, noiseFloor) {
    // The threshold for the pulse tail is slightly lower than the dead zone threshold
    const threshold = noiseFloor * SIGNAL_EXTENT_NOISE_FACTOR;
    
    // Start searching immediately after the initial consistent index
    for (let i = startIndex + 1; i < samples.length; i++) { 
        if (samples[i] < threshold) { 
            // The pulse has dropped below the noise threshold
            return i - 1; // Return the index *before* it dropped below threshold
        }
        // Safety break to prevent searching the entire 1800 samples if a massive signal is detected
        if (i - startIndex > 200) { 
            return i; 
        }
    }
    // Fallback: If the pulse runs off the end of the sample array
    return samples.length - 1; 
}

// --- Signal Tracker Class with Two-Tiered Consistency ---

class SignalTracker {
    /**
     * Manages consistency checking across frames with two tiers (strong/weak).
     */
    constructor(strongFrames, weakFrames, strongThreshold, weakThreshold, tolerance) {
        this.buffer = [];
        this.strongFrames = strongFrames;
        this.weakFrames = weakFrames;
        this.strongThreshold = strongThreshold; // Value Threshold for strong signals
        this.weakThreshold = weakThreshold;   // Value Threshold for weak signals
        this.tolerance = tolerance;           // Position Tolerance
    }

    /**
     * Finds and filters indices that meet either strong or weak consistency criteria.
     * @param {Array<number>} currentValues - Raw amplitude samples for the current frame.
     * @param {number} deadZoneIndex - Index where the dead zone ends.
     * @returns {Set<number>} Set of indices that passed consistency check.
     */
    updateAndGetConsistentIndices(currentValues, deadZoneIndex) {
        const currentPeakIndices = [];
        const currentStrongPeaks = new Set();
        
        // Find current peaks above the weak threshold, starting AFTER the dead zone
        for (let i = deadZoneIndex; i < currentValues.length; i++) {
            // Note: We are just looking for indices that exceed the threshold here.
            // Pulse extent detection will handle grouping in the next step.
            if (currentValues[i] >= this.weakThreshold) {
                currentPeakIndices.push({ index: i, value: currentValues[i] });
                if (currentValues[i] >= this.strongThreshold) {
                    currentStrongPeaks.add(i);
                }
            }
        }

        // Update buffer (store the index list for the current frame)
        this.buffer.push({ 
            frameIndex: pointIndex,
            peaks: currentPeakIndices, 
            strongPeaks: currentStrongPeaks 
        });

        // Keep buffer size for the longest required history
        while (this.buffer.length > this.weakFrames) {
            this.buffer.shift();
        }

        if (this.buffer.length === 0) return new Set();

        const consistentIndices = new Set();
        const latestFrame = this.buffer[this.buffer.length - 1];

        // Check consistency for each peak in the latest frame
        for (const { index, value } of latestFrame.peaks) {
            // Determine required buffer size based on signal strength
            const requiredBufferSize = value >= this.strongThreshold 
                ? this.strongFrames 
                : this.weakFrames;

            // Start checking from the frame required for the consistency level
            const startIndex = Math.max(0, this.buffer.length - requiredBufferSize);
            
            // If we don't have enough frames for the required consistency, skip
            if (this.buffer.length < requiredBufferSize) continue;

            let isConsistent = true;
            
            // Check consistency across the required window
            for (let i = startIndex; i < this.buffer.length - 1; i++) {
                const pastPeakIndices = this.buffer[i].peaks;

                const minIdx = Math.max(0, index - this.tolerance);
                const maxIdx = Math.min(NUM_SAMPLES - 1, index + this.tolerance);

                let isMatch = false;
                for (const { index: pIdx } of pastPeakIndices) {
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
                consistentIndices.add(index);
            }
        }

        return consistentIndices;
    }
}

// --- CONSOLIDATION & MEDIAN AVERAGING ---

/**
 * Groups adjacent consistent indices into single, consolidated reflection points, 
 * using the dynamic pulse extent detection (new logic).
 * @param {Set<number>} allConsistentIndices - Set of indices that passed consistency check.
 * @param {Array<number>} rawSamples - The raw amplitude values for the current frame.
 * @param {number} noiseFloor - The current frame's calculated noise floor.
 * @returns {Array<{indices: number[], avgIndex: number, avgPeak: number, firstIndex: number}>} Consolidated groups.
 */
function consolidateAdjacentIndices(allConsistentIndices, rawSamples, noiseFloor) {
    if (allConsistentIndices.size === 0) return [];

    // Sort the indices to process them sequentially
    const sortedConsistentIndices = Array.from(allConsistentIndices).sort((a, b) => a - b);
    const consolidatedGroups = [];
    const usedIndices = new Set();

    for (const startIndex of sortedConsistentIndices) {
        // Skip indices that have already been assigned to a previous group's pulse extent
        if (usedIndices.has(startIndex)) {
            continue;
        }

        // Determine the end of the pulse
        const endIndex = findSignalPulseExtent(rawSamples, startIndex, noiseFloor);

        // Collect all indices from startIndex to endIndex (inclusive) for the average
        const pulseIndices = [];
        for (let i = startIndex; i <= endIndex; i++) {
            // Only include samples that are above the weak threshold to avoid skewing the average with noise
            if (rawSamples[i] >= VALUE_THRESHOLD_WEAK) {
                 pulseIndices.push(i);
            }
            usedIndices.add(i); // Mark all indices in this range as used
        }

        if (pulseIndices.length > 0) {
            // Process the dynamically defined group (the pulse)
            const peakValues = pulseIndices.map(index => rawSamples[index]);
            const sumIndex = pulseIndices.reduce((sum, index) => sum + index, 0);
            const sumPeak = peakValues.reduce((sum, val) => sum + val, 0);
            
            consolidatedGroups.push({
                indices: pulseIndices,
                firstIndex: pulseIndices[0],
                avgIndex: sumIndex / pulseIndices.length,
                avgPeak: sumPeak / pulseIndices.length,
                // Store average index for plotting Y-axis
                avgIndexValue: sumIndex / pulseIndices.length,
                isStrong: (sumPeak / pulseIndices.length) >= VALUE_THRESHOLD_STRONG, 
            });
        }
    }

    return consolidatedGroups;
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
    const MAX_DECAY_FRAMES = CONSISTENCY_SAMPLES_WEAK;

    // 1. Match current consolidated groups to existing persistent signals
    for (const currentGroup of consolidatedGroups) {
        let matchedSignalId = null;
        let minDiff = Infinity;

        // Find the closest existing signal based on index
        for (const [id, signal] of persistentSignals.entries()) {
            // Check proximity based on POSITION_TOLERANCE
            const diff = Math.abs(currentGroup.avgIndexValue - signal.lastAvgIndex);
            if (diff <= POSITION_TOLERANCE * 2 && diff < minDiff) {
                minDiff = diff;
                matchedSignalId = id;
            }
        }

        const signalId = matchedSignalId || `sig_${pointIndex}_${currentGroup.firstIndex}`;

        // Get existing or set initial state
        const signal = persistentSignals.get(signalId) || {
            id: signalId,
            indexHistory: [], 
            medianIndex: 0,
            lastSeenFrame: pointIndex,
            lastAvgIndex: currentGroup.avgIndexValue,
        };

        // Update history and persistence
        const newIndexHistory = [...signal.indexHistory, currentGroup.avgIndexValue].slice(-MAX_DECAY_FRAMES * 2);
        const medianIndex = calculateMedian(newIndexHistory);

        const updatedSignal = {
            ...signal,
            lastAvgIndex: currentGroup.avgIndexValue, // The index that matched this frame
            avgIndexValue: currentGroup.avgIndexValue,
            avgPeak: currentGroup.avgPeak,
            indexHistory: newIndexHistory,
            medianIndex: medianIndex, 
            lastSeenFrame: pointIndex,
            isStrong: currentGroup.isStrong,
        };

        newPersistentSignals.set(signalId, updatedSignal);
        currentFrameIds.add(signalId);
    }

    // 2. Decay/remove signals not seen in the current frame (simple retention)
    for (const [id, signal] of persistentSignals.entries()) {
        // If a signal wasn't matched in this frame, add it back but don't update lastSeenFrame
        if (!currentFrameIds.has(id)) {
             // Keep signals alive for a short duration after they disappear, or use the last known data for plotting
             if (pointIndex - signal.lastSeenFrame < MAX_DECAY_FRAMES) {
                 newPersistentSignals.set(id, signal);
             }
        }
    }

    // 3. Prepare final array for plotting (sorted by median index)
    persistentSignals.clear();
    for (const [id, signal] of newPersistentSignals.entries()) {
        persistentSignals.set(id, signal);
    }
    
    const finalSignals = Array.from(persistentSignals.values())
        .filter(s => s.medianIndex > 0 && s.medianIndex <= MAX_PLOT_INDEX)
        .sort((a, b) => a.medianIndex - b.medianIndex);

    return finalSignals;
}


function applyExponentialSmoothing(currentIndex) {
    if (lastSmoothedIndex === null || currentIndex === 0) {
        lastSmoothedIndex = currentIndex;
        return currentIndex;
    }
    const smoothedIndex =
        (EMA_ALPHA * currentIndex) +
        ((1 - EMA_ALPHA) * lastSmoothedIndex);
    lastSmoothedIndex = smoothedIndex;
    return smoothedIndex;
}


// --- TERMINAL PLOTTING FUNCTIONS ---

/**
 * Returns the appropriate ANSI color code based on the peak value thresholds.
 * @param {number} peakValue - The average peak value of the consolidated reflection group.
 * @returns {string} The ANSI color code (White, Yellow, Orange, Red).
 */
function getSignalColor(peakValue) {
    if (peakValue >= 80) {
        return ANSI_RED;
    } else if (peakValue >= 60) {
        return ANSI_ORANGE;
    } else if (peakValue >= 40) {
        return ANSI_YELLOW;
    } else {
        return ANSI_WHITE;
    }
}

/**
 * Returns a text classification based on the peak value thresholds.
 */
function getSignalClassification(peakValue) {
    if (peakValue >= 80) {
        return 'Critical';
    } else if (peakValue >= 60) {
        return 'High';
    } else if (peakValue >= 40) {
        return 'Medium';
    } else {
        return 'Low';
    }
}


function drawFixedHeader(latencyUs, totalPersistentCount, smoothedIndex, finalSignals, noiseFloor) {
    // Clear screen and reset cursor position before drawing header
    process.stdout.write('\x1b[2J\x1b[0;0H');

    // Update FPS
    // ... (existing rate calculation logic) ...
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
    const avgLatencyUs = latencyCount > 0 ? totalLatencyUs / latencyCount : 0;

    // Convert dead zone index to distance for display (kept for reference, but main focus is index)
    const deadZoneDistance = dynamicDeadZoneIndex * SAMPLE_RESOLUTION;

    const line1 = `${ANSI_BOLD}${ANSI_BLUE}--- SONAR REAL-TIME REFLECTION MONITOR ---${ANSI_RESET} (Frame ${pointIndex}, Ctrl+C to stop)`;
    const line2 = `Rate: ${ANSI_YELLOW}${framesPerSecond.toFixed(1)} FPS${ANSI_RESET} | Latency: ${ANSI_YELLOW}${avgLatencyUs.toFixed(2)} µs${ANSI_RESET} | Primary Signal (Smoothed Index): ${ANSI_GREEN}${smoothedIndex.toFixed(0)}${ANSI_RESET} | Active Reflections: ${ANSI_RED}${totalPersistentCount}${ANSI_RESET}`;
    const line3 = `Tracking Config: ${ANSI_BOLD}S.V=${VALUE_THRESHOLD_STRONG}, W.V=${VALUE_THRESHOLD_WEAK}${ANSI_RESET} (Value), ${ANSI_BOLD}S.C=${CONSISTENCY_SAMPLES_STRONG}, W.C=${CONSISTENCY_SAMPLES_WEAK}${ANSI_RESET} (Frames), ${ANSI_BOLD}T=${POSITION_TOLERANCE}${ANSI_RESET} (Pos Tol), ${ANSI_BOLD}G=${SIGNAL_EXTENT_NOISE_FACTOR}${ANSI_RESET} (Noise Factor for Pulse Extent)`;
    
    // MODIFIED LINE 4
    const line4 = `Dynamic Dead Zone: ${DEAD_ZONE_COLOR}0-${dynamicDeadZoneIndex} indices${ANSI_RESET} (${deadZoneDistance.toFixed(1)} cm) | Frame Noise Floor: ${noiseFloor.toFixed(2)}`;
    
    // NEW LINE 5
    const line5_new = `${ANSI_BOLD}${ANSI_CYAN}Running Noise Floor EMA: ${runningNoiseFloorEMA.toFixed(2)}${ANSI_RESET} (Min: ${minNoiseFloor.toFixed(2)}, Max: ${maxNoiseFloor.toFixed(2)})`;

    const line6 = `Plot (X-Y: 0-${MAX_PLOT_HISTORY_FRAMES} Frames / 0-${MAX_PLOT_INDEX} Samples): ${ANSI_CLEAR_LINE}`;

    process.stdout.write(line1.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(line2.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(line3.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(line4.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write(line5_new.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n'); // New line
    process.stdout.write(line6.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n'); // Line 6 (was 5)

    // Reflection Summary (Line 7+)
    process.stdout.write(`${ANSI_BOLD}${ANSI_MAGENTA}ACTIVE REFLECTIONS (Median Index):${ANSI_RESET} (Total: ${totalPersistentCount})${ANSI_CLEAR_LINE}\n`);
    const summaryLine = finalSignals.map((s, index) => {
        const color = getSignalColor(s.avgPeak);
        const classification = getSignalClassification(s.avgPeak);
        return `[R${index + 1}] ${color}${s.medianIndex.toFixed(0)}${ANSI_RESET} (Peak: ${s.avgPeak.toFixed(0)}, ${classification})`;
    }).join(' | ');

    process.stdout.write(summaryLine.padEnd(TERMINAL_WIDTH) + ANSI_CLEAR_LINE + '\n');
    process.stdout.write('--------------------------------------------------------------------------------' + ANSI_CLEAR_LINE + '\n');
}

function drawScrollingPlot() {
    let output = '';
    const plotWidth = MAX_PLOT_HISTORY_FRAMES;
    const maxIndex = MAX_PLOT_INDEX;
    
    // Calculate the starting index for the scrolling plot history.
    const startFrame = Math.max(0, plotHistory.length - plotWidth);

    // Y-axis to plot index (top is MAX_PLOT_INDEX, bottom is 0)
    for (let y = 0; y < PLOT_HEIGHT_LINES; y++) {
        let line = '';

        // Calculate Index represented by this row
        // Row 0 is MAX_PLOT_INDEX (1800), Row PLOT_HEIGHT_LINES-1 is ~0
        const indexLabel = maxIndex * (1 - y / PLOT_HEIGHT_LINES);
        line += `${indexLabel.toFixed(0).padStart(4)} |`;

        // Loop through the visible portion of the history buffer (X-axis)
        for (let i = 0; i < plotWidth; i++) {
            const frame = plotHistory[startFrame + i];
            let char = ' ';
            let minReflectionIndex = Infinity;

            if (frame) {
                // Calculate dead zone row for the current frame
                const deadZoneIndex = frame.deadZoneIndex;
                
                // Normalize dead zone index to plot row 'y'
                // This formula inverts the Y axis: higher index (closer to maxIndex) is lower on the screen (higher y)
                const deadZoneRow = Math.floor((maxIndex - deadZoneIndex) / maxIndex * PLOT_HEIGHT_LINES);

                // Draw a dashed line to indicate the dead zone boundary
                if (y === deadZoneRow) {
                    // Use the dedicated dead zone color (Blue)
                    char = DEAD_ZONE_COLOR + '-' + ANSI_RESET;
                }

                // Check all persistent signals in this frame
                frame.finalSignals.forEach((signal, index) => {
                    const medianIndex = signal.medianIndex;
                    const reflectionIndex = index + 1; // 1, 2, 3...

                    // SCALING LOGIC: map median index to the plot row 'y'
                    const normalizedY = Math.floor((maxIndex - medianIndex) / maxIndex * PLOT_HEIGHT_LINES);

                    // Check if this signal falls on the current row 'y'
                    if (normalizedY === y && medianIndex <= maxIndex) {
                        // Plot the lowest reflection index (R1 is primary)
                        if (reflectionIndex < minReflectionIndex) {
                            minReflectionIndex = reflectionIndex;
                        }
                    }
                });

                // Set the character based on the highest priority signal found
                if (minReflectionIndex !== Infinity) {
                    // Use a simple dot ('.') for historical points
                    const plotChar = '.'; 
                    const reflectionSignal = frame.finalSignals[minReflectionIndex - 1];
                    // Use the W/Y/O/R color grading based on avgPeak
                    const color = getSignalColor(reflectionSignal.avgPeak); 

                    if (i === plotWidth - 1) {
                        // Use '>' for the newest data point at the far right
                        char = color + '>' + ANSI_RESET;
                    } else {
                        // Plot the signal (now a dot) and it will overwrite the dead zone dash if they collide.
                        char = color + plotChar + ANSI_RESET;
                    }
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
        const frame = plotHistory[startFrame + i];
        if (i % 5 === 0 && frame) { // Label every 5 frames
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

function updateConsoleDashboard(latencyUs, finalSignals, noiseFloor) {
    const totalPersistentCount = finalSignals.length;
    // Use medianIndex instead of medianDistance
    const primaryIndex = finalSignals.length > 0 ? finalSignals[0].medianIndex : 0;
    const smoothedIndex = applyExponentialSmoothing(primaryIndex);

    // 1. Update history buffer
    if (plotHistory.length >= MAX_PLOT_HISTORY_FRAMES) {
        plotHistory.shift();
    }
    plotHistory.push({
        smoothedIndex: smoothedIndex, // Storing index instead of distance
        finalSignals: finalSignals,
        frameIndex: pointIndex,
        deadZoneIndex: dynamicDeadZoneIndex, // Store the dynamic index for plotting history
    });

    // 2. Redraw fixed header (y=0)
    // The number of lines has increased from 7 to 8 due to the new header line
    drawFixedHeader(latencyUs, totalPersistentCount, smoothedIndex, finalSignals, noiseFloor);

    // 3. Draw plot (starts at y=9, was 8)
    const plotStartLine = 9; 
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
    
    // --- 1. DYNAMIC DEAD ZONE AND RUNNING NOISE FLOOR CALCULATION ---
    const currentFrameNoiseFloor = calculateNoiseFloor(rawSamples);
    
    // 1a. Update Running Noise Floor EMA
    if (runningNoiseFloorEMA === 0.0) {
        runningNoiseFloorEMA = currentFrameNoiseFloor;
    } else {
        runningNoiseFloorEMA = (NOISE_FLOOR_EMA_ALPHA * currentFrameNoiseFloor) + 
                               ((1 - NOISE_FLOOR_EMA_ALPHA) * runningNoiseFloorEMA);
    }

    // 1b. Update Min/Max Noise Floor Tracking
    if (currentFrameNoiseFloor < minNoiseFloor) {
        minNoiseFloor = currentFrameNoiseFloor;
    }
    if (currentFrameNoiseFloor > maxNoiseFloor) {
        maxNoiseFloor = currentFrameNoiseFloor;
    }

    // 1c. Blind Zone Calculation
    const deadZoneIndex = findBlindZoneEnd(rawSamples, currentFrameNoiseFloor);
    dynamicDeadZoneIndex = deadZoneIndex; // Update global state for header/plot

    // 1d. NOISE SUPPRESSION (Zeroing out noise samples)
    // We suppress samples that are after the dead zone but below the weak threshold 
    // and below the running average noise floor.
    const suppressedSamples = [...rawSamples]; // Create a mutable copy
    const suppressThreshold = runningNoiseFloorEMA; // Use the smooth, global EMA

    for (let i = dynamicDeadZoneIndex; i < suppressedSamples.length; i++) {
        const sampleValue = suppressedSamples[i];
        
        // Suppress samples that are above the *dynamic* dead zone but below the EMA noise floor.
        // This is a direct implementation of the requested noise reduction.
        if (sampleValue < VALUE_THRESHOLD_WEAK && sampleValue <= suppressThreshold) {
            // Aggressively reduce the value
            suppressedSamples[i] = sampleValue * NOISE_SUPPRESSION_FACTOR; 
            // A more aggressive (or simpler) approach is to zero it out: 
            // suppressedSamples[i] = 0;
        }
    }
    
    // Use the suppressed samples for all subsequent processing
    const processedSamples = suppressedSamples; 

    // --- 2. FIND CONSISTENT INDICES (Two-Tiered) ---
    // The dynamic dead zone index is passed here to filter out the ring-down noise.
    const consistentIndices = signalTracker.updateAndGetConsistentIndices(processedSamples, dynamicDeadZoneIndex);

    // 3. Consolidate adjacent consistent indices into groups (the single reflection point)
    // NOTE: The suppressed samples array is used here.
    const consolidatedGroups = consolidateAdjacentIndices(consistentIndices, processedSamples, currentFrameNoiseFloor);

    // 4. Update persistent tracking and get median-averaged signals for plotting
    const finalSignals = updatePersistentSignals(consolidatedGroups);

    pointIndex++;

    const endTime = performance.now();
    const latencyUs = (endTime - startTime) * 1000;

    // 5. Update the console display
    updateConsoleDashboard(latencyUs, finalSignals, currentFrameNoiseFloor);
}

// --- MAIN EXECUTION ---

function startComPortListener() {
    // Initialize the Signal Tracker instance globally
    signalTracker = new SignalTracker(
        CONSISTENCY_SAMPLES_STRONG, 
        CONSISTENCY_SAMPLES_WEAK, 
        VALUE_THRESHOLD_STRONG,
        VALUE_THRESHOLD_WEAK,
        POSITION_TOLERANCE
    );

    try {
        const port = new SerialPort({
             path: COM_PORT_PATH,
             baudRate: BAUD_RATE
        });

        port.on('open', () => {
             process.stdout.write('\x1b[2J\x1b[0;0H');
             console.log(`[INIT] Serial port opened successfully on ${COM_PORT_PATH} at ${BAUD_RATE} Bps.`);
             console.log(`[INIT] Max Plot Depth: ${MAX_PLOT_INDEX} samples. Plot History: ${MAX_PLOT_HISTORY_FRAMES} frames.`);
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
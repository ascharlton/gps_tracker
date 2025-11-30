// sonar-perf-monitor.js
// Node.js script for fixed terminal statistics, median tracking, and expanded X-Y plot.
// FIX: Corrected RangeError by reading 16-bit samples iteratively (alignment safe).

const { SerialPort } = require('serialport'); 
const { performance } = require('perf_hooks');
const readline = require('readline'); 

// --- CONFIGURATION PARAMETERS (Algorithm - From console_1800_finding_distant_weak_signals.js) ---
let VALUE_THRESHOLD = 110;          // Min value a sample must have (THRESHOLD)
const CONSISTENCY_SAMPLES = 10;    // Min frames required for an index to be consistent (CONSISTENCY_SAMPLES)
const POSITION_TOLERANCE = 2;      // Range (+/-) within which a peak position is considered the same (POSITION_TOLERANCE)
let EMA_ALPHA = 0.4;               // Smoothing factor for the Stable Signal line (Header Only)

// --- SERIAL CONFIGURATION ---
const COM_PORT_PATH = '/dev/ttyACM0'; 
const BAUD_RATE = 250000;
// Sonar Physical Constants 
const SAMPLE_RESOLUTION = 0.2178; // cm/sample
const MAX_PLOT_DISTANCE_CM = 250; // Plot ceiling (0 to 250 cm)
const ORIGINAL_SAMPLE_COUNT = 1800;
const NUM_SAMPLES = ORIGINAL_SAMPLE_COUNT;
const NOISE_FLOOR_RANGE = 100; 
// Serial Packet Structure
const HEADER_BYTE = 0xAA;
const PAYLOAD_LEN = 6 + 2 * NUM_SAMPLES;
const PACKET_LEN = 1 + PAYLOAD_LEN + 1; // Header (1) + Payload + Checksum (1)

// --- CONSOLE PLOTTING CONFIGURATION ---
const PLOT_HISTORY_FRAMES = 40; // How many frames (time) to show on the X-axis
const PLOT_HEIGHT_LINES = 25; // Y-axis height (1 line = ~10cm)
const MIN_TERMINAL_WIDTH = 90; // Minimum required width (40 plot columns + padding/stats)
const CONSOLIDATION_TOLERANCE = 15; // Max indices between peaks to be considered one reflection (~1.1 cm)

// --- GLOBAL STATE ---
let serialBuffer = Buffer.alloc(0);
let pointIndex = 0;
let smoothedDistance = 0;
let signalTracker = null;

const plotHistory = []; // Array of size PLOT_HISTORY_FRAMES. Each entry is an array of persistent signals.
const allSignalHistory = []; 

// --- ANSI ESCAPE CODES (for terminal control) ---
const ANSI_CLS = '\x1b[2J\x1b[0;0H'; // Clear screen and move cursor to (0,0)
const ANSI_RESET = '\x1b[0m';
const ANSI_FG_WHITE = '\x1b[37m';
const ANSI_FG_GRAY = '\x1b[90m';
const ANSI_FG_CYAN = '\x1b[36m';
const ANSI_FG_YELLOW = '\x1b[33m';
const ANSI_FG_GREEN = '\x1b[32m';
const ANSI_FG_RED = '\x1b[31m';
const ANSI_BG_BLUE = '\x1b[44m';
const ANSI_BG_BLACK = '\x1b[40m';

// --- UTILITY FUNCTIONS ---

/**
 * Calculates the median of an array of numbers.
 * @param {number[]} arr - The array of numbers.
 * @returns {number} The median value.
 */
function median(arr) {
    if (!arr || arr.length === 0) return NaN;
    const sorted = [...arr].sort((a, b) => a - b);
    const middle = Math.floor(sorted.length / 2);

    if (sorted.length % 2 === 0) {
        return (sorted[middle - 1] + sorted[middle]) / 2;
    }
    return sorted[middle];
}

/**
 * Applies Exponential Moving Average (EMA) smoothing.
 * @param {number} newValue - The current primary distance measurement.
 * @returns {number} The smoothed distance.
 */
function applyExponentialSmoothing(newValue) {
    if (newValue === 0) {
        return smoothedDistance; // Keep old value if new value is 0 (no primary signal)
    }

    if (smoothedDistance === 0) {
        smoothedDistance = newValue; // Initialize on first valid read
    } else {
        smoothedDistance = EMA_ALPHA * newValue + (1 - EMA_ALPHA) * smoothedDistance;
    }
    return smoothedDistance;
}

/**
 * Converts a sample index to distance in centimeters.
 * @param {number} index - The sample index.
 * @returns {number} Distance in cm.
 */
function indexToCm(index) {
    return index * SAMPLE_RESOLUTION;
}

/**
 * Signal Tracker Class
 * Maintains a persistent list of sonar returns over time using consolidation and buffer tracking.
 */
class SignalTracker {
    constructor(consistencySamples, valueThreshold, positionTolerance) {
        this.consistencySamples = consistencySamples;
        this.valueThreshold = valueThreshold;
        this.positionTolerance = positionTolerance;
        // Map: {id: {indexBuffer: [idx, idx...], peakValue: number, age: number, primaryIndex: number, missedFrames: number}}
        this.signals = new Map(); 
        this.nextSignalId = 1;
    }

    /**
     * Attempts to consolidate closely spaced peaks into a single, representative index.
     * @param {Array<{index: number, value: number}>} rawPeaks - Array of detected peaks.
     * @returns {Array<{index: number, value: number}>} Array of consolidated peaks.
     */
    consolidatePeaks(rawPeaks) {
        if (rawPeaks.length === 0) return [];
        const sortedPeaks = rawPeaks.sort((a, b) => a.index - b.index);
        const consolidated = [];

        let currentConsolidation = [];
        for (const peak of sortedPeaks) {
            if (currentConsolidation.length === 0) {
                currentConsolidation.push(peak);
            } else {
                const lastPeak = currentConsolidation[currentConsolidation.length - 1];
                // Check if the current peak is within CONSOLIDATION_TOLERANCE of the last peak
                if (peak.index - lastPeak.index <= CONSOLIDATION_TOLERANCE) {
                    currentConsolidation.push(peak);
                } else {
                    // Finalize the current group: pick the strongest peak
                    const bestPeak = currentConsolidation.reduce((a, b) => (a.value > b.value ? a : b));
                    consolidated.push(bestPeak);
                    currentConsolidation = [peak];
                }
            }
        }
        // Don't forget the last group
        if (currentConsolidation.length > 0) {
            const bestPeak = currentConsolidation.reduce((a, b) => (a.value > b.value ? a : b));
            consolidated.push(bestPeak);
        }

        return consolidated;
    }

    /**
     * Processes a new set of sonar data (1800 samples) to update signal tracking.
     * @param {number[]} sampleValues - Array of 16-bit sample values.
     * @returns {{totalPersistentCount: number, primaryValueForSmoothing: number, allPersistentMedianDistances: Array<{distanceCm: number, signalId: number}>, detailedPersistentSignals: Map}}
     */
    processFrame(sampleValues) {
        // 1. Peak Detection
        let rawPeaks = [];
        for (let i = NOISE_FLOOR_RANGE; i < NUM_SAMPLES - 1; i++) {
            if (sampleValues[i] > this.valueThreshold) {
                // Simple peak detection: check if i is a local maximum
                if (sampleValues[i] >= sampleValues[i-1] && sampleValues[i] >= sampleValues[i+1]) {
                     rawPeaks.push({ index: i, value: sampleValues[i] });
                }
            }
        }
        const currentPeaks = this.consolidatePeaks(rawPeaks);

        const activeSignals = new Set();

        // 2. Match Peaks to Existing Signals
        for (const peak of currentPeaks) {
            let matched = false;
            for (const [id, signal] of this.signals) {
                // Check if peak is within tolerance of the signal's running index median
                const signalIndexMedian = median(signal.indexBuffer);
                if (!isNaN(signalIndexMedian) && Math.abs(peak.index - signalIndexMedian) <= this.positionTolerance) {
                    // Match found: update buffer, reset miss count, and track as active
                    signal.indexBuffer.push(peak.index);
                    signal.primaryIndex = peak.index;
                    signal.peakValue = peak.value;
                    signal.missedFrames = 0;
                    signal.age++;
                    activeSignals.add(id);
                    matched = true;
                    break;
                }
            }
            
            // If no match found, create a new potential signal
            if (!matched) {
                const newSignalId = this.nextSignalId++;
                this.signals.set(newSignalId, {
                    indexBuffer: [peak.index],
                    peakValue: peak.value,
                    primaryIndex: peak.index,
                    missedFrames: 0,
                    age: 1
                });
                activeSignals.add(newSignalId);
            }
        }

        // 3. Age and Prune Signals
        let totalPersistentCount = 0;
        let primaryValueForSmoothing = 0; 
        const allPersistentMedianDistances = [];

        // Use a temporary list of IDs to safely iterate and delete from the Map
        const signalIdsToDelete = [];

        for (const [id, signal] of this.signals) {
            if (!activeSignals.has(id)) {
                signal.missedFrames++;
                signal.age++;
            }

            // Trim buffer to CONSISTENCY_SAMPLES
            if (signal.indexBuffer.length > this.consistencySamples) {
                signal.indexBuffer.shift();
            }

            // If a signal is missed too many times (lost track) or buffer is too short, mark for deletion
            if (signal.missedFrames > 5 && signal.indexBuffer.length < this.consistencySamples) {
                 signalIdsToDelete.push(id);
                 continue;
            }

            // Check persistence based on buffer length
            if (signal.indexBuffer.length >= this.consistencySamples) {
                totalPersistentCount++;
                const medianIndex = median(signal.indexBuffer);
                const medianCm = indexToCm(medianIndex);
                
                allPersistentMedianDistances.push({
                    distanceCm: medianCm,
                    signalId: id 
                });
            }
        }
        
        // Delete marked signals
        signalIdsToDelete.forEach(id => this.signals.delete(id));

        // Sort persistent signals by distance (closest first)
        allPersistentMedianDistances.sort((a, b) => a.distanceCm - b.distanceCm);
        
        // Set the primary value for EMA smoothing (the closest persistent signal)
        if (allPersistentMedianDistances.length > 0) {
             primaryValueForSmoothing = allPersistentMedianDistances[0].distanceCm;
        } else {
             primaryValueForSmoothing = 0; // No primary signal this frame
        }

        return {
            totalPersistentCount,
            primaryValueForSmoothing,
            allPersistentMedianDistances,
            detailedPersistentSignals: this.signals
        };
    }
}

/**
 * Draws the real-time sonar dashboard to the console.
 * @param {number} latencyUs - Processing latency in microseconds.
 * @param {number} persistentCount - Number of persistent signals detected.
 * @param {number} smoothedDistance - EMA smoothed distance of the primary signal.
 * @param {Array<{distanceCm: number, signalId: number}>} allPersistentDistances - Array of persistent signal distances.
 * @param {Map} allSignals - Detailed signal tracking data.
 */
function updateConsoleDashboard(latencyUs, persistentCount, smoothedDistance, allPersistentDistances, allSignals) {
    // --- 1. PREP & CONSOLE CLEAR ---
    process.stdout.write(ANSI_CLS); 

    // Add new persistent signal data to history for plotting
    allSignalHistory.push(allPersistentDistances.map(s => ({
        distanceCm: s.distanceCm,
        signalId: s.signalId,
        isPrimary: s.signalId === allPersistentDistances[0]?.signalId
    })));
    if (allSignalHistory.length > PLOT_HISTORY_FRAMES) {
        allSignalHistory.shift();
    }
    
    // --- TERMINAL WIDTH CHECK ---
    const terminalWidth = process.stdout.columns || 80;
    if (terminalWidth < MIN_TERMINAL_WIDTH) {
        console.log(`${ANSI_FG_RED}${ANSI_BG_BLACK}⚠️  TERMINAL TOO NARROW! Requires >= ${MIN_TERMINAL_WIDTH} columns. Current: ${terminalWidth}. Plot may be corrupted.${ANSI_RESET}\n`);
    }


    // --- 2. HEADER/STATS SECTION ---
    let output = '';
    
    // Header Line
    const primaryCm = allPersistentDistances.length > 0 ? allPersistentDistances[0].distanceCm.toFixed(2) : '---';
    const smoothedCm = smoothedDistance > 0 ? smoothedDistance.toFixed(2) : '---';

    output += `${ANSI_BG_BLUE}${ANSI_FG_WHITE} FRAME: ${pointIndex} ${ANSI_RESET} | `;
    output += ` ${ANSI_FG_CYAN}LATENCY: ${latencyUs.toFixed(2)} μs ${ANSI_RESET} | `;
    output += ` ${ANSI_FG_GREEN}PRIMARY CM: ${primaryCm} ${ANSI_RESET} | `;
    output += ` ${ANSI_FG_YELLOW}SMOOTHED CM: ${smoothedCm} ${ANSI_RESET} | `;
    output += ` ${ANSI_FG_WHITE}PERSISTENT TRACKS: ${persistentCount} ${ANSI_RESET}\n`;

    // Configuration Line
    output += `${ANSI_FG_GRAY}CFG: T=${VALUE_THRESHOLD} V=${CONSISTENCY_SAMPLES} P=${POSITION_TOLERANCE} A=${EMA_ALPHA} | Port: ${COM_PORT_PATH} | Samples: ${NUM_SAMPLES} (${MAX_PLOT_DISTANCE_CM} cm Max)${ANSI_RESET}\n`;
    output += '\n';

    // --- 3. X-Y HISTORY PLOT ---
    output += `${ANSI_FG_WHITE}Distance Over Time (Max ${MAX_PLOT_DISTANCE_CM} cm):\n`;

    const maxPlotHeight = PLOT_HEIGHT_LINES;
    const cmPerLine = MAX_PLOT_DISTANCE_CM / maxPlotHeight;
    const historyLength = allSignalHistory.length;
    const plotWidth = PLOT_HISTORY_FRAMES;

    // Draw the Y-axis and plot lines
    for (let i = 0; i < maxPlotHeight; i++) {
        const cmValue = MAX_PLOT_DISTANCE_CM - (i * cmPerLine);
        const yCmLabel = cmValue.toFixed(0).padStart(3, ' ') + ' cm '; 

        output += `${ANSI_FG_GRAY}${yCmLabel}${ANSI_RESET} |`; 

        // Draw plot cells
        for (let j = 0; j < plotWidth; j++) {
            let char = ' ';
            let color = ANSI_FG_GRAY;

            const frameIndex = historyLength - plotWidth + j;
            if (frameIndex >= 0 && frameIndex < historyLength) {
                const signalsInFrame = allSignalHistory[frameIndex];
                
                // Check for primary and secondary signals at this Y-level
                for (const signal of signalsInFrame) {
                    const signalCm = signal.distanceCm;
                    
                    if (signalCm >= cmValue - cmPerLine && signalCm < cmValue) {
                        // Signal falls within this line segment
                        const signalId = signal.signalId;
                        
                        if (signal.isPrimary) {
                            // Primary signal (closest one)
                            char = '●'; // Large dot for primary
                            color = ANSI_FG_GREEN;
                        } else {
                            // Secondary persistent signal
                            // Use the last digit of the signal ID for tracking
                            const plotChar = (signalId % 10).toString();
                            char = plotChar; 
                            color = ANSI_FG_CYAN;
                        }
                        break; // Stop checking signals for this cell
                    }
                }
            }
            output += `${color}${char}${ANSI_RESET}`;
        }
        output += '\n'; // End of plot line
    }

    // Draw X-axis
    output += '-----|' + '-'.repeat(plotWidth) + `  Frame (${historyLength}/${PLOT_HISTORY_FRAMES})\n`;
    
    // --- 4. DETAILS / OTHER SIGNALS ---
    output += `\n${ANSI_FG_WHITE}Other Persistent Signals (${persistentCount - 1 > 0 ? persistentCount - 1 : 0} found):\n`;
    
    let detailLines = [];
    if (allPersistentDistances.length > 1) {
        for (let i = 1; i < allPersistentDistances.length; i++) {
            const signal = allPersistentDistances[i];
            const signalDetails = allSignals.get(signal.signalId);
            
            if (signalDetails) {
                 const bufferMedian = median(signalDetails.indexBuffer);
                 const medianCm = indexToCm(bufferMedian).toFixed(2);
                 const indexMin = Math.min(...signalDetails.indexBuffer);
                 const indexMax = Math.max(...signalDetails.indexBuffer);
                
                 detailLines.push(
                    `${ANSI_FG_CYAN}ID ${signal.signalId}:${ANSI_RESET} ` +
                    `CM: ${medianCm} ` +
                    `| Pkts: ${signalDetails.indexBuffer.length.toString().padStart(2, '0')}/${CONSISTENCY_SAMPLES} ` +
                    `| Var: ${indexMin}-${indexMax}`
                 );
            }
        }
    } else if (allPersistentDistances.length === 0) {
        detailLines.push(`${ANSI_FG_GRAY}No stable, persistent signals detected.${ANSI_RESET}`);
    } else {
         detailLines.push(`${ANSI_FG_GRAY}Only primary signal is persistent.${ANSI_RESET}`);
    }

    output += detailLines.join(' | ') + '\n';
    output += ANSI_RESET; // Final reset

    process.stdout.write(output);
}

// --- SERIAL DATA HANDLING ---

/**
 * Handles incoming serial data chunks. Buffers and processes full packets.
 * @param {Buffer} data - Incoming data buffer.
 */
function serialBufferHandler(data) {
    const startTime = performance.now();
    serialBuffer = Buffer.concat([serialBuffer, data]);

    while (serialBuffer.length >= PACKET_LEN) {
        if (serialBuffer[0] === HEADER_BYTE) {
            // We have a header and enough buffer for a full packet
            const packet = serialBuffer.slice(0, PACKET_LEN);

            // Checksum validation
            let checksum = 0;
            for (let i = 1; i < PACKET_LEN - 1; i++) {
                checksum ^= packet[i];
            }

            if (checksum === packet[PACKET_LEN - 1]) {
                processPacket(packet, startTime);
                // Remove the processed packet
                serialBuffer = serialBuffer.slice(PACKET_LEN);
            } else {
                // Invalid checksum: Log error and attempt to resync
                console.error(`${ANSI_FG_RED}\n[ERROR] Checksum mismatch. Calculated: 0x${checksum.toString(16)}, Received: 0x${packet[PACKET_LEN - 1].toString(16)}. Resyncing...${ANSI_RESET}`);
                // Discard the bad header byte and look for the next one
                serialBuffer = serialBuffer.slice(1); 
                
                const nextHeaderIndex = serialBuffer.indexOf(HEADER_BYTE);
                 if (nextHeaderIndex !== -1) {
                     serialBuffer = serialBuffer.slice(nextHeaderIndex);
                 } else {
                     // Clear buffer if no subsequent header is found
                     serialBuffer = Buffer.alloc(0);
                 }
                return; // Stop processing this loop iteration
            }
        } else {
            // Not a header byte, discard it and shift buffer to find the next header
            const nextHeaderIndex = serialBuffer.indexOf(HEADER_BYTE);
            if (nextHeaderIndex !== -1) {
                serialBuffer = serialBuffer.slice(nextHeaderIndex);
            } else {
                // No header found, clear buffer
                serialBuffer = Buffer.alloc(0);
                break;
            }
        }
    }
}

/**
 * Processes a validated serial packet, extracts data, and updates the console.
 * @param {Buffer} packet - The validated serial packet buffer.
 * @param {number} startTime - Performance timestamp when data was received.
 */
function processPacket(packet, startTime) {
    // The samples start at byte 7 (index 1 header + 6 metadata = 7)
    const SAMPLE_START_OFFSET = 7;
    const sampleValues = new Array(NUM_SAMPLES);

    // FIX: Iteratively read 16-bit values starting from the odd offset 7.
    // This bypasses the memory alignment issue with Uint16Array and Buffer.slice()
    for (let i = 0; i < NUM_SAMPLES; i++) {
        const offset = SAMPLE_START_OFFSET + i * 2;
        // readUInt16LE handles reading from non-aligned offsets internally
        sampleValues[i] = packet.readUInt16LE(offset); 
    }

    // Run Signal Tracking Algorithm
    const result = signalTracker.processFrame(sampleValues);

    // Apply EMA smoothing to the primary signal distance
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
    // Initialize the Signal Tracker instance globally
    signalTracker = new SignalTracker(CONSISTENCY_SAMPLES, VALUE_THRESHOLD, POSITION_TOLERANCE);

    try {
        const port = new SerialPort({ 
             path: COM_PORT_PATH, 
             baudRate: BAUD_RATE
        });
        
        port.on('open', () => {
             process.stdout.write(ANSI_CLS); 
             console.log(`[INIT] Serial port opened successfully on ${COM_PORT_PATH} at ${BAUD_RATE} Bps.`);
             console.log(`[INIT] Plotting Depth vs. Frame. Max Depth Plot: ${MAX_PLOT_DISTANCE_CM} cm.`);
             console.log(`[INIT] Tracking Logic: T=${VALUE_THRESHOLD}, C=${CONSISTENCY_SAMPLES}, P=${POSITION_TOLERANCE}, A=${EMA_ALPHA}`);
             
             const terminalWidth = process.stdout.columns || 80;
             if (terminalWidth < MIN_TERMINAL_WIDTH) {
                 console.warn(`[WARNING] Your terminal width is currently ${terminalWidth} columns. The plot requires at least ${MIN_TERMINAL_WIDTH} columns and may wrap/corrupt.`);
                 console.warn(`[HINT] Please widen your terminal window for best results.`);
             }
        });

        port.on('error', (err) => { 
            process.stdout.write(ANSI_CLS); 
            console.error('\n[FATAL COM ERROR]', err.message);
            console.log(`[HINT] Check that your device is connected to ${COM_PORT_PATH} and powered on.`);
            process.exit(1); 
        });

        port.on('data', serialBufferHandler); 
        
    } catch (err) {
        console.error('\n[FATAL INIT ERROR] Could not initialize SerialPort.', err.message);
        process.exit(1);
    }
}

// Check for required module and run
try {
    require('serialport'); 
    startComPortListener();
} catch (e) {
    if (e.code === 'MODULE_NOT_FOUND') {
        console.error(`\n[FATAL] Dependency missing: ${e.message}`);
        console.log('[HINT] Please run "npm install serialport" to install required modules.');
    } else {
        console.error(`\n[FATAL] An unexpected error occurred: ${e.message}`);
    }
    process.exit(1);
}
// sonar-plot-monitor-signals.js
// Node.js console application for plotting real-time sonar reflection indices over time.

const { SerialPort } = require('serialport');
const { program } = require('commander');

// --- Sonar Hardware Configuration ---
const BAUD_RATE = 250000;
const NUM_SAMPLES = 1800; 
const PAYLOAD_LEN = 6 + 2 * NUM_SAMPLES;
const PACKET_LEN = 1 + PAYLOAD_LEN + 1;
const PACKET_HEADER = 0xAA;

// --- Signal Tracking Configuration ---
const NUM_SIGNALS = 10;
const SIGNAL_THRESHOLD = 50;            // 100 40KHz = 60, 200 KHz = 42 is optimum
const CONSISTENCY_SAMPLES = 5;          // 4-6 is good        
const POSITION_TOLERANCE = 1;          
const NOISE_FLOOR_RANGE = 300;          // 200 how far from the end of the signal to sample for noise
const MIN_SIGNAL_SEPARATION = 8;        // 15 Minimum index difference for a separate signal
const SNR_FACTOR = 2.5;                // 2.5 New: Multiplier (k) for Noise Std Dev to set the Dynamic Detection Threshold. 
// Kalman Filter Constants
const Q_PROCESS_NOISE = 0.1;           // 0.1 Process Noise (Q): Uncertainty in the model's prediction
const P_INITIAL_COVARIANCE = 10.0;      // 10.0 Initial Covariance (P): Initial uncertainty in the state estimate
const ASSOCIATION_WINDOW = 8;           // 8 How far away a raw peak can be from a predicted track (in samples)
//const R_NOISE_FACTOR = 5;             // NEW: Multiplier for Measurement Noise R (R = R_FACTOR * StdDev^2). Use >1.0 for smoother tracks.
// Blind Zone Characteristics
// ...
// Blind Zone Characteristics
// ... (rest of the constants remain the same)
// Blind Zone Characteristics
const IGNORE_FIRST_SAMPLES = 4;         // usually the first samples are below noise floor
const MAX_BZ_SEARCH_SAMPLES = 150;      // Any signals close to the blind zone can be tracked with high thresholds only

// --- Plotting Configuration ---
const MAX_PLOT_HISTORY_FRAMES = 180;    // 80 The width of the plot in frames (X-axis)
const PLOT_HEIGHT_SAMPLES = 1800;       // 1000 The maximum sample index to display (0 to 1000, max is 1800)
const PLOT_ROWS = 50;                   // 50 The height of the plot in console rows (Y-axis)
const SAMPLE_STEP = Math.ceil(PLOT_HEIGHT_SAMPLES / PLOT_ROWS); // Samples per row

// --- Data Conversion Constants ---
const SPEED_OF_SOUND = 330;
const SAMPLE_TIME = 13.2e-6;
const SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2;
let packetcounter = 0;

// --- Global State ---
// Stores an array of up to 7 signal indices for each frame, creating the history.
// Format: [[S1_idx, S2_idx, ...], [S1_idx, S2_idx, ...], ...]
const plotHistory = []; 

// --- ANSI Escape Codes for Console Formatting ---
const ANSI_RESET = '\x1b[0m';
const ANSI_RED = '\x1b[31m';
const ANSI_YELLOW = '\x1b[33m';
const ANSI_BLUE = '\x1b[34m';
const ANSI_MAGENTA = '\x1b[35m';
const ANSI_CYAN = '\x1b[36m';
const ANSI_GREEN = '\x1b[32m';

// Array of colors for the 7 signals (1-7)
const SIGNAL_COLORS = [
    ANSI_RED, ANSI_YELLOW, ANSI_GREEN, ANSI_CYAN, ANSI_BLUE, ANSI_MAGENTA, ANSI_RED, ANSI_YELLOW, ANSI_GREEN, ANSI_CYAN
];
const SIGNAL_CHARACTERS = ['.', '.', '.', '.', '.', '.', '.', '.', '.', '.'];


// --- HELPER CLASSES AND FUNCTIONS (from previous script) ---

function findBlindZoneEnd(values, noiseFloor) { // blind zone values are 30 -149, noise is running average
    const CONSISTENCY_COUNT = 1;    
    const threshold = (noiseFloor); // 38 * 1.1 -10 (41.8 -10 = 31.8) set a value that is noise
    let consecutiveCount = 0;
    // try to find where BZ ends for this frame only look until 250 MAX_BZ_SEARCH_SAMPLES
    // DONT RELY on the first few samples as they are lower than the noise floor
    for (let i = IGNORE_FIRST_SAMPLES; i < MAX_BZ_SEARCH_SAMPLES; i++) { 
        if (values[i] < threshold) {          // if value is less than noise floor average then we have found the end of the blind zone                       // 31.8
            consecutiveCount++;
            if (consecutiveCount >= CONSISTENCY_COUNT) {
               return i - CONSISTENCY_COUNT; 
            }
        } else {
            consecutiveCount = 0;
        }
    }
    return MAX_BZ_SEARCH_SAMPLES; // default
}

function calculateNoiseFloor(values) {
    const tailSamples = values.slice(NUM_SAMPLES - NOISE_FLOOR_RANGE);
    const sortedTail = tailSamples.sort((a, b) => a - b);
    const index = Math.floor(sortedTail.length * 0.05);
    const noiseFloor = sortedTail[index] || 0;
    const NOISE_FACTOR = 2.5; 
    const noiseFilterAmplitude = noiseFloor * NOISE_FACTOR;
    return { noiseFloor, noiseFilterAmplitude };
}

// --- NEW: Kalman-based Signal Tracker ---
class KalmanSignalTracker {
    constructor(maxSignals, detectionThreshold) {
        this.maxSignals = maxSignals;
        this.threshold = detectionThreshold; // Raw threshold for initial peak finding
        // Stores active Kalman Filters: Map<SignalID, {kf: KalmanFilter, amplitude: number}>
        this.tracks = new Map();
        this.nextSignalID = 0;
    }

    /**
     * Finds the raw index and amplitude of the most prominent peak in the array.
     * @param {Array<number>} values - The raw sample values.
     * @param {number} startIdx - The starting index to search from.
     * @param {number} currentTracks - Set of indices already associated with tracks.
     * @returns {{index: number, amplitude: number} | null}
     */
    findNextRawPeak(values, startIdx, currentTracks, dynamicThreshold) { // <-- ADD dynamicThreshold
        let maxAmp = 0;
        let peakIdx = -1;
        
        for (let i = startIdx; i < values.length; i++) {
             // Use the DYNAMIC threshold for detection
            if (!currentTracks.has(i) && values[i] >= dynamicThreshold) { 
                if (values[i] > maxAmp) {
                    maxAmp = values[i];
                    peakIdx = i;
                }
            }
        }
        
        if (peakIdx !== -1) {
            return { index: peakIdx, amplitude: maxAmp };
        }
        return null;
    }

    /**
     * Core tracking logic: Prediction, Association, Update.
     * @param {Array<number>} values - The raw sample values.
     * @param {number} noiseStdDev - The calculated noise standard deviation (used to set R).
     * @returns {Array<{index: number, amplitude: number}>} Array of filtered signal positions and amplitudes.
     */
    updateTracks(values, noiseStdDev, dynamicThreshold) {
        const R_dynamic = Math.pow(noiseStdDev, 2);
        //const R_dynamic = R_NOISE_FACTOR * Math.pow(noiseStdDev, 2); // Measurement Noise R is variance of noise floor
        const measurements = new Set();
        const predictedTracks = new Map();
        const nextTracks = new Map();
        
        // --- 1. Prediction Step & Collect Measurements ---
        for (const [id, data] of this.tracks) {
            data.kf.predict();
            predictedTracks.set(id, data.kf.x);
        }

        // Find all initial raw peaks above the threshold
        let searchIndex = 0;
        let associatedIndices = new Set();
        while (true) {
            const peak = this.findNextRawPeak(values, searchIndex, associatedIndices, dynamicThreshold);
            if (!peak) break;
            measurements.add(peak);
            searchIndex = peak.index + MIN_SIGNAL_SEPARATION; // Skip past the minimum separation
        }

        // --- 2. Data Association & Update Step ---
        const usedMeasurements = new Set();

        for (const [id, data] of this.tracks) {
            const predictedPos = predictedTracks.get(id);
            let bestMatch = null;
            let minDistance = Infinity;

            // Find the closest unused raw measurement to the predicted track
            for (const m of measurements) {
                const distance = Math.abs(predictedPos - m.index);
                if (distance < ASSOCIATION_WINDOW && distance < minDistance && !usedMeasurements.has(m)) {
                    minDistance = distance;
                    bestMatch = m;
                }
            }

            if (bestMatch) {
                // Update existing track with new measurement
                const filteredPos = data.kf.update(bestMatch.index, R_dynamic);
                nextTracks.set(id, { 
                    kf: data.kf, 
                    amplitude: bestMatch.amplitude, // Update amplitude with current measurement
                    filteredIndex: filteredPos 
                });
                usedMeasurements.add(bestMatch);
            } else {
                // Track lost or missed measurement: continue prediction for stability
                // The predicted state is carried over (already done in predict() call)
                nextTracks.set(id, { 
                    kf: data.kf, 
                    amplitude: data.amplitude, // Keep old amplitude
                    filteredIndex: data.kf.x // Use the predicted index
                });
            }
        }

        // --- 3. New Track Initialization (for unused measurements) ---
        if (nextTracks.size < this.maxSignals) {
            for (const m of measurements) {
                if (!usedMeasurements.has(m) && nextTracks.size < this.maxSignals) {
                    // Initialize a new KF at the raw measurement position
                    const newKF = new KalmanFilter(m.index, P_INITIAL_COVARIANCE);
                    nextTracks.set(this.nextSignalID++, {
                        kf: newKF,
                        amplitude: m.amplitude,
                        filteredIndex: m.index
                    });
                    usedMeasurements.add(m);
                }
            }
        }
        
        this.tracks = nextTracks;
        
        // --- 4. Prepare Output ---
        const finalSignals = Array.from(nextTracks.values())
            .sort((a, b) => a.filteredIndex - b.filteredIndex)
            .slice(0, this.maxSignals)
            .map(data => ({
                index: Math.round(data.filteredIndex), 
                amplitude: data.amplitude,
                P: data.kf.P // For debugging/reporting filter uncertainty
            }));

        return finalSignals;
    }
}

// --- NEW: Kalman Filter Class for 1D Position Tracking (Index) ---
class KalmanFilter {
    constructor(initialState, initialP) {
        // State: Position (Index)
        this.x = initialState; // Initial estimate (x-hat)
        this.P = initialP;     // Error Covariance (Uncertainty)
        this.Q = Q_PROCESS_NOISE; // Process Noise (fixed)
        this.R = 1.0;          // Measurement Noise (dynamic, set by caller)
    }

    /**
     * Prediction step (Constant position/random walk model)
     * Predicts the next state and covariance.
     */
    predict() {
        // Predicted State: x_k = A * x_{k-1} + B * u_k (A=1, B*u=0)
        // x remains the same in a simple constant position model
        // this.x = this.x; 

        // Predicted Covariance: P_k = A * P_{k-1} * A^T + Q (A=1)
        this.P = this.P + this.Q;
        
        return this.x; // Return predicted position
    }

    /**
     * Update step
     * Incorporates a new measurement.
     * @param {number} z - The new measurement (raw index).
     * @param {number} R_dynamic - The dynamically calculated Measurement Noise (variance).
     */
    update(z, R_dynamic) {
        this.R = R_dynamic;

        // Kalman Gain: K_k = P_k * H^T * (H * P_k * H^T + R_k)^-1 (H=1)
        const K = this.P / (this.P + this.R);

        // Update State: x_k = x_k + K_k * (z_k - H * x_k) (H=1)
        this.x = this.x + K * (z - this.x);

        // Update Covariance: P_k = (I - K_k * H) * P_k (I=1, H=1)
        this.P = (1 - K) * this.P;
        
        return this.x; // Return updated (filtered) position
    }
}
// ... (NoiseFloorAverager class remains the same)
class NoiseFloorAverager {
    constructor() {
        this.runningNoiseAvg = 0;
        this.min = Infinity;
        this.max = 0;
        this.count = 0;
        this.M2 = 0; 
    }

    update(currentNoiseFloor) {
        if (currentNoiseFloor === Infinity) return;
        this.count++;
        
        if (currentNoiseFloor < this.min) {
            this.min = currentNoiseFloor;
        }
        if (currentNoiseFloor > this.max) {
            this.max = currentNoiseFloor;
        }

        const delta = currentNoiseFloor - this.runningNoiseAvg;
        this.runningNoiseAvg += delta / this.count;
        const delta2 = currentNoiseFloor - this.runningNoiseAvg;
        this.M2 += delta * delta2;
    }

    getRunningAverage() {
        if (this.count === 0) return '0.0';
        return this.runningNoiseAvg.toFixed(1);
    }
    getMinMax() {
        if (this.count === 0) return { min: 0, max: 0 };
        return { min: Math.round(this.min), max: Math.round(this.max) };
    }
    getStandardDeviation() {
        if (this.count <= 1) return '0.0'; 
        const variance = this.M2 / this.count;
        const stdDev = Math.sqrt(Math.max(0, variance)); 
        return stdDev.toFixed(1);
    }
}

// --- SERIAL PACKET HANDLING ---
let buffer = Buffer.alloc(0); 
let isReading = false;

function extractAndProcessPacket() {
    if (isReading) return null; 

    isReading = true;
    let packet = null;

    try {
        let headerIndex = -1;
        for (let i = 0; i <= buffer.length - PACKET_LEN; i++) {
            if (buffer[i] === PACKET_HEADER) {
                headerIndex = i;
                break;
            }
        }

        if (headerIndex === -1 || buffer.length < headerIndex + PACKET_LEN) {
            isReading = false;
            return null;
        }

        const potentialPacket = buffer.subarray(headerIndex, headerIndex + PACKET_LEN);
        const payload = potentialPacket.subarray(1, 1 + PAYLOAD_LEN);
        const receivedChecksum = potentialPacket[PACKET_LEN - 1];

        let calculatedChecksum = 0;
        for (const byte of payload) {
            calculatedChecksum ^= byte;
        }

        if (calculatedChecksum !== receivedChecksum) {
            buffer = buffer.subarray(headerIndex + 1);
            isReading = false;
            return extractAndProcessPacket();
        }

        const values = [];
        for (let i = 0; i < NUM_SAMPLES; i++) {
            values.push(payload.readUInt16BE(6 + i * 2));
        }

        buffer = buffer.subarray(headerIndex + PACKET_LEN);
        packet = { values };

    } catch (e) {
        console.error("Critical error during packet processing:", e);
    }

    isReading = false;
    return packet;
}

// --- NEW PLOTTING LOGIC ---

/**
 * Draws the real-time scrolling plot to the console.
 */
function drawScrollingPlot() {
    // Clear the console and set cursor to top left
    process.stdout.write('\x1Bc'); 

    // --- 1. Draw Header and Metrics (as before) ---
    const now = new Date();
    const timeString = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}`;
    const horizontalLine = '-'.repeat(MAX_PLOT_HISTORY_FRAMES + 9);
    
    // Print the Legend
    let legend = 'LEGEND: ';
    for(let i = 0; i < NUM_SIGNALS; i++) {
        legend += `${SIGNAL_COLORS[i]}${SIGNAL_CHARACTERS[i]}${ANSI_RESET} = Sig ${i+1} | `;
    }

    console.log(legend);
    console.log(horizontalLine);
    console.log(`Time: ${timeString} | Packet: ${packetcounter} | Samples Plotted: 0 to ${PLOT_HEIGHT_SAMPLES}`);
    console.log(horizontalLine);
    
    // --- 2. Draw Plot Body (Y-axis: Depth, X-axis: Time) ---
    const plotLines = [];
    
    // Iterate over plot rows (Y-axis, Depth) from top (0) to bottom (PLOT_HEIGHT_SAMPLES)
    for (let r = 0; r < PLOT_ROWS; r++) {
        let plotRow = '';
        
        // Calculate the depth range for this row
        const minSample = r * SAMPLE_STEP;
        const maxSample = minSample + SAMPLE_STEP;
        
        // Add Y-axis label (Depth in cm)
        const depthCm = parseFloat((minSample * SAMPLE_RESOLUTION).toFixed(1));
        plotRow += `${depthCm.toFixed(0).padStart(4, ' ')}|`;

        // Iterate over history frames (X-axis, Time) from oldest (left) to newest (right)
        for (let f = 0; f < MAX_PLOT_HISTORY_FRAMES; f++) {
            const frameData = plotHistory[f]; // Array of [S1_idx, S2_idx, ...]
            let char = ' ';
            
            if (frameData) {
                // Check each of the 7 signals
                for (let s = 0; s < NUM_SIGNALS; s++) {
                    const signalIndex = frameData[s];
                    
                    if (signalIndex !== -1 && signalIndex >= minSample && signalIndex < maxSample) {
                        // Signal is within the current row's sample range
                        char = SIGNAL_COLORS[s] + SIGNAL_CHARACTERS[s] + ANSI_RESET;
                        break; 
                    }
                }
            }
            plotRow += char;
        }
        plotLines.push(plotRow);
    }

    plotLines.forEach(line => console.log(line));
    console.log(horizontalLine);
}

// --- Main Execution Loop ---

async function runSerialConsole(portName) {
    const tracker = new KalmanSignalTracker(NUM_SIGNALS, SIGNAL_THRESHOLD);
    const noiseAverager = new NoiseFloorAverager();
    let port;

    try {
        port = new SerialPort({ path: portName, baudRate: BAUD_RATE });

        port.on('open', () => {
            process.stdout.write('\x1Bc'); // Initial screen clear
            console.log(`\n✅ Serial port opened on ${portName} at ${BAUD_RATE}.`);
            console.log(`   Config: SIGNAL_THRESHOLD=${SIGNAL_THRESHOLD}, CONSISTENCY_SAMPLES=${CONSISTENCY_SAMPLES}, SEPARATION=${MIN_SIGNAL_SEPARATION}`);
        });

        port.on('data', data => {
            buffer = Buffer.concat([buffer, data]);
            
            let packet;
            while ((packet = extractAndProcessPacket())) {
                
                const { values } = packet;
                packetcounter++;

                // 1. Calculate and Track Noise Floor
                const { noiseFloor } = calculateNoiseFloor(values);
                noiseAverager.update(noiseFloor);
                const runningNoiseAvg = noiseAverager.getRunningAverage();
                const runningStdDev = noiseAverager.getStandardDeviation(); 
                const runningNoiseAvgValue = parseFloat(runningNoiseAvg);
                const runningStdDevValue = parseFloat(runningStdDev);
                const DYNAMIC_THRESHOLD = parseFloat((runningNoiseAvgValue + runningStdDevValue * SNR_FACTOR).toFixed(1));
                const blindZoneIndexEnd = findBlindZoneEnd(values, runningNoiseAvgValue); 
                
                // 2. --- NEW: Get Kalman-Filtered Tracks and Indices ---
                // The tracker internally handles detection, association, and filtering
                const detectedSignals = tracker.updateTracks(values, runningStdDevValue, DYNAMIC_THRESHOLD);

                // Filter signals for plot: only plot signals starting AFTER the blind zone
                const currentFrameIndices = new Array(NUM_SIGNALS).fill(-1);
                
                // Get the filtered signal/amplitude pairs for the metrics report
                const currentFrameSignalsMetrics = []; 

                for (let i = 0; i < detectedSignals.length; i++) {
                    const sig = detectedSignals[i];
                    if (sig.index !== -1 && sig.index >= blindZoneIndexEnd) {
                        // For Plotting
                        currentFrameIndices[i] = sig.index;
                        
                        // For Metrics Report
                        currentFrameSignalsMetrics.push(`${sig.index} (${sig.amplitude}, P:${sig.P.toFixed(2)})`);
                    }
                }
                
                // Add to history and maintain size
                plotHistory.push(currentFrameIndices);
                while (plotHistory.length > MAX_PLOT_HISTORY_FRAMES) {
                    plotHistory.shift();
                }

                // --- 4. Plot and Report ---
                drawScrollingPlot();
                
                // Print detailed metrics below the plot
                console.log(`--- Real-Time Metrics ---`);
                console.log(`Noise Floor: ${noiseFloor} (Avg: ${runningNoiseAvg}, Std Dev: ${runningStdDev})`);
                console.log(`Dynamic Detection Threshold (k=${SNR_FACTOR}*StdDev): ${DYNAMIC_THRESHOLD}`); // <-- NEW METRIC
                console.log(`Blind Zone End Index (def: ${MAX_BZ_SEARCH_SAMPLES}): ${blindZoneIndexEnd}`);
                console.log(`Detected Signals (Index, Amplitude, P-Covariance): ${currentFrameSignalsMetrics.join(', ') || 'None'}`);
                console.log('='.repeat(MAX_PLOT_HISTORY_FRAMES + 9));
            }
        });

        port.on('error', err => {
            console.error("\n❌ Serial Port Error:", err.message);
            console.log(`Ensure device is connected and port '${portName}' is correct.`);
        });

        port.on('close', () => {
            console.log("\n❌ Serial Port Closed.");
        });

    } catch (e) {
        console.error(`\n❌ Error: ${e.message}`);
        if (port && port.isOpen) {
            port.close();
        }    
    }
}


// --- Command Line Interface ---

program
    .arguments('[port]')
    .description('Starts the Node.js serial console for the Open Echo Sonar and displays a scrolling plot.')
    .action(async (port) => {
        let selectedPort = port;
        // Port detection logic (omitted for brevity, assume runSerialConsole handles it)
        
        runSerialConsole(selectedPort || '/dev/ttyACM0'); // Use a sensible default
    });

program.parse(process.argv);
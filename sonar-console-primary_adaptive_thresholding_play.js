// sonar-plot-monitor-signals.js
// Node.js console application for plotting real-time sonar reflection indices over time.
// Doesnt pick up weak signals well but tracks strong signals well.

const { SerialPort } = require('serialport');
const { program } = require('commander');

// --- Sonar Hardware Configuration ---
const BAUD_RATE = 250000;
const NUM_SAMPLES = 1800; 
const PAYLOAD_LEN = 6 + 2 * NUM_SAMPLES;
const PACKET_LEN = 1 + PAYLOAD_LEN + 1;
const PACKET_HEADER = 0xAA;

// --- Signal Tracking Configuration ---
const NUM_SIGNALS = 3;
const SIGNAL_THRESHOLD = 60;                   // 40KHz = 60, 200 KHz = 42 is optimum
const CONSISTENCY_SAMPLES = 10;          // 4-6 is good        
const POSITION_TOLERANCE = 1;          
const NOISE_FLOOR_RANGE = 350;          // 200 how far from the end of the signal to sample for noise
const MIN_SIGNAL_SEPARATION = 20; // Minimum index difference for a separate signal
const SNR_FACTOR = 4.4; // New: Multiplier (k) for Noise Std Dev to set the Dynamic Detection Threshold.
const SONAR_FREQUENCY = 40; // 40 or 200 

// Blind Zone Characteristics
let MAX_BZ_SEARCH_SAMPLES; 
let IGNORE_FIRST_SAMPLES;
// Blind Zone Characteristics
if (SONAR_FREQUENCY == 40){
    MAX_BZ_SEARCH_SAMPLES = 300; 
    IGNORE_FIRST_SAMPLES = 8;
}else{
    MAX_BZ_SEARCH_SAMPLES = 150;
    IGNORE_FIRST_SAMPLES = 4; 
}


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

const ANSI_RESET = '\x1b[0m';

// Standard ANSI Colors (31-36)
const ANSI_RED = '\x1b[31m';
const ANSI_GREEN = '\x1b[32m';
const ANSI_YELLOW = '\x1b[33m';
const ANSI_BLUE = '\x1b[34m';
const ANSI_MAGENTA = '\x1b[35m';
const ANSI_CYAN = '\x1b[36m';

// Bright ANSI Colors (91-96)
const ANSI_BRIGHT_RED = '\x1b[91m';
const ANSI_BRIGHT_GREEN = '\x1b[92m';
const ANSI_BRIGHT_YELLOW = '\x1b[93m';
const ANSI_BRIGHT_BLUE = '\x1b[94m';
const ANSI_BRIGHT_MAGENTA = '\x1b[95m';
const ANSI_BRIGHT_CYAN = '\x1b[96m';

// --- Dynamic Color Generation ---

// The core set of 12 distinct ANSI color codes (Standard + Bright)
const CORE_COLORS = [
    ANSI_RED, ANSI_GREEN, ANSI_YELLOW, ANSI_BLUE, ANSI_MAGENTA, ANSI_CYAN,
    ANSI_BRIGHT_RED, ANSI_BRIGHT_GREEN, ANSI_BRIGHT_YELLOW, ANSI_BRIGHT_BLUE, ANSI_BRIGHT_MAGENTA, ANSI_BRIGHT_CYAN
];

// Dynamically generate the SIGNAL_COLORS array based on NUM_SIGNALS (assumed to be defined)
const SIGNAL_COLORS = [];
for (let i = 0; i < NUM_SIGNALS; i++) {
    // Cycles through the 12 CORE_COLORS using the modulo operator
    const colorIndex = i % CORE_COLORS.length;
    SIGNAL_COLORS.push(CORE_COLORS[colorIndex]);
}
const SIGNAL_CHARACTERS = new Array(NUM_SIGNALS).fill('.');


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

class SignalTracker {
    constructor(consistencySamples, threshold, tolerance) {
        this.consistencySamples = consistencySamples;
        this.threshold = threshold;
        this.tolerance = tolerance;
        this.history = new Map();
    }

    updateAndGetConsistentIndices(values) {
        const currentPeaks = new Set();
        const nextHistory = new Map();
        for (let i = 0; i < values.length; i++) {
            if (values[i] >= this.threshold) {
                currentPeaks.add(i);
            }
        }

        const consistentIndices = new Set();
        for (const [pos, data] of this.history) {
            let foundMatch = false;
            for (let i = pos - this.tolerance; i <= pos + this.tolerance; i++) {
                if (currentPeaks.has(i)) {
                    const newCount = data.count + 1;
                    const newPos = i;
                    if (newCount >= this.consistencySamples) {
                        consistentIndices.add(newPos);
                    }
                    if (!nextHistory.has(newPos)) {
                        nextHistory.set(newPos, { count: newCount });
                    }
                    foundMatch = true;
                    currentPeaks.delete(i); 
                    break;
                }
            }
        }

        for (const newPos of currentPeaks) {
            if (!nextHistory.has(newPos)) {
                nextHistory.set(newPos, { count: 1 });
            }
        }

        this.history = nextHistory;
        return consistentIndices;
    }
}

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
                // Check each of the signals
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

// --- Signal Index Extraction ---

/**
 * Executes the pulse detection logic and returns an array of up to NUM_SIGNALS 
 * objects, each containing the index and amplitude of a detected signal.
 * @param {Array<number>} values - The raw sample values.
 * @param {Set<number>} consistentIndices - The indices of consistent peaks.
 * @param {number} detectionThreshold - The dynamic threshold (Avg + StdDev * SNR_FACTOR).
 * @returns {Array<{index: number, amplitude: number}>} Array of signal objects.
 */
function extractSignalIndices(values, consistentIndices, detectionThreshold) {
    // Change to an array of objects to store both index and amplitude
    const detectedSignals = [];
    
    let lastPulseEnd = -1;
    let signalCount = 0;

    for (let i = 0; i < values.length; i++) {
        
        // Start searching for the next signal only after the separation threshold
        if (i < lastPulseEnd + MIN_SIGNAL_SEPARATION) {
            continue;
        }

        // Check for consistency AND if the amplitude is above the DYNAMIC THRESHOLD
        if (consistentIndices.has(i) && values[i] >= detectionThreshold) {
            
            // Found the start of Signal (N+1)
            const currentSignalStart = i;
            const currentSignalAmplitude = values[i]; // <-- New: Get the amplitude
            let currentPulseWidth = 0;
            let currentPulseEnd = -1;
            
            // Calculate Pulse Width and End Index of the current Signal
            for (let j = currentSignalStart; j < values.length; j++) {
                // Pulse continues if it is above the DYNAMIC THRESHOLD
                if (values[j] >= detectionThreshold) {
                    currentPulseWidth++;
                } else {
                    currentPulseEnd = j; 
                    break;
                }
            }
            // Handle case where pulse extends to the end of the array
            if (currentPulseEnd === -1 && currentPulseWidth > 0) {
                currentPulseEnd = values.length - 1;  // 1800
            }
            
            // Store the index and amplitude in the new array
            detectedSignals.push({
                index: currentSignalStart,
                amplitude: currentSignalAmplitude, // <-- New: Store amplitude
                pulsewidth: currentPulseWidth
            });
            lastPulseEnd = currentPulseEnd;
            signalCount++;
            
            // Break if we've found all NUM_SIGNALS
            if (signalCount === NUM_SIGNALS) break;
            
            // Skip forward to avoid searching within the current pulse
            i = currentPulseEnd;
        }
    }
    
    // Pad with -1 index/amplitude objects to maintain NUM_SIGNALS length for the plot
    while (detectedSignals.length < NUM_SIGNALS) {
        detectedSignals.push({ index: -1, amplitude: -1 , pulsewidth: -1});
    }

    return detectedSignals;
}

// --- Main Execution Loop ---

async function runSerialConsole(portName) {
    const tracker = new SignalTracker(CONSISTENCY_SAMPLES, SIGNAL_THRESHOLD, POSITION_TOLERANCE);
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
                
                // --- NEW: Calculate Dynamic Threshold using Std Dev for Noise Resilience ---
                // Threshold = Noise Avg + (Noise Std Dev * SNR_FACTOR)
                const DYNAMIC_THRESHOLD = parseFloat((runningNoiseAvgValue + runningStdDevValue * SNR_FACTOR).toFixed(1));
                
                const blindZoneIndexEnd = findBlindZoneEnd(values, runningNoiseAvgValue); 
                
                // 2. Get Consistent Indices (Still uses constant SIGNAL_THRESHOLD for temporal tracking)
                const consistentIndices = tracker.updateAndGetConsistentIndices(values);

                // --- 3. Extract Signals and Update Plot History ---
                // Now uses the DYNAMIC_THRESHOLD for amplitude checking
                const detectedSignals = extractSignalIndices(values, consistentIndices, DYNAMIC_THRESHOLD);
                
                // Filter signals for plot: only plot signals starting AFTER the blind zone
                // We map the array of objects back to an array of indices for the existing plotHistory logic.
                //const currentFrameIndices = detectedSignals.map(sig => (sig.index !== -1 && sig.index >= blindZoneIndexEnd) ? sig.index : -1);
                const currentFrameIndices = detectedSignals.map(sig => (sig.index !== -1 && sig.index > 0) ? sig.index : -1);
                
                // Get the filtered signal/amplitude pairs for the metrics report
                //const currentFrameSignalsMetrics = detectedSignals.filter(sig => sig.index !== -1 && sig.index >= blindZoneIndexEnd).map(sig => `${sig.index} (${sig.amplitude})`); // Formatting: Index (Amplitude)
                const currentFrameSignalsMetrics = detectedSignals.filter(sig => sig.index !== -1 && sig.index > 0).map(sig => `${sig.index} | ${sig.amplitude} | ${sig.pulsewidth}`); // Formatting: Index (Amplitude)
                
                // Add to history and maintain size
                plotHistory.push(currentFrameIndices);
                while (plotHistory.length > MAX_PLOT_HISTORY_FRAMES) {
                    plotHistory.shift();
                }

                // --- 4. Plot and Report ---
                drawScrollingPlot();
                
                // Print detailed metrics below the plot
                console.log(`--- Real-Time Metrics --- ${SONAR_FREQUENCY}KHz`);
                console.log(`Noise Floor: ${noiseFloor} (Avg: ${runningNoiseAvg}, Std Dev: ${runningStdDev})`);
                console.log(`Dynamic Detection Threshold (k=${SNR_FACTOR}*StdDev): ${DYNAMIC_THRESHOLD}`);
                console.log(`Blind Zone End Index (def: ${MAX_BZ_SEARCH_SAMPLES}): ${blindZoneIndexEnd}`);
                console.log(`Detected Signals (Index, Amplitude, Pulse Width): ${currentFrameSignalsMetrics.join(', ') || 'None'}`);
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
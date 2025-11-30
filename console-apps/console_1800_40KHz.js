// Node.js implementation equivalent to the Python serial console script.

const { SerialPort } = require('serialport');
const { program } = require('commander');
const util = require('util');

// --- Configuration (MUST MATCH ARDUINO FIRMWARE) ---
const BAUD_RATE = 250000;
const NUM_SAMPLES = 1800; // CHANGED to 1800 samples as requested
const PAYLOAD_LEN = 6 + 2 * NUM_SAMPLES;
const PACKET_LEN = 1 + PAYLOAD_LEN + 1; // Header (1) + Payload + Checksum (1)
const PACKET_HEADER = 0xAA;

// --- TRACKING CONFIGURATION ---
const THRESHOLD = 60;                  //  Minimum value a point must have to be considered a peak
const CONSISTENCY_SAMPLES = 3;         // 3 Minimum number of samples a peak must appear in to be 'consistent'
const POSITION_TOLERANCE = 1;          // 5 Range (+/-) within which a peak position is considered the same

// Noise Calculations
const NOISE_FLOOR_RANGE = 200;         // Number of tail samples to use for noise floor calculation

// Constants used for data conversion
const SPEED_OF_SOUND = 330;
const SAMPLE_TIME = 13.2e-6;
const SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2;
const MAX_DEPTH = NUM_SAMPLES * SAMPLE_RESOLUTION  // Total depth in cm;

let packetcounter = 0;

function calculateNoiseFloor(samples) {
    const start = samples.length - NOISE_FLOOR_RANGE;
    
    if (start < 0) return 0; // Safety check
    const tailSamples = samples.slice(start);
    const sum = tailSamples.reduce((acc, val) => acc + val, 0);
    return Math.round(sum / NOISE_FLOOR_RANGE);
}

function findBlindZoneEnd(values, noiseFloor) {
    // Blind zone ends when N consecutive samples are below (Noise Floor + a margin).
    const MARGIN_FACTOR = 1.2; 
    const CONSISTENCY_COUNT = 5; 
    const threshold = noiseFloor * MARGIN_FACTOR;
    let consecutiveCount = 0;

    for (let i = 0; i < MAX_BLIND_ZONE_SEARCH_SAMPLES; i++) {
        if (values[i] < threshold) {
            consecutiveCount++;
            if (consecutiveCount >= CONSISTENCY_COUNT) {
                // Return the start of the consistent low-value zone
                return i - CONSISTENCY_COUNT; 
            }
        } else {
            consecutiveCount = 0;
        }
    }
    // If not found in the search area, default to the maximum search area end
    return MAX_BLIND_ZONE_SEARCH_SAMPLES;
}

class NoiseFloorAverager {
    constructor() {
        this.runningNoiseAvg = 0;
        this.min = Infinity;
        this.max = 0;
        this.count = 0;
        this.M2 = 0; // For Welford's algorithm (variance)
    }

    update(currentNoiseFloor) {
        // Guard against Infinity from initial state
        if (currentNoiseFloor === Infinity) return;

        this.count++;
        
        // 1. Update Min/Max
        if (currentNoiseFloor < this.min) {
            this.min = currentNoiseFloor;
        }
        if (currentNoiseFloor > this.max) {
            this.max = currentNoiseFloor;
        }

        // 2. Update Running Average (and Variance via Welford's)
        const delta = currentNoiseFloor - this.runningNoiseAvg;
        this.runningNoiseAvg += delta / this.count;
        const delta2 = currentNoiseFloor - this.runningNoiseAvg;
        this.M2 += delta * delta2;
    }

    /**
     * Calculates the running average.
     */
    getRunningAverage() {
        if (this.count === 0) return '0.0';
        return this.runningNoiseAvg.toFixed(1);
    }

    /**
     * Returns the minimum and maximum noise floor values recorded.
     */
    getMinMax() {
        if (this.count === 0) {
            return { min: 0, max: 0 };
        }
        return {
            min: Math.round(this.min),
            max: Math.round(this.max),
        };
    }
    
    /**
     * Calculates the population standard deviation of the running noise floor.
     */
    getStandardDeviation() {
        if (this.count <= 1) return '0.0'; 

        // Variance is M2 / count
        const variance = this.M2 / this.count;
        
        const stdDev = Math.sqrt(Math.max(0, variance)); 

        return stdDev.toFixed(1);
    }
}
// --- Signal Tracker Class for Time-Series Analysis ---

class SignalTracker {
    /**
     * Manages a buffer of past sample arrays and identifies consistent signal indices.
     */
    constructor(bufferSize, threshold, tolerance) {
        this.buffer = [];
        this.bufferSize = bufferSize; // CONSISTENCY_SAMPLES
        this.threshold = threshold;
        this.tolerance = tolerance;
        this.consistentIndices = new Set();
    }

    // The issue with this is its not considering pulse widths or an average of the pulse width
    // peak values can be spread all over the same signal pulse width and look like separate signals
    // we should find the signal, simplify it ans summarise the pulse widht to a single value for display.
    updateAndGetConsistentIndices(currentValues) { // all samples here
        // 1. Identify current 'peak' indices (above threshold)
        const currentPeakIndices = [];
        for (let i = 0; i < currentValues.length; i++) {
            if (currentValues[i] >= this.threshold) {
                currentPeakIndices.push(i);
            }
        }

        // 2. Update buffer (Store the indices, not the full values array) Just keep CONSISTENCY_SAMPLES size array
        this.buffer.push(currentPeakIndices);
        if (this.buffer.length > this.bufferSize) {
            this.buffer.shift(); // Remove oldest peak list
        }

        // Skip analysis until the buffer is full
        if (this.buffer.length < this.bufferSize) {
            this.consistentIndices.clear();
            return this.consistentIndices;
        }

        // 3. Analyze Consistency across all buffered samples
        const indexCounts = {};

        // Iterate through the latest peaks (which is the last element in the buffer)
        for (const index of this.buffer[this.buffer.length - 1]) {
            let isConsistent = true;
            // Check this peak index against ALL *previous* buffers
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
                // If consistent across all past frames, this index is significant.
                // We use the current index as the canonical consistent position.
                indexCounts[index] = (indexCounts[index] || 0) + 1;
            }
        }

        // 4. Final selection: indices that passed the check
        this.consistentIndices = new Set(Object.keys(indexCounts).map(Number));
        return this.consistentIndices;
    }
}


// --- Core Packet Reading Function (Node.js/SerialPort style) ---

let buffer = Buffer.alloc(0); // Persistent buffer for incomplete data
let isReading = false;

/**
 * Attempts to extract a single valid sonar packet from the buffer.
 * @returns {Object|null} The parsed packet data or null if a full packet isn't available.
 */
function extractAndProcessPacket() {
    if (isReading) return null; // Prevent re-entry

    isReading = true;
    let packet = null;

    try {
        // 1. Find Header (0xAA)
        let headerIndex = -1;
        for (let i = 0; i <= buffer.length - PACKET_LEN; i++) {
            if (buffer[i] === PACKET_HEADER) {  // AA
                headerIndex = i;
                break;
            }
        }

        if (headerIndex === -1 || buffer.length < headerIndex + PACKET_LEN) {
            // No full packet found yet
            isReading = false;
            return null;
        }

        // Extract potential packet (Header + Payload + Checksum)
        const potentialPacket = buffer.subarray(headerIndex, headerIndex + PACKET_LEN);
        const payload = potentialPacket.subarray(1, 1 + PAYLOAD_LEN);
        const receivedChecksum = potentialPacket[PACKET_LEN - 1];

        // 2. Verify Checksum
        let calculatedChecksum = 0;
        for (const byte of payload) {
            calculatedChecksum ^= byte;
        }

        if (calculatedChecksum !== receivedChecksum) {
            // Checksum mismatch: discard the bad header and continue search
            console.error(`\n⚠️ Checksum mismatch! Calculated: ${calculatedChecksum.toString(16).toUpperCase()}, Received: ${receivedChecksum.toString(16).toUpperCase()}`);
            buffer = buffer.subarray(headerIndex + 1);
            isReading = false;
            return extractAndProcessPacket(); // Try again from remaining buffer
        }

        // Samples start at offset 6. Unpack as an array of 16-bit unsigned integers (H)
        const values = [];
        // let sum = 0;
        for (let i = 0; i < NUM_SAMPLES; i++) {
            values.push(payload.readUInt16BE(6 + i * 2));
            // sum += values[i];
            // if (values.length == 400){
            //     console.log(util.inspect(values, { maxArrayLength: null }))
            //     console.log("sum",sum);
            //     console.log("avg",sum/(i+1));
            //     if((sum/i) > 100){
            //         //process.exit();
            //     }
            // }
        }

        // 5. Cleanup buffer and return
        buffer = buffer.subarray(headerIndex + PACKET_LEN);
        packet = { values };

    } catch (e) {
        console.error("Critical error during packet processing:", e);
    }

    isReading = false;
    return packet;
}

// --- Main Execution Loop ---

async function runSerialConsole(portName) {
    console.log(`Attempting to connect to: ${portName} @ ${BAUD_RATE} baud...`);
    console.log(`Tracking: Threshold=${THRESHOLD}, Consistency=${CONSISTENCY_SAMPLES}, Tolerance=${POSITION_TOLERANCE}`);

    // Initialize the Signal Tracker
    const tracker = new SignalTracker(CONSISTENCY_SAMPLES, THRESHOLD, POSITION_TOLERANCE);
    const noiseAverager = new NoiseFloorAverager();
    let port;

    try {
        port = new SerialPort({ path: portName, baudRate: BAUD_RATE, autoOpen: false });

        port.open(err => {
            if (err) {
                throw new Error(`Failed to open serial port ${portName}: ${err.message}`);
            }
            console.log(`✅ Connected to ${portName}. Reading data...`);
        });

        port.on('data', data => {
            // Append incoming data to the buffer
            buffer = Buffer.concat([buffer, data]);
            
            // Process the buffer as long as complete packets are found
            let packet;
            while ((packet = extractAndProcessPacket())) {
                
                const { values } = packet;

                packetcounter++;
                // if(packetcounter < 5){
                //     console.log(`checking packet: ${packetcounter}`);
                //     console.log(packet);
                //     if(packetcounter == 4){
                //         process.exit();
                //     }
                // }
                
                // calculate noise floor average
                const noiseFloor = calculateNoiseFloor(values);
                noiseAverager.update(noiseFloor);
                const runningNoiseAvg = noiseAverager.getRunningAverage();
                const { min, max } = noiseAverager.getMinMax(); 
                const runningStdDev = noiseAverager.getStandardDeviation(); 
                const runningNoiseAvgValue = parseFloat(runningNoiseAvg);
                const runningStdDevValue = parseFloat(runningStdDev);
                const noiseFilterAmplitude = parseFloat((runningNoiseAvgValue + runningStdDevValue).toFixed(1));
                console.log(`noise: ${noiseFloor}| average: ${runningNoiseAvg}| Min: ${min}, Max: ${max}| Std Dev: ${runningStdDevValue}| noise+std dev: ${noiseFilterAmplitude}`);
                // find the end of the blind zone
                //const blindZoneIndexEnd = findBlindZoneEnd(values, runningNoiseAvgValue); 
                //console.log(blindZoneIndexEnd);
                // 1. Get Consistent Indices
                // Need to change this to: 
                // 1. finding the signal (consistent in one place for at least 3 frames),  
                // 2. summarize it (reduce pulse width to single item), 
                // 3. track and display a single line

                // We build up a buffer first (CONSISTENCY) then check this packets values are found in 
                // that buffer, then print
                const consistentIndices = tracker.updateAndGetConsistentIndices(values);

                // 2. Create Highlighted String
                const highlightedSamples = [];
                for (let i = 0; i < values.length; i++) {
                    const value = values[i];
                    const isConsistent = consistentIndices.has(i);

                    if (isConsistent && value >= THRESHOLD) {
                        // Highlight: Show value for consistent, high-value data
                        output = ` \x1b[31m${value}\x1b[0m `;    // red
                        highlightedSamples.push(`${output.toString().padStart(4, ' ')}`);
                    } else if (value >= THRESHOLD) {
                        // Show a marker for high-value but is not consistent
                        highlightedSamples.push("  . ");
                    } else {
                        // Show a dot for low-value data (below threshold)
                        highlightedSamples.push("    ");
                    }
                }
                
                const samplesOutput = highlightedSamples.join("");
                
                // --- CONSOLE OUTPUT ---
                const now = new Date();
                const timeString = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}`;
                
                console.log("\n" + "=".repeat(100));
                console.log(`Time: ${timeString} | Sample Resolution: ${SAMPLE_RESOLUTION} `);
                console.log("-".repeat(100));
                console.log(`Consistent Peaks (Value shown) | Inconsistent Peaks (*) | Background (.) | Packet ${packetcounter}`);
                console.log(samplesOutput);
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
    .description('Starts the Node.js serial console for the Open Echo Sonar.')
    .action(async (port) => {
        let selectedPort = port;

        if (!selectedPort) {
            const ports = await SerialPort.list();
            const availablePorts = ports.map(p => p.path);
            
            if (availablePorts.length === 0) {
                // Fallback to a common default port if nothing is found
                selectedPort = process.platform.startsWith('win') ? 'COM1' : '/dev/ttyACM0';
                console.warn(`\n⚠️ No serial ports found. Defaulting to '${selectedPort}'.`);
            } else {
                // Select the first available port (or handle user selection if needed)
                selectedPort = availablePorts[0];
                console.log(`Found available serial ports: ${availablePorts}. Using first: ${selectedPort}`);
            }
        }
        
        runSerialConsole(selectedPort);
    });

program.parse(process.argv);

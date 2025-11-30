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
const THRESHOLD = 60;                  // 50 Minimum value a point must have to be considered a peak
const CONSISTENCY_SAMPLES = 5;         // 3 Minimum number of samples a peak must appear in to be 'consistent'
const POSITION_TOLERANCE = 1;          // 5 Range (+/-) within which a peak position is considered the same

// Constants used for data conversion
const SPEED_OF_SOUND = 330;
const SAMPLE_TIME = 13.2e-6;
const SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2;
const MAX_DEPTH = NUM_SAMPLES * SAMPLE_RESOLUTION  // Total depth in cm;



let packetcounter = 0;

// --- Signal Tracker Class for Time-Series Analysis ---

class SignalTracker {
    /**
     * Manages a buffer of past sample arrays and identifies consistent signal indices.
     */
    constructor(bufferSize, threshold, tolerance) {
        this.buffer = [];
        this.bufferSize = bufferSize;
        this.threshold = threshold;
        this.tolerance = tolerance;
        this.consistentIndices = new Set();
    }

    updateAndGetConsistentIndices(currentValues) {
        // 1. Identify current 'peak' indices (above threshold)
        const currentPeakIndices = [];
        for (let i = 0; i < currentValues.length; i++) {
            if (currentValues[i] >= this.threshold) {
                currentPeakIndices.push(i);
            }
        }

        // 2. Update buffer (Store the indices, not the full values array)
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
            if (buffer[i] === PACKET_HEADER) {
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
        let sum = 0;
        for (let i = 0; i < NUM_SAMPLES; i++) {
            values.push(payload.readUInt16BE(6 + i * 2));
            sum += values[i];
            if (values.length == 400){
                //console.log("values",values);
                console.log(util.inspect(values, { maxArrayLength: null }))
                console.log("sum",sum);
                console.log("avg",sum/(i+1));
                //if((sum/i) > 100){
                    //process.exit();
                //}
            }
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
                if(packetcounter < 5){
                    console.log(`checking packet: ${packetcounter}`);
                    console.log(packet);
                    if(packetcounter == 4){
                        process.exit();
                    }
                }
                
                // 1. Get Consistent Indices
                const consistentIndices = tracker.updateAndGetConsistentIndices(values);

                // 2. Create Highlighted String
                const highlightedSamples = [];
                for (let i = 0; i < values.length; i++) {
                    const value = values[i];
                    const isConsistent = consistentIndices.has(i);

                    if (isConsistent && value >= THRESHOLD) {
                        // Highlight: Show value for consistent, high-value data
                        output = ` \x1b[31m${value}\x1b[0m `;    // red
                        //highlightedSamples.push(`${value.toString().padStart(4, ' ')}`);
                        highlightedSamples.push(`${output.toString().padStart(4, ' ')}`);
                    } else if (value >= THRESHOLD) {
                        // Show a marker for high-value but inconsistent data
                        
                        highlightedSamples.push("  * ");
                    } else {
                        // Show a dot for low-value data (below threshold)
                        highlightedSamples.push(" .  ");
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

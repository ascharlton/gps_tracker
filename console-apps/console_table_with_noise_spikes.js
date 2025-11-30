// Node.js implementation equivalent to the Python serial console script.

const { SerialPort } = require('serialport');
const { program } = require('commander');

// --- Configuration (MUST MATCH ARDUINO FIRMWARE) ---
const BAUD_RATE = 250000;
const NUM_SAMPLES = 1800; // CHANGED to 1800 samples as requested
const PAYLOAD_LEN = 6 + 2 * NUM_SAMPLES;
const PACKET_LEN = 1 + PAYLOAD_LEN + 1; // Header (1) + Payload + Checksum (1)
const PACKET_HEADER = 0xAA;

// --- TRACKING CONFIGURATION ---
const THRESHOLD = 72;                  // Minimum value a point must have to be considered a peak
const CONSISTENCY_SAMPLES = 10;         // Minimum number of samples a peak must appear in to be 'consistent'
const POSITION_TOLERANCE = 1;          // Range (+/-) within which a peak position is considered the same
const NOISE_FLOOR_RANGE = 800;         // Number of tail samples to use for noise floor calculation

// Constants used for data conversion
const SPEED_OF_SOUND = 330;
const SAMPLE_TIME = 13.2e-6;
const SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2;
const MAX_DEPTH = NUM_SAMPLES * SAMPLE_RESOLUTION  // Total depth in cm;

let packetcounter = 0;
const ANSI_RED = '\x1b[31m';
const ANSI_BLUE = '\x1b[34m';
const ANSI_ORANGE = '\x1b[33m'; // ANSI code for orange (usually yellow)
const ANSI_RESET = '\x1b[0m';

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

/**
 * Calculates the average noise floor from the last N samples (NOISE_FLOOR_RANGE).
 * @param {number[]} samples - The full array of sonar sample values (1800 values).
 * @returns {number} The calculated noise floor value (rounded to the nearest integer).
 */
function calculateNoiseFloor(samples) {
    const start = samples.length - NOISE_FLOOR_RANGE;
    if (start < 0) return 0; // Safety check
    
    // Use the last NOISE_FLOOR_RANGE samples
    const tailSamples = samples.slice(start);
    const sum = tailSamples.reduce((acc, val) => acc + val, 0);
    
    // Return the integer average
    return Math.round(sum / NOISE_FLOOR_RANGE);
}

// --- Noise Floor Averaging Class ---

class NoiseFloorAverager {
    /**
     * Manages a running average, minimum, and maximum of the calculated noise floor from all received frames.
     */
    constructor() {
        this.totalSum = 0;
        this.frameCount = 0;
        this.minNoiseFloor = Infinity;
        this.maxNoiseFloor = -Infinity;
    }

    /**
     * Updates the running sum, count, min, and max with a new noise floor value.
     * @param {number} noiseFloorValue - The noise floor calculated from the current frame.
     */
    update(noiseFloorValue) {
        this.totalSum += noiseFloorValue;
        this.frameCount += 1;
        // Update min/max
        this.minNoiseFloor = Math.min(this.minNoiseFloor, noiseFloorValue);
        this.maxNoiseFloor = Math.max(this.maxNoiseFloor, noiseFloorValue);
    }

    /**
     * Calculates the running average.
     * @returns {number} The running average noise floor, rounded to 1 decimal place.
     */
    getRunningAverage() {
        if (this.frameCount === 0) return 0;
        return (this.totalSum / this.frameCount).toFixed(1);
    }
    
    /**
     * Returns the minimum and maximum noise floor values recorded.
     * @returns {object} An object containing min and max values.
     */
    getMinMax() {
        // Return 0 if no data has been collected yet
        if (this.frameCount === 0) {
            return { min: 0, max: 0 };
        }
        // Return the min/max as integers since the noise floor calculation is rounded
        return {
            min: Math.round(this.minNoiseFloor),
            max: Math.round(this.maxNoiseFloor),
        };
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

        // 3. Checksum OK: Unpack Payload
        const depthIndex = payload.readUInt16BE(0);
        const tempScaled = payload.readInt16BE(2); // Signed short (h)
        const vDrvScaled = payload.readUInt16BE(4);

        // Samples start at offset 6. Unpack as an array of 16-bit unsigned integers (H)
        const values = [];
        for (let i = 0; i < NUM_SAMPLES; i++) {
            values.push(payload.readUInt16BE(6 + i * 2));
        }

        // 4. Scale and Calculate Real Values
        const depthCm = parseFloat((depthIndex * SAMPLE_RESOLUTION).toFixed(1));
        const temperature = parseFloat((tempScaled / 100.0).toFixed(1));
        const driveVoltage = parseFloat((vDrvScaled / 100.0).toFixed(1));

        // 5. Cleanup buffer and return
        buffer = buffer.subarray(headerIndex + PACKET_LEN);
        packet = { values, depthCm, temperature, driveVoltage };

    } catch (e) {
        console.error("Critical error during packet processing:", e);
    }

    isReading = false;
    return packet;
}

// --- Main Execution Loop ---

async function runSerialConsole(portName) {
    console.log(`Attempting to connect to: ${portName} @ ${BAUD_RATE} baud...`);
    console.log(`Tracking: Threshold=${THRESHOLD}, Consistency=${CONSISTENCY_SAMPLES}, Tolerance=${POSITION_TOLERANCE}, NoiseFloorRange=${NOISE_FLOOR_RANGE}`);

    // Initialize the Signal Tracker and Averager
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
                
                const { values, depthCm, temperature, driveVoltage } = packet;

                packetcounter++;

                // 1. Calculate and Track Noise Floor
                const noiseFloor = calculateNoiseFloor(values);
                noiseAverager.update(noiseFloor);
                const runningNoiseAvg = noiseAverager.getRunningAverage();
                const { min, max } = noiseAverager.getMinMax(); // Get min/max values

                // 2. Get Consistent Indices
                const consistentIndices = tracker.updateAndGetConsistentIndices(values);

                // --- TABLE GENERATION LOGIC ---
                const NUM_COLUMNS = 50;
                const NUM_ROWS = values.length / NUM_COLUMNS; 
                const CELL_WIDTH = 3; 
                const SPACE_SEPARATOR = ' '; 
                const ROW_LABEL_WIDTH = 3; 

                // Array to hold all 1800 processed (padded/colored/spaced) strings
                const processedValues = [];

                // Process all 1800 values for padding and color
                for (let i = 0; i < values.length; i++) {
                    const value = values[i];
                    const isConsistent = consistentIndices.has(i);
                    let outputString = '';
                    
                    // Priority 1: High-value, Consistent Peak (Red)
                    if (isConsistent && value >= THRESHOLD) {
                         // Check for the distance range (arbitrary example, adjust as needed)
                        if (i > 700) { 
                            const paddedValue = value.toString().padStart(CELL_WIDTH, ' ');
                            outputString = `${ANSI_RED}${paddedValue}${ANSI_RESET}`;
                        } else {
                            outputString = value.toString().padStart(CELL_WIDTH, ' ');
                        }
                    } 
                    // Priority 2: Sub-Threshold, Above Noise Floor average (Orange)
                    // Note: Use the current frame's noise floor average for real-time comparison
                    else if (value < THRESHOLD && value > noiseFloor) { 
                        const paddedValue = value.toString().padStart(CELL_WIDTH, ' ');
                        outputString = `${ANSI_ORANGE}${paddedValue}${ANSI_RESET}`;
                    }
                    // Priority 3: Below Noise Floor or Inconsistent Peak
                    else {
                        // Inconsistent peak (high value) or background noise (low value or below noise floor)
                        //const paddedValue = value.toString().padStart(CELL_WIDTH, ' ');
                        //outputString = `${ANSI_BLUE}${paddedValue}${ANSI_RESET}`;
                        outputString = ' '.repeat(CELL_WIDTH);
                    }
                    
                    // Add the space separator AFTER the 3-char padded number/placeholder
                    processedValues.push(outputString + SPACE_SEPARATOR);
                }

                // ----------------------------------------------------
                // Build the Console Table

                // 1. Build Header Row (Labels 1 to 50)
                let header = ' '.repeat(ROW_LABEL_WIDTH) + ' | '; // Space for row label | space

                // Column labels 1 to 50
                for (let c = 1; c <= NUM_COLUMNS; c++) {
                    // Pad the column number to match the cell width (CELL_WIDTH + SPACE_SEPARATOR = 4)
                    const labelPadding = CELL_WIDTH + 1; 
                    header += c.toString().padStart(labelPadding, ' ');
                }
                header = header.trimEnd(); // Remove final trailing space

                // Determine the total table width for separators
                const tableWidth = header.length;

                // 2. Build Separator Line
                const separator = '-'.repeat(ROW_LABEL_WIDTH) + '-|-' + '-'.repeat(tableWidth - ROW_LABEL_WIDTH - 2); 

                // 3. Build Data Rows (1 to 36)
                const tableRows = [];
                for (let r = 0; r < NUM_ROWS; r++) {
                    const rowNumber = r + 1; // 1-indexed row number
                    
                    // Start the row string with the padded row number
                    let rowString = rowNumber.toString().padStart(ROW_LABEL_WIDTH, ' ') + ' | ';
                    
                    // Extract the 50 processed values for this row
                    const start = r * NUM_COLUMNS;
                    const end = (r + 1) * NUM_COLUMNS;
                    const rowData = processedValues.slice(start, end).join('');
                    
                    rowString += rowData;
                    tableRows.push(rowString.trimEnd()); // Trim any final trailing space
                }


                // --- CONSOLE OUTPUT (Revised) ---
                const now = new Date();
                const timeString = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}`;
                
                // Print metadata header
                console.log("\n" + "=".repeat(tableWidth));
                // UPDATED: Added Min and Max noise floor values
                console.log(`Time: ${timeString} | Depth: ${depthCm} cm | Temp: ${temperature} °C | Signal Threshold: ${THRESHOLD}  | Noise Floor: ${noiseFloor} (Avg: ${runningNoiseAvg}, Min: ${min}, Max: ${max})`);
                console.log("-".repeat(tableWidth));
                console.log(`Not Consistent ( ) | Consistent Peaks (\x1b[31mRed\x1b[0m) | Sub-Threshold Signals (\x1b[33mOrange\x1b[0m) | Packet ${packetcounter}`);

                // Print the table structure
                console.log(header);
                console.log(separator);
                tableRows.forEach(row => console.log(row));
                console.log("=".repeat(tableWidth));
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
                selectedPort = process.platform.startsWith('win') ? 'COM1' : '/dev/ttySONAR';
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

// Node.js implementation equivalent to the Python serial console script.

const { SerialPort } = require('serialport');
const { program } = require('commander');

// --- Configuration (MUST MATCH ARDUINO FIRMWARE) ---
const BAUD_RATE = 250000;
const NUM_SAMPLES = 1800; // Total number of samples in the frame
const PAYLOAD_LEN = 6 + 2 * NUM_SAMPLES;
const PACKET_LEN = 1 + PAYLOAD_LEN + 1; // Header (1) + Payload + Checksum (1)
const PACKET_HEADER = 0xAA;

// --- TRACKING CONFIGURATION ---
const THRESHOLD = 42;                  // Minimum value a point must have to be considered a peak
const CONSISTENCY_SAMPLES = 1;         // Minimum number of samples a peak must appear in to be 'consistent'
const POSITION_TOLERANCE = 1;          // Range (+/-) within which a peak position is considered the same
const NOISE_FLOOR_RANGE = 200;         // Number of tail samples to use for noise floor calculation

// FIX: Resetting to a sensible value (10) to enforce separation and prevent closely related peaks 
// (like Signal 2 and 3 in your output) from being counted separately.
const SIGNAL_SEPARATION_THRESHOLD = 10; 

const MAX_BLIND_ZONE_SEARCH_SAMPLES = 150; 

// Constants used for data conversion
const SPEED_OF_SOUND = 330;
const SAMPLE_TIME = 13.2e-6;
const SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2;
const MAX_DEPTH = NUM_SAMPLES * SAMPLE_RESOLUTION;
let packetcounter = 0;

// --- ANSI Escape Codes for Console Formatting ---
const ANSI_RESET = '\x1b[0m';
const ANSI_RED = '\x1b[31m';
const ANSI_YELLOW = '\x1b[33m';
const ANSI_BLUE = '\x1b[34m';
const ANSI_MAGENTA = '\x1b[35m';


// --- CLASSES ---

/**
 * Calculates a dynamic blind zone end index based on noise floor activity.
 * @param {Array<number>} values 
 * @param {number} noiseFloor 
 * @returns {number} The index where the blind zone (ring-down) is considered finished.
 */
function findBlindZoneEnd(values, noiseFloor) {
    // --- 1. Noise Floor Criteria (Original Logic) ---
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

/**
 * Calculates the noise floor and the amplitude threshold used for noise filtering.
 * @param {Array<number>} values 
 * @returns {{noiseFloor: number, noiseFilterAmplitude: number}}
 */
function calculateNoiseFloor(values) {
    // Use the last NOISE_FLOOR_RANGE samples
    const tailSamples = values.slice(NUM_SAMPLES - NOISE_FLOOR_RANGE);
    
    // Sort and use the 5th percentile to get a robust noise estimate
    const sortedTail = tailSamples.sort((a, b) => a - b);
    const index = Math.floor(sortedTail.length * 0.05);
    const noiseFloor = sortedTail[index] || 0;

    // Use a factor (e.g., 2.5) times the noise floor as the actual noise filter amplitude
    const NOISE_FACTOR = 2.5; 
    const noiseFilterAmplitude = noiseFloor * NOISE_FACTOR;

    return { noiseFloor, noiseFilterAmplitude };
}

/**
 * Tracks the consistency of signal positions over time.
 */
class SignalTracker {
    constructor(consistencySamples, threshold, tolerance) {
        this.consistencySamples = consistencySamples;
        this.threshold = threshold;
        this.tolerance = tolerance;
        // Map stores: position -> { count: number, history: number[] }
        this.history = new Map();
        this.frameCounter = 0;
    }

    /**
     * Updates the history with peaks from the current frame and identifies consistent indices.
     * @param {Array<number>} values 
     * @returns {Set<number>} A set of indices that have been consistent for enough frames.
     */
    updateAndGetConsistentIndices(values) {
        this.frameCounter++;
        const currentPeaks = new Set();
        const nextHistory = new Map();
        
        // Find peaks in the current frame (simple threshold crossing)
        for (let i = 0; i < values.length; i++) {
            if (values[i] >= this.threshold) {
                currentPeaks.add(i);
            }
        }

        const consistentIndices = new Set();
        
        // Update counts based on overlap with previous history
        for (const [pos, data] of this.history) {
            let foundMatch = false;
            
            // Check for peaks within the tolerance range
            for (let i = pos - this.tolerance; i <= pos + this.tolerance; i++) {
                if (currentPeaks.has(i)) {
                    // Match found, update count and use the current peak's position (i) for the new history
                    const newCount = data.count + 1;
                    const newPos = i;
                    
                    if (newCount >= this.consistencySamples) {
                        consistentIndices.add(newPos);
                    }
                    
                    // Update next history map
                    if (!nextHistory.has(newPos)) {
                        nextHistory.set(newPos, { count: newCount });
                    }
                    foundMatch = true;
                    currentPeaks.delete(i); // Remove matched peak to avoid double-counting
                    break;
                }
            }

            // If no match was found, the count resets (or decays, but we stick to reset for simplicity)
            if (!foundMatch && data.count > 0) {
                // Decay not implemented, simply dropped if not found
            }
        }

        // Add any remaining new peaks (those not matched by history) to the nextHistory with count 1
        for (const newPos of currentPeaks) {
            if (!nextHistory.has(newPos)) {
                nextHistory.set(newPos, { count: 1 });
            }
        }

        this.history = nextHistory;
        return consistentIndices;
    }
}


/**
 * Maintains a running average, min, max, and standard deviation of the noise floor.
 */
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


// --- SERIAL PACKET HANDLING ---
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
            // console.error(`\n⚠️ Checksum mismatch! Calculated: ${calculatedChecksum.toString(16).toUpperCase()}, Received: ${receivedChecksum.toString(16).toUpperCase()}`);
            buffer = buffer.subarray(headerIndex + 1);
            isReading = false;
            return extractAndProcessPacket(); // Try again from remaining buffer
        }

        // 3. Checksum OK: Unpack Payload

        // Samples start at offset 6. Unpack as an array of 16-bit unsigned integers (H)
        const values = [];
        for (let i = 0; i < NUM_SAMPLES; i++) {
            values.push(payload.readUInt16BE(6 + i * 2));
        }

        // 4. Validate Blind Zone Amplitude (NEW LOGIC)
        const blindZoneSamples = values.slice(0, 10);
        const blindZoneSum = blindZoneSamples.reduce((sum, val) => sum + val, 0);
        const blindZoneAverage = blindZoneSum / blindZoneSamples.length;
        
        const MIN_BLIND_ZONE_AVG = 100; // The threshold you specified

        if (blindZoneAverage <= MIN_BLIND_ZONE_AVG) {
            // Data corruption suspected: First 10 values do not meet the expected minimum amplitude.
            console.error(`\n⚠️ Packet corrupted: Blind zone average (${blindZoneAverage.toFixed(1)}) is <= ${MIN_BLIND_ZONE_AVG}. Discarding packet.`);
            
            // Discard the bad header (only 1 byte) and continue search for the next packet
            buffer = buffer.subarray(headerIndex + 1);
            isReading = false;
            return extractAndProcessPacket(); // Try again from remaining buffer
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
    // Initialize required objects
    const tracker = new SignalTracker(CONSISTENCY_SAMPLES, THRESHOLD, POSITION_TOLERANCE);
    const noiseAverager = new NoiseFloorAverager();
    let port;

    // Table width calculation
    const CELL_WIDTH = 3;
    const SPACE_SEPARATOR = ' ';
    const ROW_LABEL_WIDTH = 3; 
    const NUM_COLUMNS = 50;
    
    // NOTE: This recalculates the width to ensure the separator line matches the header line length
    const totalCellWidth = (CELL_WIDTH + SPACE_SEPARATOR.length) * NUM_COLUMNS;
    const tableWidth = ROW_LABEL_WIDTH + 1 + 1 + totalCellWidth; // Row Label + Space + Pipe + Data

    try {
        port = new SerialPort({ path: portName, baudRate: BAUD_RATE });

        port.on('open', () => {
            console.log(`\n✅ Serial port opened on ${portName} at ${BAUD_RATE}.`);
            console.log(`   Config: THRESHOLD=${THRESHOLD}, CONSISTENCY_SAMPLES=${CONSISTENCY_SAMPLES}, TOLERANCE=${POSITION_TOLERANCE}`);
        });

        port.on('data', data => {
            // Append incoming data to the buffer
            buffer = Buffer.concat([buffer, data]);

            // --- DEBUGGING LINE ---
            //console.log(`[DEBUG] New data received. Buffer length: ${buffer.length} bytes.`);
            //console.log('Buffer Content (Hex):', data.toString('hex').toUpperCase()); 
            // ------------------------
            
            // if (data.length >= 20) {
            //     const byte1 = data[0];
            //     const byte2 = data[1];
            //     const byte3 = data[2];
            //     const byte4 = data[3];
            //     const byte5 = data[4];
            //     const byte6 = data[5];
            //     const byte7 = data[6];
            //     const byte8 = data[7];
            //     const byte9 = data[8];
            //     const byte10 = data[9];
            //     const byte11 = data[10];
            //     const byte12 = data[11];
            //     const byte13 = data[12];
            //     const byte14 = data[13];
            //     const decimalValue1 = data.readUInt16BE(0);
            //     const decimalValue2 = data.readUInt16BE(2);
            //     const decimalValue3 = data.readUInt16BE(4);
            //     const decimalValue4 = data.readUInt16BE(6);
            //     const decimalValue5 = data.readUInt16BE(8);
            //     const decimalValue6 = data.readUInt16BE(10);
            //     const decimalValue7 = data.readUInt16BE(12);


            //     console.log(`\n--- Two-Byte Analysis ---`);
            //     console.log(`Byte 1 (Hex): 0x${byte1.toString(16).toUpperCase()}`);
            //     console.log(`Byte 2 (Hex): 0x${byte2.toString(16).toUpperCase()}`);
            //     console.log(`Combined 16-bit Decimal Value: ${decimalValue1}`);
            //     console.log(`Byte 3 (Hex): 0x${byte3.toString(16).toUpperCase()}`);
            //     console.log(`Byte 4 (Hex): 0x${byte4.toString(16).toUpperCase()}`);
            //     console.log(`Combined 16-bit Decimal Value: ${decimalValue2}`);
            //     console.log(`Byte 5 (Hex): 0x${byte5.toString(16).toUpperCase()}`);
            //     console.log(`Byte 6 (Hex): 0x${byte6.toString(16).toUpperCase()}`);
            //     console.log(`Combined 16-bit Decimal Value: ${decimalValue3}`);
            //     console.log(`Byte 7 (Hex): 0x${byte7.toString(16).toUpperCase()}`);
            //     console.log(`Byte 8 (Hex): 0x${byte8.toString(16).toUpperCase()}`);
            //     console.log(`Combined 16-bit Decimal Value: ${decimalValue4}`);
            //     console.log(`Byte 9 (Hex): 0x${byte9.toString(16).toUpperCase()}`);
            //     console.log(`Byte 10 (Hex): 0x${byte10.toString(16).toUpperCase()}`);
            //     console.log(`Combined 16-bit Decimal Value: ${decimalValue5}`);
            //     console.log(`Byte 11 (Hex): 0x${byte11.toString(16).toUpperCase()}`);
            //     console.log(`Byte 12 (Hex): 0x${byte12.toString(16).toUpperCase()}`);
            //     console.log(`Combined 16-bit Decimal Value: ${decimalValue6}`);
            //     console.log(`Byte 13 (Hex): 0x${byte13.toString(16).toUpperCase()}`);
            //     console.log(`Byte 14 (Hex): 0x${byte14.toString(16).toUpperCase()}`);
            //     console.log(`Combined 16-bit Decimal Value: ${decimalValue7}`);


            //     console.log(`-------------------------\n`);
            // }

            // Process the buffer as long as complete packets are found
            let packet;
            while ((packet = extractAndProcessPacket())) {
                
                //const { values, depthCm, temperature, driveVoltage } = packet;
                const { values } = packet;

                // probably the first packet recived contains rubbish data from misalignment 
                packetcounter++;
                if(packetcounter == 1 ){
                    console.log(`checking packet: ${packetcounter}`);
                    console.log(packet);
                    //process.exit();
                }

                // 1. Calculate and Track Noise Floor
                const { noiseFloor } = calculateNoiseFloor(values);
                noiseAverager.update(noiseFloor);
                const runningNoiseAvg = noiseAverager.getRunningAverage();
                const { min, max } = noiseAverager.getMinMax(); 
                const runningStdDev = noiseAverager.getStandardDeviation(); 
                const runningNoiseAvgValue = parseFloat(runningNoiseAvg);
                const runningStdDevValue = parseFloat(runningStdDev);
                const noiseFilterAmplitude = parseFloat((runningNoiseAvgValue + runningStdDevValue).toFixed(1));
                const blindZoneIndexEnd = findBlindZoneEnd(values, runningNoiseAvgValue); 
                const consistentIndices = tracker.updateAndGetConsistentIndices(values);

                // --- TABLE GENERATION LOGIC ---
                const NUM_ROWS = values.length / NUM_COLUMNS; 
                const processedValues = [];
                let firstConsistentSignalIndex = -1; 

                for (let i = 0; i < values.length; i++) {
                    const value = values[i];
                    let outputString = '';
                    
                    if (i < blindZoneIndexEnd) {
                        outputString = ' '.repeat(CELL_WIDTH);
                    } else {
                        const isConsistent = consistentIndices.has(i);

                        if (isConsistent && value >= THRESHOLD) {
                            if (firstConsistentSignalIndex === -1) {
                                firstConsistentSignalIndex = i;
                            }
                            
                            if (i > 600) { 
                                const paddedValue = value.toString().padStart(CELL_WIDTH, ' ');
                                outputString = `${ANSI_RED}${paddedValue}${ANSI_RESET}`;
                            } else {
                                outputString = value.toString().padStart(CELL_WIDTH, ' ');
                            }
                        } 
                        else if (value < THRESHOLD && value > noiseFilterAmplitude) { 
                            outputString = ' '.repeat(CELL_WIDTH);
                        }
                        else {
                            outputString = ' '.repeat(CELL_WIDTH);
                        }
                    }
                    processedValues.push(outputString + SPACE_SEPARATOR);
                } // End of FOR loop
                

                // =========================================================================
                // --- PULSE WIDTH AND MULTI-SIGNAL TRACKING (AFTER Loop) ---
                // =========================================================================
                let firstSignalPulseWidth = 0;
                let firstSignalPulseEnd = -1;
                let secondConsistentSignalIndex = -1; 
                let secondSignalPulseWidth = 0;
                let secondSignalPulseEnd = -1;
                let thirdConsistentSignalIndex = -1; 
                let thirdSignalPulseWidth = 0;
                let thirdSignalPulseEnd = -1;
                let fourthConsistentSignalIndex = -1; 
                let fourthSignalPulseWidth = 0;
                let fourthSignalPulseEnd = -1;
                let fifthConsistentSignalIndex = -1; 
                let fifthSignalPulseWidth = 0;
                let fifthSignalPulseEnd = -1;
                let sixthConsistentSignalIndex = -1; 
                let sixthSignalPulseWidth = 0;
                let sixthSignalPulseEnd = -1;
                let seventhConsistentSignalIndex = -1; 
                let seventhSignalPulseWidth = 0;
                let seventhSignalPulseEnd = -1;


                if (firstConsistentSignalIndex !== -1) {
                    let currentWidth = 0;
                    
                    // 1. Calculate Pulse Width and End Index of the First Signal
                    for (let i = firstConsistentSignalIndex; i < values.length; i++) {
                        if (values[i] >= THRESHOLD) {
                            currentWidth++;
                        } else {
                            firstSignalPulseEnd = i; 
                            break;
                        }
                    }
                    firstSignalPulseWidth = currentWidth;
                    if (firstSignalPulseEnd === -1 && currentWidth > 0) {
                        firstSignalPulseEnd = values.length - 1; 
                    }

                    // 2. Find the start of the Second Consistent Signal
                    const searchStart2 = firstSignalPulseEnd + SIGNAL_SEPARATION_THRESHOLD;
                    
                    if (firstSignalPulseEnd !== -1) {
                        for (let i = searchStart2; i < values.length; i++) {
                            if (consistentIndices.has(i) && values[i] >= THRESHOLD) {
                                secondConsistentSignalIndex = i;
                                break; 
                            }
                        }
                    }

                    // 3. Calculate Pulse Width and End Index of the Second Signal
                    if (secondConsistentSignalIndex !== -1) {
                        let currentWidth2 = 0;
                        for (let i = secondConsistentSignalIndex; i < values.length; i++) {
                            if (values[i] >= THRESHOLD) {
                                currentWidth2++;
                            } else {
                                secondSignalPulseEnd = i; 
                                break;
                            }
                        }
                        secondSignalPulseWidth = currentWidth2;
                        
                        if (secondSignalPulseEnd === -1 && currentWidth2 > 0) {
                            secondSignalPulseEnd = values.length - 1; 
                        }

                        // 4. Find the start of the Third Consistent Signal
                        const searchStart3 = secondSignalPulseEnd + SIGNAL_SEPARATION_THRESHOLD;
                        
                        if (secondSignalPulseEnd !== -1) {
                            for (let i = searchStart3; i < values.length; i++) {
                                if (consistentIndices.has(i) && values[i] >= THRESHOLD) {
                                    thirdConsistentSignalIndex = i;
                                    break; 
                                }
                            }
                        }
                        
                        // 5. Calculate Pulse Width and End Index of the Third Signal
                        if (thirdConsistentSignalIndex !== -1) {
                            let currentWidth3 = 0;
                            for (let i = thirdConsistentSignalIndex; i < values.length; i++) {
                                if (values[i] >= THRESHOLD) {
                                    currentWidth3++;
                                } else {
                                    thirdSignalPulseEnd = i; 
                                    break;
                                }
                            }
                            thirdSignalPulseWidth = currentWidth3;
                            
                            if (thirdSignalPulseEnd === -1 && currentWidth3 > 0) {
                                thirdSignalPulseEnd = values.length - 1; 
                            }

                            // 6. Find the start of the Fourth Consistent Signal
                            const searchStart4 = thirdSignalPulseEnd + SIGNAL_SEPARATION_THRESHOLD;
                            
                            if (thirdSignalPulseEnd !== -1) {
                                for (let i = searchStart4; i < values.length; i++) {
                                    if (consistentIndices.has(i) && values[i] >= THRESHOLD) {
                                        fourthConsistentSignalIndex = i;
                                        break; 
                                    }
                                }
                            }
                            
                            // 7. Calculate Pulse Width and End Index of the Fourth Signal
                            if (fourthConsistentSignalIndex !== -1) {
                                let currentWidth4 = 0;
                                for (let i = fourthConsistentSignalIndex; i < values.length; i++) {
                                    if (values[i] >= THRESHOLD) {
                                        currentWidth4++;
                                    } else {
                                        fourthSignalPulseEnd = i; 
                                        break;
                                    }
                                }
                                fourthSignalPulseWidth = currentWidth4;
                                
                                if (fourthSignalPulseEnd === -1 && currentWidth4 > 0) {
                                    fourthSignalPulseEnd = values.length - 1; 
                                }

                                // 8. Find the start of the Fifth Consistent Signal
                                const searchStart5 = fourthSignalPulseEnd + SIGNAL_SEPARATION_THRESHOLD;
                                
                                if (fourthSignalPulseEnd !== -1) {
                                    for (let i = searchStart5; i < values.length; i++) {
                                        if (consistentIndices.has(i) && values[i] >= THRESHOLD) {
                                            fifthConsistentSignalIndex = i;
                                            break; 
                                        }
                                    }
                                }
                                
                                // 9. Calculate Pulse Width and End Index of the Fifth Signal
                                if (fifthConsistentSignalIndex !== -1) {
                                    let currentWidth5 = 0;
                                    for (let i = fifthConsistentSignalIndex; i < values.length; i++) {
                                        if (values[i] >= THRESHOLD) {
                                            currentWidth5++;
                                        } else {
                                            fifthSignalPulseEnd = i; 
                                            break;
                                        }
                                    }
                                    fifthSignalPulseWidth = currentWidth5;
                                    
                                    if (fifthSignalPulseEnd === -1 && currentWidth5 > 0) {
                                        fifthSignalPulseEnd = values.length - 1; 
                                    }

                                    // 10. Find the start of the Sixth Consistent Signal
                                    const searchStart6 = fifthSignalPulseEnd + SIGNAL_SEPARATION_THRESHOLD;
                                    
                                    if (fifthSignalPulseEnd !== -1) {
                                        for (let i = searchStart6; i < values.length; i++) {
                                            if (consistentIndices.has(i) && values[i] >= THRESHOLD) {
                                                sixthConsistentSignalIndex = i;
                                                break; 
                                            }
                                        }
                                    }

                                    // 11. Calculate Pulse Width and End Index of the Sixth Signal
                                    if (sixthConsistentSignalIndex !== -1) {
                                        let currentWidth6 = 0;
                                        for (let i = sixthConsistentSignalIndex; i < values.length; i++) {
                                            if (values[i] >= THRESHOLD) {
                                                currentWidth6++;
                                            } else {
                                                sixthSignalPulseEnd = i; 
                                                break;
                                            }
                                        }
                                        sixthSignalPulseWidth = currentWidth6;
                                        
                                        if (sixthSignalPulseEnd === -1 && currentWidth6 > 0) {
                                            sixthSignalPulseEnd = values.length - 1; 
                                        }

                                        // 12. Find the start of the Seventh Consistent Signal
                                        const searchStart7 = sixthSignalPulseEnd + SIGNAL_SEPARATION_THRESHOLD;
                                        
                                        if (sixthSignalPulseEnd !== -1) {
                                            for (let i = searchStart7; i < values.length; i++) {
                                                if (consistentIndices.has(i) && values[i] >= THRESHOLD) {
                                                    seventhConsistentSignalIndex = i;
                                                    break; 
                                                }
                                            }
                                        }
                                        
                                        // 13. Calculate Pulse Width and End Index of the Seventh Signal
                                        if (seventhConsistentSignalIndex !== -1) {
                                            let currentWidth7 = 0;
                                            for (let i = seventhConsistentSignalIndex; i < values.length; i++) {
                                                if (values[i] >= THRESHOLD) {
                                                    currentWidth7++;
                                                } else {
                                                    seventhSignalPulseEnd = i; 
                                                    break;
                                                }
                                            }
                                            seventhSignalPulseWidth = currentWidth7;
                                            
                                            if (seventhSignalPulseEnd === -1 && currentWidth7 > 0) {
                                                seventhSignalPulseEnd = values.length - 1; 
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                // ----------------------------------------------------
                // Build the Console Table

                // 1. Build Header Row (Labels 1 to 50)
                let header = ' '.repeat(ROW_LABEL_WIDTH) + '|'; // Space for row label | space

                for (let c = 1; c <= NUM_COLUMNS; c++) {
                    const labelPadding = CELL_WIDTH + 1; 
                    header += c.toString().padStart(labelPadding, ' ');
                }
                header = header.trimEnd(); 

                const separator = '-'.repeat(tableWidth); 

                // 2. Build Data Rows (1 to 36)
                const tableRows = [];
                for (let r = 0; r < NUM_ROWS; r++) {
                    const rowNumber = r + 1; 
                    let rowString = rowNumber.toString().padStart(ROW_LABEL_WIDTH, ' ') + ' | ';
                    
                    const start = r * NUM_COLUMNS;
                    const end = (r + 1) * NUM_COLUMNS;
                    const rowData = processedValues.slice(start, end).join('');
                    
                    rowString += rowData;
                    tableRows.push(rowString.trimEnd()); 
                }


                // --- CONSOLE OUTPUT ---
                // clear the screen
                process.stdout.write('\x1Bc'); 

                const now = new Date();
                const timeString = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}`;
                
                // Print metadata header
                console.log("\n" + "=".repeat(tableWidth));
                // Metrics Line
                console.log(`Time: ${timeString} |  Noise Floor: ${noiseFloor} (Avg: ${runningNoiseAvg}, Min: ${min}, Max: ${max}, Std Dev: ${runningStdDev})`);
                console.log("-".repeat(tableWidth));
                
                // Status for all 7 signals (REVISED MULTI-LINE REPORTING)
                let firstSignalStatus = firstConsistentSignalIndex === -1 ? 
                    `${ANSI_MAGENTA}NO SIG 1${ANSI_RESET}` : 
                    `${ANSI_YELLOW}Index 1: ${firstConsistentSignalIndex} (End: ${firstSignalPulseEnd}) (Width: ${firstSignalPulseWidth})${ANSI_RESET}`; 

                let secondSignalStatus = secondConsistentSignalIndex === -1 ? 
                    `${ANSI_MAGENTA}NO SIG 2${ANSI_RESET}` : 
                    `${ANSI_YELLOW}Index 2: ${secondConsistentSignalIndex} (End: ${secondSignalPulseEnd}) (Width: ${secondSignalPulseWidth})${ANSI_RESET}`;

                let thirdSignalStatus = thirdConsistentSignalIndex === -1 ? 
                    `${ANSI_MAGENTA}NO SIG 3${ANSI_RESET}` : 
                    `${ANSI_YELLOW}Index 3: ${thirdConsistentSignalIndex} (End: ${thirdSignalPulseEnd}) (Width: ${thirdSignalPulseWidth})${ANSI_RESET}`;

                let fourthSignalStatus = fourthConsistentSignalIndex === -1 ? 
                    `${ANSI_MAGENTA}NO SIG 4${ANSI_RESET}` : 
                    `${ANSI_YELLOW}Index 4: ${fourthConsistentSignalIndex} (End: ${fourthSignalPulseEnd}) (Width: ${fourthSignalPulseWidth})${ANSI_RESET}`;

                let fifthSignalStatus = fifthConsistentSignalIndex === -1 ? 
                    `${ANSI_MAGENTA}NO SIG 5${ANSI_RESET}` : 
                    `${ANSI_YELLOW}Index 5: ${fifthConsistentSignalIndex} (End: ${fifthSignalPulseEnd}) (Width: ${fifthSignalPulseWidth})${ANSI_RESET}`;

                let sixthSignalStatus = sixthConsistentSignalIndex === -1 ? 
                    `${ANSI_MAGENTA}NO SIG 6${ANSI_RESET}` : 
                    `${ANSI_YELLOW}Index 6: ${sixthConsistentSignalIndex} (End: ${sixthSignalPulseEnd}) (Width: ${sixthSignalPulseWidth})${ANSI_RESET}`;

                let seventhSignalStatus = seventhConsistentSignalIndex === -1 ? 
                    `${ANSI_MAGENTA}NO SIG 7${ANSI_RESET}` : 
                    `${ANSI_YELLOW}Index 7: ${seventhConsistentSignalIndex} (End: ${seventhSignalPulseEnd}) (Width: ${seventhSignalPulseWidth})${ANSI_RESET}`;

                console.log(`Blind Zone Filter Amplitude: ${noiseFilterAmplitude.toFixed(2)} | Suppression End Index (Dynamic): ${blindZoneIndexEnd} | Amplitude Threshold: ${THRESHOLD}`);
                console.log(`--- SIGNAL METRICS (Separation Threshold: ${SIGNAL_SEPARATION_THRESHOLD}) ---`);
                console.log(firstSignalStatus);
                console.log(secondSignalStatus);
                console.log(thirdSignalStatus);
                console.log(fourthSignalStatus);
                console.log(fifthSignalStatus);
                console.log(sixthSignalStatus);
                console.log(seventhSignalStatus);
                console.log(`Consistent Peaks (${ANSI_RED}Red${ANSI_RESET}) | Noise Filter Active | Packet ${packetcounter}`);

                // Print the table structure
                console.log("=".repeat(tableWidth));
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
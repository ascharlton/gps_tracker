
const { spawn } = require('child_process');
// NOTE: You must install 'serialport' and import it here when running locally:
const { SerialPort } = require('serialport'); 

// --- GLOBAL STATE ---
let lastSmoothedDistance = null; 
let comBuffer = Buffer.alloc(0); // Global running buffer for fragmented serial packets


// --- SERIAL PORT CONFIGURATION ---
// !!! IMPORTANT: Update these constants for your physical setup !!!
const COM_PORT_PATH = '/dev/ttyACM0'; // e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux
const BAUD_RATE = 250000;
// --- END SERIAL PORT CONFIG ---

// Physical Constants
// **NEW CALIBRATION** using physical constants provided by user's script: 
// (Speed of Sound * Sample Time * 100 cm/m) / 2 = (330 * 13.2e-6 * 100) / 2 = 0.2178 cm/sample
const SAMPLE_RESOLUTION = 0.2178; // 2.1mm This is the smallest distance change that can be measured or resolved
const ORIGINAL_SAMPLE_COUNT = 1800;

// SERIAL PACKET CONSTANTS (Based on user's script logic)
const HEADER_BYTE = 0xAA;
const NUM_METADATA_BYTES = 6;
const NUM_SAMPLES = ORIGINAL_SAMPLE_COUNT; // 1800
const SAMPLES_BYTE_SIZE = NUM_SAMPLES * 2; // 3600 bytes
// 1 (Header) + 6 (Metadata) + 3600 (Samples) + 1 (Checksum) = 3608 bytes
const PACKET_SIZE = 1 + NUM_METADATA_BYTES + SAMPLES_BYTE_SIZE + 1; 

// Defines the range (from the end of the 1800 samples) used to calculate average noise floor
const NOISE_FLOOR_RANGE = 100; 

// FILTERING CONSTANT
// EMA_ALPHA controls the smoothing factor: 
const EMA_ALPHA = 0.1;

// --- VERBOSE SETTING ---
// Set to true to print the entire raw sample array for each frame.
const PRINT_RAW_SAMPLES = true;

/**
 * Handles incoming data chunks, accumulates them, and extracts complete, valid packets.
 * This logic mirrors the user's provided script for handling binary packets.
 * @param {Buffer} data - A chunk of binary data received from the port.
 */
function serialBufferHandler(data) {
    // 1. Concatenate the new data chunk to the accumulated buffer
    comBuffer = Buffer.concat([comBuffer, data]);

    while (comBuffer.length >= PACKET_SIZE) {
        
        const headerIndex = comBuffer.indexOf(HEADER_BYTE);
        
        if (headerIndex === -1) {
            // No header found in buffer, discard it all
            comBuffer = Buffer.alloc(0);
            console.warn(`[COM WARNING] Dropping entire buffer. No header (${HEADER_BYTE.toString(16)}) found.`);
            break;
        }

        if (headerIndex > 0) {
            console.warn(`[COM WARNING] Discarding ${headerIndex} junk bytes before header.`);
            // Slice off junk bytes up to the header
            comBuffer = comBuffer.slice(headerIndex);
            // After slicing, check if we still have a full packet's worth of data
            if (comBuffer.length < PACKET_SIZE) break; 
        }

        // We know the buffer now starts with HEADER_BYTE and is at least PACKET_SIZE long
        const packet = comBuffer.slice(0, PACKET_SIZE);
        
        // --- Data Extraction ---
        // The payload is everything between the start byte (1) and the checksum (1)
        const payload = packet.slice(1, PACKET_SIZE - 1); 
        
        // 1. Unpack Metadata (6 bytes) using BIG ENDIAN (BE)
        // const depthIndex = payload.readUInt16BE(0); // Not used in our processing, but extracted for completeness
        // const tempScaled = payload.readInt16BE(2);   // Not used in our processing
        // const vDrvScaled = payload.readUInt16BE(4);  // Not used in our processing
        
        // 2. Unpack Samples (3600 bytes)
        // Samples start at payload offset 6 (NUM_METADATA_BYTES)
        const samplesBuffer = payload.slice(NUM_METADATA_BYTES);
        const rawSamples = [];
        for (let i = 0; i < NUM_SAMPLES; i++) {
            // Read 16-bit (2-byte) sample data
            console.log(i,samplesBuffer.readUInt16BE(i * 2));
            rawSamples.push(samplesBuffer.readUInt16BE(i * 2)); 
        }
        
        // Pass the extracted samples to the main application processor
        processDataPacket(rawSamples); 

        // 3. Slice buffer: Move past the packet we just processed
        comBuffer = comBuffer.slice(PACKET_SIZE);                    
    }
}


/**
 * Initializes the serial port connection and sets up the data listener.
 * !!! USER ACTION REQUIRED: Replace the MockDataStream() call with the real serialport implementation. !!!
 */
function startComPortListener() {
    // ----------------------------------------------------------------------
    // --- START REAL SERIALPORT IMPLEMENTATION HERE ---
    //
    const port = new SerialPort({ 
         path: COM_PORT_PATH, 
         baudRate: BAUD_RATE 
    });
    //
    port.on('error', (err) => { console.error('[COM ERROR]', err.message); });
    //
    // // Pipe the raw data chunk directly to our handler
    port.on('data', serialBufferHandler); 
    //
    // ----------------------------------------------------------------------

}


// --- 5. SONAR PROCESSING LOGIC (Dynamic Blind Zone & Smoothing - Unchanged) ---

/**
 * Calculates the average noise floor from the tail end of the samples (where only noise is expected).
 * @param {number[]} samples - Array of 1800 ADC values.
 * @returns {number} - The average noise floor value.
 */
function calculateNoiseFloor(samples) {
    // Use the last NOISE_FLOOR_RANGE samples
    const start = samples.length - NOISE_FLOOR_RANGE;
    if (start < 0) return 0;

    const tailSamples = samples.slice(start);
    const sum = tailSamples.reduce((acc, val) => acc + val, 0);
    
    return sum / NOISE_FLOOR_RANGE;
}

/**
 * Dynamically finds the end of the primary pulse/blind zone, defined as the first index
 * where the signal permanently drops to or below the calculated noise floor.
 * @param {number[]} samples - Array of 1800 ADC values.
 * @param {number} noiseFloor - The calculated average noise floor.
 * @returns {number} - The index where the signal is considered to have ended.
 */
function findBlindZoneEnd(samples, noiseFloor) {
    // We only need to check the first few hundred samples for the blind zone
    const searchLimit = 500; 
    const threshold = noiseFloor * 1.2; // Use a slight buffer above the pure average

    for (let i = 0; i < Math.min(samples.length, searchLimit); i++) {
        // Find the point where the signal drops close to the noise level
        if (samples[i] <= threshold) {
            // Return this index as the start of the usable measurement area
            return i;
        }
    }
    // Fallback: If no drop is detected, use a fixed safe limit
    return 150; 
}


/**
 * Finds the strongest reflection (max value and its index) in the raw sample array,
 * AFTER dynamically determining the end of the blind zone.
 * @param {number[]} samples - Array of 1800 ADC values.
 * @returns {object} - { value, index, distance, blindZoneEndIndex }
 */
function findPrimaryReflection(samples) {
    if (samples.length === 0) {
        return { value: 0, index: -1, distance: 0, blindZoneEndIndex: 0 };
    }

    // 1. Calculate the dynamic noise floor and start index
    const noiseFloor = calculateNoiseFloor(samples); 
    const startIndex = findBlindZoneEnd(samples, noiseFloor);
    
    let maxValue = 0;
    let maxIndex = -1;

    // 2. Start the search for the strongest peak AFTER the dynamically determined blind zone
    for (let i = startIndex; i < samples.length; i++) {
        if (samples[i] > maxValue) {
            maxValue = samples[i];
            maxIndex = i;
        }
    }
    
    // The calculated distance uses the index of the max value and the new calibrated resolution
    const rawDistanceCm = maxIndex * SAMPLE_RESOLUTION;

    return { 
        value: maxValue, 
        index: maxIndex, 
        distance: rawDistanceCm, // Return the raw, unfiltered distance
        blindZoneEndIndex: startIndex
    };
}

/**
 * Applies Exponential Moving Average (EMA) to the distance for smoothing.
 * This filters out short noise spikes while still tracking movement.
 * @param {number} currentDistance - The raw distance value from the latest sample.
 * @returns {number} - The smoothed distance value.
 */
function applyExponentialSmoothing(currentDistance) {
    if (lastSmoothedDistance === null) {
        // Initialize with the first reading
        lastSmoothedDistance = currentDistance;
        return currentDistance;
    }

    // EMA Formula: Smoothed = (Alpha * Current) + ((1 - Alpha) * Previous Smoothed)
    const smoothedDistance = 
        (EMA_ALPHA * currentDistance) + 
        ((1 - EMA_ALPHA) * lastSmoothedDistance);

    lastSmoothedDistance = smoothedDistance;
    return smoothedDistance;
}


// --- 6. MAIN DATA PROCESSING FUNCTION ---

/**
 * Called asynchronously when a complete data packet is received from the serial port.
 * @param {number[]} rawSamples - The array of 1800 ADC values received from the sonar.
 */
async function processDataPacket(rawSamples) {
    // 1. --- Print Raw Samples for Debugging ---
    if (PRINT_RAW_SAMPLES) {
        // Truncate the output to the first 100 samples and the last 10 samples
        //const partialSamples = rawSamples.slice(0, 100).concat(['...'], rawSamples.slice(-10));
        //console.log(`[RAW FRAME SAMPLES - Partial View] ${JSON.stringify(partialSamples)}`);
        //console.log(`[RAW FRAME SAMPLES - Full View] ${JSON.stringify(rawSamples)}`);
        //console.log(rawSamples);
    }

    const reflection = findPrimaryReflection(rawSamples);
    
    // 2. Apply smoothing filter to the raw distance
    const smoothedDistance = applyExponentialSmoothing(reflection.distance);
    
    // Check if the detected peak is strong enough to be considered a real reflection
    if (reflection.value < 50) {
        console.log(`[LOG WARN] Skipping log: Peak value ${reflection.value.toFixed(0)} is below threshold (50).`);
        return;
    }
}

/**
 * Main execution block
 */
async function main() {

    
    startComPortListener(); // Start the sonar data listener (now event-driven)
    
    console.log(`\nLogger started. Distance is now smoothed (EMA Alpha: ${EMA_ALPHA}).`);
    if (PRINT_RAW_SAMPLES) {
        console.log("!!! VERBOSE MODE ACTIVE: Partial raw sample array will be printed for each frame. !!!");
        console.log("!!! NOTE: Output is truncated to the first 100 and last 10 samples for readability. !!!");
    }
}

main().catch(err => {
    console.error('An unrecoverable error occurred in main:', err);
    process.exit(1);
});

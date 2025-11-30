// server.js

const { SerialPort } = require('serialport');
const { program } = require('commander');
const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const path = require('path');

// --- Sonar Hardware Configuration ---
const BAUD_RATE = 250000;
const NUM_SAMPLES = 1800; 
const PAYLOAD_LEN = 6 + 2 * NUM_SAMPLES; // 3606 bytes (Metadata + Samples)
// PACKET_LEN: Header (1) + Payload (3606) + Footer (1) = 3608 bytes
const PACKET_LEN = 1 + PAYLOAD_LEN + 1; 
const PACKET_HEADER = 0xAA;

// --- Distance Conversion Constants (UPDATED with user's values) ---
const SPEED_OF_SOUND = 343;             // Speed of sound in M/s
const SAMPLE_TIME = 12.2e-6;            // Time per sample in seconds
// Distance Factor: (Time_per_sample * Speed_of_Sound) / 2 (two-way trip)
const M_PER_SAMPLE_FACTOR = (SAMPLE_TIME * SPEED_OF_SOUND) / 2; 

// --- Signal Tracking Configuration ---
const NUM_SIGNALS = 20;                 
const SIGNAL_THRESHOLD = 60;            
const CONSISTENCY_SAMPLES = 10;          
const POSITION_TOLERANCE = 1;          
const NOISE_FLOOR_RANGE = 200;          
const MIN_SIGNAL_SEPARATION = 20;        
const SNR_FACTOR = 2.5;     
const SONAR_FREQUENCY = 40; // 40 or 200            
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

let packetcounter = 0;
let wss; // WebSocket Server instance


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
    const startIndex = NUM_SAMPLES - NOISE_FLOOR_RANGE;
    if (startIndex < 0) return { noiseFloor: 0 }; 
    const noiseSamples = values.slice(startIndex, NUM_SAMPLES);
    const sum = noiseSamples.reduce((a, b) => a + b, 0);
    const noiseFloor = Math.round(sum / noiseSamples.length);
    return { noiseFloor };
}

function xcalculateNoiseFloor(values) {
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


function extractSignalIndices(values, consistentIndices) {
    // Change to an array of objects to store both index and amplitude
    const detectedSignals = [];
    
    let lastPulseEnd = -1;
    let signalCount = 0;

    for (let i = 0; i < values.length; i++) {
        
        // Start searching for the next signal only after the separation threshold
        if (i < lastPulseEnd + MIN_SIGNAL_SEPARATION) {
            continue;
        }

        // Check for the start of a new consistent signal cluster
        if (consistentIndices.has(i) && values[i] >= SIGNAL_THRESHOLD) {
            
            // Found the start of Signal (N+1)
            const currentSignalStart = i;
            const currentSignalAmplitude = values[i]; // <-- New: Get the amplitude
            let currentPulseWidth = 0;
            let currentPulseEnd = -1;
            
            // Calculate Pulse Width and End Index of the current Signal
            for (let j = currentSignalStart; j < values.length; j++) {
                if (values[j] >= SIGNAL_THRESHOLD) {
                    currentPulseWidth++;
                } else {
                    currentPulseEnd = j; 
                    break;
                }
            }
            // Handle case where pulse extends to the end of the array
            if (currentPulseEnd === -1 && currentPulseWidth > 0) {
                currentPulseEnd = values.length - 1; 
            }
            
            // Store the index and amplitude in the new array
            detectedSignals.push({
                index: currentSignalStart,
                amplitude: currentSignalAmplitude // <-- New: Store amplitude
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
        detectedSignals.push({ index: -1, amplitude: -1 });
    }

    return detectedSignals;
}


function initWebServer(portName) {
    const app = express();
    const server = http.createServer(app);
    const HTTP_PORT = 3000;

    app.use(express.static(path.join(__dirname)));
    app.get('/', (req, res) => {
        res.sendFile(path.join(__dirname, 'index.html'));
    });

    wss = new WebSocket.Server({ server });

    server.listen(HTTP_PORT, () => {
        console.log(`HTTP server listening on http://SERVER_IP:${HTTP_PORT}`);
        initSerialPort(portName);
    });
}

function initSerialPort(portName) {
    const tracker = new SignalTracker(CONSISTENCY_SAMPLES, SIGNAL_THRESHOLD, POSITION_TOLERANCE);
    const noiseAverager = new NoiseFloorAverager();
    
    let port;

    try {
        port = new SerialPort({ path: portName, baudRate: BAUD_RATE, autoOpen: false });

        port.open(err => {
            if (err) {
                console.error(`\n❌ Error opening serial port '${portName}':`, err.message);
                console.log(`Please ensure the device is connected and the port name is correct.`);
                return;
            }
            console.log(`\n✅ Serial port '${portName}' opened at ${BAUD_RATE} baud.`);
        });

        port.on('data', data => {
            buffer = Buffer.concat([buffer, data]);
            
            let packet;
            while ((packet = extractAndProcessPacket())) {
                
                const { values } = packet;
                packetcounter++;

                const { noiseFloor } = calculateNoiseFloor(values);
                noiseAverager.update(noiseFloor);
                const runningNoiseAvgValue = parseFloat(noiseAverager.getRunningAverage());
                const { min, max } = noiseAverager.getMinMax();
                const runningStdDevValue = parseFloat(noiseAverager.getStandardDeviation()); 
                const noiseFilterAmplitude = parseFloat((runningNoiseAvgValue + runningStdDevValue).toFixed(1));
                const blindZoneIndexEnd = findBlindZoneEnd(values, runningNoiseAvgValue); 

                // 2. Get Consistent Indices and Filter out Blind Zone/Noise
                // NOTE: We rely on the tracker to filter for *consistent* signals.
                const consistentIndices = tracker.updateAndGetConsistentIndices(values);


                const detectedSignals = extractSignalIndices(values, consistentIndices);
                const signalIndices = detectedSignals.map(s => s.index); 
                const dataToSend = {
                    frame: packetcounter,
                    signals: signalIndices,
                    // Sending the conversion factor and raw sample values  
                    noiseAvg: runningNoiseAvgValue,
                    stdDev: runningStdDevValue,
                    threshold: noiseFilterAmplitude,
                    blindZoneEnd: blindZoneIndexEnd,
                    numSignals: NUM_SIGNALS,
                    maxSamples: NUM_SAMPLES,
                    M_PER_SAMPLE: M_PER_SAMPLE_FACTOR // NEW: Send the conversion factor
                };

                if (wss) {
                    const jsonPayload = JSON.stringify(dataToSend);
                    wss.clients.forEach(client => {
                        if (client.readyState === WebSocket.OPEN) {
                            client.send(jsonPayload);
                        }
                    });
                }
                console.log(`${SONAR_FREQUENCY}KHz Frame ${packetcounter}: Found ${detectedSignals.filter(s => s.index !== -1).length} signals. Noise Floor (Frame): ${noiseFloor.toFixed(1)} (Running Stats: Avg: ${runningNoiseAvgValue.toFixed(1)} | Std Dev: ${runningStdDevValue.toFixed(1)} | Min: ${min.toFixed(1)} | Max: ${max.toFixed(1)})`);
            }
        });

        port.on('error', err => {
            console.error("\n❌ Serial Port Error:", err.message);
        });

        port.on('close', () => {
            console.log("\n❌ Serial Port Closed.");
        });

    } catch (e) {
        console.error(`\n❌ Fatal Error: ${e.message}`);
        if (port && port.isOpen) {
            port.close();
        }    
    }
}


// --- Command Line Interface ---

program
    .arguments('<port>')
    .description('Starts the Node.js server and serial monitor for web plotting.')
    .action(async (port) => {
        initWebServer(port);
    })
    .on('--help', () => {
        console.log('\nExample usage: node server.js /dev/ttyACM0');
    });

program.parse(process.argv);

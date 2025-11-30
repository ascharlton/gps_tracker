// blessed_console.js
// To run: node blessed_console.js [port]

const { SerialPort } = require('serialport');
const { program } = require('commander');
const blessed = require('blessed'); // Import the blessed library

// --- Configuration (MUST MATCH ARDUINO FIRMWARE) ---
const BAUD_RATE = 250000;
const NUM_SAMPLES = 1800;
const PAYLOAD_LEN = 6 + 2 * NUM_SAMPLES;
const PACKET_LEN = 1 + PAYLOAD_LEN + 1; // Header (1) + Payload + Checksum (1)
const PACKET_HEADER = 0xAA;

// --- TRACKING CONFIGURATION ---
const THRESHOLD = 50;                  // Minimum value a point must have to be considered a peak
const CONSISTENCY_SAMPLES = 3;         // Minimum number of samples a peak must appear in to be 'consistent'
const POSITION_TOLERANCE = 1;          // Range (+/-) within which a peak position is considered the same
const NOISE_FLOOR_RANGE = 300;         // Number of tail samples to use for noise floor calculation

// Constants used for data conversion
const SPEED_OF_SOUND = 330;
const SAMPLE_TIME = 13.2e-6;
const SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2;
const MAX_DEPTH = NUM_SAMPLES * SAMPLE_RESOLUTION;

let packetcounter = 0;

// BLESSED color tags (used instead of raw ANSI codes)
const BLESSED_RED = '{red-fg}';
const BLESSED_BLUE = '{blue-fg}';
const BLESSED_ORANGE = '{yellow-fg}'; // Use yellow-fg for the orange effect
const BLESSED_RESET = '{/}';

// --- Signal Tracker Class for Time-Series Analysis (Unchanged) ---

class SignalTracker {
    constructor(bufferSize, threshold, tolerance) {
        this.buffer = [];
        this.bufferSize = bufferSize; 
        this.threshold = threshold;
        this.tolerance = tolerance;
        this.consistentIndices = new Set();
    }

    updateAndGetConsistentIndices(currentValues) {
        const currentPeakIndices = [];
        for (let i = 0; i < currentValues.length; i++) {
            if (currentValues[i] >= this.threshold) {
                currentPeakIndices.push(i);
            }
        }

        this.buffer.push(currentPeakIndices);
        if (this.buffer.length > this.bufferSize) {
            this.buffer.shift();
        }

        if (this.buffer.length < this.bufferSize) {
            this.consistentIndices.clear();
            return this.consistentIndices;
        }

        const indexCounts = {};

        for (const index of this.buffer[this.buffer.length - 1]) {
            let isConsistent = true;
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
                indexCounts[index] = (indexCounts[index] || 0) + 1;
            }
        }

        this.consistentIndices = new Set(Object.keys(indexCounts).map(Number));
        return this.consistentIndices;
    }
}

// --- Noise Floor Calculation (Unchanged) ---

function calculateNoiseFloor(samples) {
    const start = samples.length - NOISE_FLOOR_RANGE;
    if (start < 0) return 0;
    
    const tailSamples = samples.slice(start);
    const sum = tailSamples.reduce((acc, val) => acc + val, 0);
    
    return Math.round(sum / NOISE_FLOOR_RANGE);
}

// --- Noise Floor Averaging Class (Unchanged) ---

class NoiseFloorAverager {
    constructor() {
        this.totalSum = 0;
        this.frameCount = 0;
        this.minNoiseFloor = Infinity;
        this.maxNoiseFloor = -Infinity;
    }

    update(noiseFloorValue) {
        this.totalSum += noiseFloorValue;
        this.frameCount += 1;
        this.minNoiseFloor = Math.min(this.minNoiseFloor, noiseFloorValue);
        this.maxNoiseFloor = Math.max(this.maxNoiseFloor, noiseFloorValue);
    }

    getRunningAverage() {
        if (this.frameCount === 0) return 0;
        return (this.totalSum / this.frameCount).toFixed(1);
    }
    
    getMinMax() {
        if (this.frameCount === 0) {
            return { min: 0, max: 0 };
        }
        return {
            min: Math.round(this.minNoiseFloor),
            max: Math.round(this.maxNoiseFloor),
        };
    }
}

// --- Core Packet Reading Function (Unchanged logic, kept external) ---

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
            // Note: Error is logged to a blessed Log element at the bottom in the main function
            // console.error(...) is replaced later in the main loop
            buffer = buffer.subarray(headerIndex + 1);
            isReading = false;
            return extractAndProcessPacket();
        }

        const depthIndex = payload.readUInt16BE(0);
        const tempScaled = payload.readInt16BE(2);
        const vDrvScaled = payload.readUInt16BE(4);

        const values = [];
        for (let i = 0; i < NUM_SAMPLES; i++) {
            values.push(payload.readUInt16BE(6 + i * 2));
        }

        const depthCm = parseFloat((depthIndex * SAMPLE_RESOLUTION).toFixed(1));
        const temperature = parseFloat((tempScaled / 100.0).toFixed(1));
        const driveVoltage = parseFloat((vDrvScaled / 100.0).toFixed(1));

        buffer = buffer.subarray(headerIndex + PACKET_LEN);
        packet = { values, depthCm, temperature, driveVoltage };

    } catch (e) {
        // console.error(...) is replaced later in the main loop
        isReading = false;
        return null;
    }

    isReading = false;
    return packet;
}

// --- Main Execution Loop (MODIFIED FOR BLESSED) ---

async function runSerialConsole(portName) {

    // ------------------------------------
    // 1. BLESSED UI SETUP
    // ------------------------------------

    const screen = blessed.screen({
        smartCSR: true, // Use full-screen character-based rendering for smoother updates
        title: 'Open Echo Sonar Console',
        // Preserve console history on exit
        fullUnicode: true,
        autoPadding: true,
    });

    // Quit on Ctrl+C or q
    screen.key(['escape', 'q', 'C-c'], function(ch, key) {
        if (port && port.isOpen) {
            port.close();
        }
        return process.exit(0);
    });

    // --- Element: Status (Metadata Header) ---
    const statusBox = blessed.box({
        top: 0,
        left: 'center',
        width: '100%',
        height: 5,
        content: `Connecting to ${portName} @ ${BAUD_RATE} baud...`,
        tags: true, // Enable blessed color tags
        border: {
            type: 'line'
        },
        style: {
            fg: 'white',
            bg: 'black',
            border: {
                fg: 'cyan'
            },
        }
    });

    // --- Element: Data Table (The main grid) ---
    const tableBox = blessed.box({
        top: 5, // Below the status box
        left: 'center',
        width: '100%',
        height: '100%-6', // Screen height - status box (5) - 1 for log/padding
        content: '',
        tags: true, // Enable blessed color tags
        scrollable: true,
        alwaysScroll: true,
        style: {
            fg: 'white',
            bg: '',
        }
    });

    // --- Element: Log (for errors/connection messages) ---
    const logBox = blessed.log({
        bottom: 0,
        left: 0,
        width: '100%',
        height: 1,
        tags: true,
        style: {
            fg: 'white',
            bg: 'blue',
        }
    });
    
    // Append elements and render
    screen.append(statusBox);
    screen.append(tableBox);
    screen.append(logBox);
    screen.render();
    
    // Helper function to log messages to the bottom bar
    const uiLog = (msg, color = 'white') => {
        logBox.setContent(`{${color}-fg}${msg}{/}`);
        screen.render();
    };

    // --- Initial setup messages ---
    uiLog(`Attempting to connect to: ${portName} @ ${BAUD_RATE} baud...`, 'cyan');
    
    // ------------------------------------
    // 2. SERIAL PORT LOGIC
    // ------------------------------------
    
    // Initialize the Signal Tracker and Averager
    const tracker = new SignalTracker(CONSISTENCY_SAMPLES, THRESHOLD, POSITION_TOLERANCE);
    const noiseAverager = new NoiseFloorAverager();
    let port;

    try {
        port = new SerialPort({ path: portName, baudRate: BAUD_RATE, autoOpen: false });

        port.open(err => {
            if (err) {
                uiLog(`❌ Failed to open serial port ${portName}: ${err.message}`, 'red');
                return;
            }
            uiLog(`✅ Connected to ${portName}. Reading data...`, 'green');
        });

        port.on('data', data => {
            buffer = Buffer.concat([buffer, data]);
            
            let packet;
            while ((packet = extractAndProcessPacket())) {
                
                const { values, depthCm, temperature, driveVoltage } = packet;

                packetcounter++;

                // 1. Calculate and Track Noise Floor
                const noiseFloor = calculateNoiseFloor(values);
                noiseAverager.update(noiseFloor);
                const runningNoiseAvg = noiseAverager.getRunningAverage();
                const { min, max } = noiseAverager.getMinMax();

                // 2. Get Consistent Indices
                const consistentIndices = tracker.updateAndGetConsistentIndices(values);

                // --- TABLE GENERATION LOGIC (MODIFIED FOR BLESSED TAGS) ---
                
                const NUM_COLUMNS = 50;
                const NUM_ROWS = values.length / NUM_COLUMNS; 
                const CELL_WIDTH = 3; 
                const SPACE_SEPARATOR = ' '; 
                const ROW_LABEL_WIDTH = 3; 

                const processedValues = [];

                for (let i = 0; i < values.length; i++) {
                    const value = values[i];
                    const isConsistent = consistentIndices.has(i);
                    let outputString = '';
                    
                    if (isConsistent && value >= THRESHOLD) {
                        if (i > 1200) { // certain distance away
                            const paddedValue = value.toString().padStart(CELL_WIDTH, ' ');
                            // Use BLESSED_RED
                            outputString = `${BLESSED_RED}${paddedValue}${BLESSED_RESET}`;
                        } else {
                            outputString = value.toString().padStart(CELL_WIDTH, ' ');
                        }
                    } 
                    else if (value < THRESHOLD && value > noiseFloor) { 
                        const paddedValue = value.toString().padStart(CELL_WIDTH, ' ');
                        // Use BLESSED_ORANGE (yellow-fg)
                        //outputString = `${BLESSED_ORANGE}${paddedValue}${BLESSED_RESET}`;
                        outputString = ' '.repeat(CELL_WIDTH);
                    }
                    else {
                        // Inconsistent peak (high value) or background noise (low value or below noise floor)
                        //const paddedValue = value.toString().padStart(CELL_WIDTH, ' ');
                        //outputString = `${BLESSED_BLUE}${paddedValue}${BLESSED_RESET}`;
                        outputString = ' '.repeat(CELL_WIDTH);
                    }
                    
                    processedValues.push(outputString + SPACE_SEPARATOR);
                }

                // ----------------------------------------------------
                // Build the Console Table String

                // Determine the total table width for formatting
                const COL_LABEL_WIDTH = CELL_WIDTH + 1; 
                const tableWidth = ROW_LABEL_WIDTH + 3 + (NUM_COLUMNS * COL_LABEL_WIDTH); 

                // 1. Build Header Row (Labels 1 to 50)
                let header = ' '.repeat(ROW_LABEL_WIDTH) + ' |'; // Space for row label | space

                for (let c = 1; c <= NUM_COLUMNS; c++) {
                    header += c.toString().padStart(COL_LABEL_WIDTH, ' ');
                }
                header = header.trimEnd();

                // 2. Build Separator Line
                const separator = '-'.repeat(ROW_LABEL_WIDTH) + '-|-' + '-'.repeat(header.length - ROW_LABEL_WIDTH - 2); 

                // 3. Build Data Rows (1 to 36)
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


                // --- BLESSED UI UPDATE ---
                
                const now = new Date();
                const timeString = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}`;
                
                // 1. Update Status Box (Metadata)
                const statusContent = [
                    `{bold}{cyan-fg}SONAR CONSOLE - Packet ${packetcounter}{/}`,
                    `Time: ${timeString} | Depth: ${depthCm} cm | Temp: ${temperature} °C | Signal Threshold: ${THRESHOLD} Noise Floor: {white-fg}${noiseFloor}{/} (Avg: ${runningNoiseAvg}, Min: ${min}, Max: ${max})`,
                    `Signals: Not Consistent ( ) | ${BLESSED_RED}Consistent Peaks over {/} (${CONSISTENCY_SAMPLES}) samples | ${BLESSED_ORANGE}Sub-Threshold Signals (possibly noise){/}`
                ].join('\n');
                
                statusBox.setContent(statusContent);

                // 2. Update Table Box (Data Grid)
                const tableContent = [
                    header,
                    separator,
                    ...tableRows
                ].join('\n');
                
                tableBox.setContent(tableContent);

                // 3. Render the screen to show updates smoothly
                screen.render();
            }
        });

        port.on('error', err => {
            uiLog(`❌ Serial Port Error: ${err.message}`, 'red');
        });

        port.on('close', () => {
            uiLog("❌ Serial Port Closed. Press 'q' or 'Ctrl+C' to exit.", 'red');
        });


    } catch (e) {
        uiLog(`❌ Critical Error: ${e.message}`, 'red');
        if (port && port.isOpen) {
            port.close();
        }
    }
}

// --- Command Line Interface (Unchanged) ---

program
    .arguments('[port]')
    .description('Starts the Node.js serial console for the Open Echo Sonar (Blessed TUI).')
    .action(async (port) => {
        let selectedPort = port;

        if (!selectedPort) {
            const ports = await SerialPort.list();
            const availablePorts = ports.map(p => p.path);
            
            if (availablePorts.length === 0) {
                selectedPort = process.platform.startsWith('win') ? 'COM1' : '/dev/ttySONAR';
                console.warn(`\n⚠️ No serial ports found. Defaulting to '${selectedPort}'.`);
            } else {
                selectedPort = availablePorts[0];
                console.log(`Found available serial ports: ${availablePorts}. Using first: ${selectedPort}`);
            }
        }
        
        runSerialConsole(selectedPort);
    });

program.parse(process.argv);
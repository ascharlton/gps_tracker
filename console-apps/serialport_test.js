// serial_test.js

const { SerialPort } = require('serialport');

// --- Configuration ---
const SERIAL_PORT = '/dev/ttySONAR'; // <--- VERIFY THIS PORT
const BAUD_RATE = 250000;
const READ_TIMEOUT_MS = 5000; // Timeout after 5 seconds of no data

console.log(`\n--- Node.js Serial Port Debugger ---`);
console.log(`Port: ${SERIAL_PORT}, Baud: ${BAUD_RATE}`);

let lastDataTime = Date.now();

try {
    const port = new SerialPort({ 
        path: SERIAL_PORT, 
        baudRate: BAUD_RATE,
        // Using autoOpen: true means it attempts to open immediately
        autoOpen: true 
    });

    port.on('open', () => {
        console.log(`✅ SUCCESS: Serial port opened.`);
        port.flush((err) => {
            if (err) console.error('Error flushing buffer:', err);
            console.log('Starting raw data monitor...');
        });
    });

    port.on('data', (data) => {
        const now = new Date();
        lastDataTime = now;
        
        // Print the raw received buffer data
        console.error(`\n--- Received ${data.length} bytes at ${now.toISOString()} ---`);
        console.log('Buffer (Hex):', data.toString('hex'));
        console.log('Buffer (Raw):', data);
        console.error(`--- Complete ---`);
        
        // Optional: Check if a header byte is present (0xAA)
        if (data.includes(0xAA)) {
            console.log(`[INFO] Header (0xAA) found in this packet.`);
            process.exit();
        }
    });

    port.on('error', (err) => {
        console.error(`\n❌ SERIAL ERROR:`, err.message);
        console.log('Check if the port is correct and permissions (e.g., dialout group) are set.');
        process.exit(1);
    });

    port.on('close', () => {
        console.log('\n🔌 Port closed.');
        process.exit(0);
    });
    
    // --- Timeout Check ---
    const timeoutInterval = setInterval(() => {
        if (Date.now() - lastDataTime > READ_TIMEOUT_MS) {
            console.error(`\n⏰ TIMEOUT: No data received for ${READ_TIMEOUT_MS / 1000} seconds.`);
            console.log('Verify the device is sending data and the baud rate is correct.');
            clearInterval(timeoutInterval);
            port.close();
        }
    }, 1000);


} catch (e) {
    console.error(`\n❌ FATAL INITIALIZATION ERROR: ${e.message}`);
    process.exit(1);
}

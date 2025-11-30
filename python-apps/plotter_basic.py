import sys
import numpy as np
import serial
import serial.tools.list_ports
import struct
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel, QHBoxLayout
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QKeySequence
import pyqtgraph as pg
import qdarktheme

# --- Serial Configuration (Retained) ---
BAUD_RATE = 250000
NUM_SAMPLES = 1800  # Number of frequency/amplitude bins (Y-axis now)

# Plot Configuration
PLOT_WIDTH = 1280  # Number of time steps (X-axis)
PLOT_HEIGHT = 860 # Height of the window for fitting the plot

# --- Data Conversion Constants (Retained) ---
SPEED_OF_SOUND = 330
SAMPLE_TIME = 13.2e-6
SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2
MAX_DEPTH = NUM_SAMPLES * SAMPLE_RESOLUTION
PACKET_SIZE = 1 + 6 + 2 * NUM_SAMPLES + 1

# --- Color/Level Configuration ---
COLOR_LEVELS = (0, 100) 
DEFAULT_COLORMAP = 'viridis' 


# --- Core Packet Reading Function (Retained) ---

# --- Core Packet Reading Function (Modified for Robustness) ---

def read_packet(ser):
    """
    Reads a single, verified packet from the serial port.
    Returns unpacked sensor data and sample array, or None if reading fails.
    """
    payload_len = 6 + 2 * NUM_SAMPLES
    
    # Use a small read size for the header to avoid blocking long if timeout is large
    header = ser.read(1) 
    
    # CHECK 1: Ensure we got a byte and it's the start marker
    if header != b'\xAA':
        # If we didn't get the header, clear the input buffer to prevent 
        # reading an incomplete sequence on the next try.
        if header:
            # Only flush if some junk data was actually read
            ser.reset_input_buffer()
        return None 

    # 2. Read Payload and Checksum
    payload = ser.read(payload_len)
    checksum = ser.read(1)

    # CHECK 2: Ensure we read a complete packet
    if len(payload) != payload_len or len(checksum) != 1:
        # If incomplete, flush buffer and wait for a new packet start
        ser.reset_input_buffer()
        return None 

    # 3. Verify Checksum
    calc_checksum = 0
    for byte in payload:
        calc_checksum ^= byte

    if calc_checksum != checksum[0]:
        # Checksum failed, discard and wait for a new packet
        ser.reset_input_buffer()
        return None 

    # 4. Unpack Payload and Samples
    depth_index, temp_scaled, vDrv_scaled = struct.unpack(">HhH", payload[:6])
    samples = struct.unpack(f">{NUM_SAMPLES}H", payload[6:])

    # 5. Scale and Calculate Real Values
    depth_cm = depth_index * SAMPLE_RESOLUTION
    temperature = temp_scaled / 100.0
    drive_voltage = vDrv_scaled / 100.0
    values = np.array(samples, dtype=np.uint16)

    return values, depth_cm, temperature, drive_voltage 

# --- QThread for Serial Reading (Retained) ---

class SerialReader(QThread):
    data_received = pyqtSignal(np.ndarray, float, float, float)

    def __init__(self, port, baud_rate, parent=None): # Accept 'parent' argument
        super().__init__(parent)  # Pass 'parent' to the QThread initializer
        self.port = port
        self.baud_rate = baud_rate
        self.running = True

    def run(self):
        """Continuously read serial data and emit processed arrays."""
        try:
            # Use fixed timeout for non-blocking read
            with serial.Serial(self.port, self.baud_rate, timeout=0.01) as ser: 
                print(f"✅ Serial Thread connected to {self.port} at {self.baud_rate} baud.")
                ser.reset_input_buffer() # Ensure a clean start to clear any prior junk
                
                # CRITICAL: Wait briefly for the device to initialize and send data
                time.sleep(1.0) 
                
                while self.running:
                    # Added a check to see if we have enough data bytes for a full packet
                    # Note: ser.in_waiting is only a hint, not a guarantee
                    if ser.in_waiting >= PACKET_SIZE:
                        result = read_packet(ser)
                    else:
                        result = None

                    if result:
                        values, depth_cm, temperature, drive_voltage = result
                        self.data_received.emit(values, depth_cm, temperature, drive_voltage)
                    
                    # Small sleep to prevent 100% CPU utilization
                    time.sleep(0.0001)
                        
        except serial.SerialException as e:
            print(f"❌ SERIAL ERROR: Port '{self.port}' access failed. Check connection/permissions: {e}")
        except Exception as e:
            print(f"❌ An unexpected error occurred in the serial thread: {e}")

    def stop(self):
        self.running = False
        self.quit()
        self.wait()


# --- PyQtGraph Plotting Application ---

class SimpleWaterfallApp(QMainWindow):
    def __init__(self, serial_port):
        super().__init__()
        self.setWindowTitle("Real-Time Ultrasonic Echo Plot (Press 'Q' to Quit)")
        
        self.setGeometry(200, 200, PLOT_WIDTH + 240, PLOT_HEIGHT + 160) 
        
        self.data = np.zeros((PLOT_WIDTH, NUM_SAMPLES), dtype=np.uint16) 

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # === Waterfall Plot Setup ===
        self.waterfall = pg.PlotWidget()
        self.waterfall.setFixedSize(PLOT_WIDTH, PLOT_HEIGHT) 
        self.imageitem = pg.ImageItem(axisOrder='row-major')
        
        self.waterfall.setYRange(0, NUM_SAMPLES)
        self.waterfall.setXRange(0, PLOT_WIDTH)
        
        y_axis_labels = {
            int(i / SAMPLE_RESOLUTION): f"{i / 100:.1f}m" 
            for i in range(0, int(MAX_DEPTH), 50) # Every 50cm
        }
        y_axis_ticks = list(y_axis_labels.items())
        self.waterfall.getAxis("left").setTicks([y_axis_ticks])
        self.waterfall.getAxis("bottom").setLabel("Time (Frames)")
        
        self.waterfall.addItem(self.imageitem)
        
        # === Colorbar Setup ===
        self.colorbar = pg.HistogramLUTWidget()
        self.colorbar.setImageItem(self.imageitem)
        self.colorbar.item.gradient.loadPreset(DEFAULT_COLORMAP)
        self.imageitem.setLevels(COLOR_LEVELS)
        self.colorbar.setMaximumHeight(PLOT_HEIGHT)
        self.colorbar.setMinimumWidth(80) 

        # Add Plot and Colorbar side-by-side
        plot_control_layout = QHBoxLayout()
        plot_control_layout.addWidget(self.waterfall)
        plot_control_layout.addWidget(self.colorbar)
        #plot_control_layout.setContentsMargins(200,200,200,200)
        main_layout.addLayout(plot_control_layout)
        
        # === Info Label ===
        self.info_label = QLabel("Serial Status: Initializing...")
        main_layout.addWidget(self.info_label)

        # === Serial Thread Setup ===
        # Pass 'self' (the QMainWindow) as the parent, or pass None/omit the parent argument.
        # Passing 'self' is good practice.
        self.serial_thread = SerialReader(serial_port, BAUD_RATE, parent=self) 
        self.serial_thread.data_received.connect(self.update_plot)
        self.serial_thread.start()


    # === NEW: Keyboard Event Handler ===
    def keyPressEvent(self, event):
        """Handle keyboard input for application control."""
        if event.key() == Qt.Key_Q:
            print("\n🛑 'Q' pressed. Closing application.")
            self.close() # Calls closeEvent()
        else:
            super().keyPressEvent(event)

    def update_plot(self, new_samples, depth_cm, temperature, drive_voltage):
        """
        Receives new samples from the serial thread and updates the scrolling plot.
        """
        # 1. Scroll the plot data one column to the left
        self.data = np.roll(self.data, -1, axis=0)
        
        # 2. Add the new sample array to the far right column
        self.data[-1, :] = new_samples 
        
        # 3. Update the image in the plot
        self.imageitem.setImage(self.data.T, autoLevels=False)
        
        # 4. Update info label
        self.info_label.setText(
            f"Depth: {depth_cm:.1f} cm | Temp: {temperature:.1f} °C | Vdrv: {drive_voltage:.1f} V"
        )


    def closeEvent(self, event):
        """Handles graceful shutdown of the serial thread on app exit."""
        if self.serial_thread:
            print("\n🛑 Shutting down serial thread...")
            self.serial_thread.stop()
        event.accept()


# --- Main Execution ---

if __name__ == "__main__":
    
    # 1. Determine Serial Port
    ports = [p.device for p in serial.tools.list_ports.comports()]
    
    if len(ports) == 0:
        DEFAULT_PORT = "/dev/ttyACM0" 
    else:
        DEFAULT_PORT = ports[0]
        
    selected_port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    
    print(f"Using serial port: {selected_port}")
    
    # 2. Start Application
    app = QApplication(sys.argv)
    
    # Apply dark theme
    qdarktheme.setup_theme("dark")
    
    window = SimpleWaterfallApp(selected_port)
    window.show()
    
    # Run the main PyQt event loop
    sys.exit(app.exec_())

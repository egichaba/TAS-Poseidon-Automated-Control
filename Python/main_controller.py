import sys
import time
import re
from pathlib import Path
from collections import deque
from math import pi
from PyQt5 import QtCore, QtWidgets 
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QLineEdit, QComboBox, QFormLayout,
                             QPushButton, QDoubleSpinBox, QTextEdit, QGroupBox,
                             QCheckBox, QSpinBox)
import pyqtgraph as pg
import serial
from serial.tools import list_ports
import threading
import queue
import csv # <--- NEW: Import the CSV module

# ---------------- Config / constants ----------------
CONFIG_PATH = Path("/tmp/candy_config.json")
NUM_DISPENSERS = 2
BAUDRATE = 115200
#PORT = "/dev/cu.usbmodem1101" this is for mac
PORT = "COM13" #For windows
MAX_POINTS = 1500       # how many points to keep in the rolling buffer
#UPDATE_INTERVAL_MS = 50  # GUI update interval (ms) -> 20 Hz
UPDATE_INTERVAL_MS = 500  # GUI update interval (ms) -> 2 Hz

SERIAL_TIMEOUT = 1.0

LEAD_SCREW_PITCH_MM = 0.8  # New value from user's formula
STEPS_PER_REV = 3200

SMOTOR = 200
M = 16
P = 0.8

DELAY_TIME = 60.0   # seconds for averaging/log update (1 minute)
#DELAY_TIME = 5.0   # 5 second average
MAX_PRESSURE_POINTS = 3600
DEFAULT_FLOW_UL_PER_MIN = 10.0

############################################## PID ADDED: defaults and safety limits
DT_DEFAULT = 0.05  # legacy default dt
K_OUT_STEPS_PER_MMHG = 2.0 # legacy
OUTPUT_MAX_STEPS = 800.0 # legacy
DEADBAND_MMHG = 0.5 # small deadband to avoid chatter
INTEGRAL_CLAMP = 200.0 # legacy
############################ Pressure control
CONTROL_FLOW_STEP = 1.0          # normal increase step
HIGH_PRESSURE_DROP_STEP = 15.0   # aggressive decrease when pressure is too high
SLOPE_LIMIT_MMHG_PER_MIN = 5.0   # if pressure rising faster than this, don't increase
PREDICT_LOOKAHEAD_MIN = 1.0      # predict 1 minute ahead

MAX_RECOVERY_ATTEMPTS_1 = 5
MAX_RECOVERY_ATTEMPTS_2 = 5
EQUALIZE_PAUSE_SEC = 5 * 60  

FINE_CONTROL_BAND_MMHG = 5.0      # within +/- 5 mmHg of target
FINE_CONTROL_STEP = 0.25          # small flow adjustment near target
FINE_SLOPE_LIMIT_MMHG_PER_MIN = 2.0

# --- NEW CONSTANT FOR CSV ---
# save CSV in the exact same folder as this Python script
SCRIPT_DIR = Path(__file__).parent.absolute()
CSV_FILE_PATH = SCRIPT_DIR / "pressure_log.csv" 

# ---------------- Serial handler (legacy - left for reference, not used below) ----------------
# (omitting autodetect_port and SerialHandler class for brevity, assume they are unchanged)
def autodetect_port(preferred=None):
    if preferred:
        return preferred
    candidates = []
    for p in list_ports.comports():
        dev = p.device or ""
        manu = (p.manufacturer or "").lower() if p.manufacturer else ""
        vid = f"{p.vid:04x}" if p.vid else ""
        score = 0
        if dev.startswith("/dev/cu.") or dev.startswith("COM"):
            score += 3
        if "arduino" in manu: score += 3
        if vid in {"2341", "2a03", "10c4", "1a86", "0403"}: score += 2
        candidates.append((score, dev))
    candidates.sort(reverse=True)
    return candidates[0][1] if candidates else None

class SerialHandler(threading.Thread):
    """Legacy thread-based handler (kept for reference). Not used by the new graphing code."""
    def __init__(self, port=None, baud=115200, incoming_callback=None, status_callback=None):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.incoming_callback = incoming_callback
        self.status_callback = status_callback
        self._outq = queue.Queue()
        self._stop_event = threading.Event()
        self.ser = None

    def run(self):
        port = autodetect_port(self.port)
        if not port:
            if self.status_callback: self.status_callback("No serial device found.")
            return
        try:
            self.ser = serial.Serial(port, self.baud, timeout=0.1)
            # allow Arduino reset
            time.sleep(1.2)
            if self.status_callback: self.status_callback(f"Connected: {port} @ {self.baud}")
        except Exception as e:
            if self.status_callback: self.status_callback(f"Serial open failed: {e}")
            return

        while not self._stop_event.is_set():
            try:
                line = self.ser.readline()
                if line:
                    sline = line.decode('utf-8', errors='ignore').strip()
                    if sline and self.incoming_callback:
                        self.incoming_callback(sline)
                try:
                    out = self._outq.get_nowait()
                    if self.ser:
                        if not out.endswith("\n"):
                            out += "\n"
                        self.ser.write(out.encode('utf-8'))
                except queue.Empty:
                    pass
                time.sleep(0.005)
            except Exception:
                time.sleep(0.1)
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                if self.status_callback: self.status_callback("Serial closed.")
        except Exception:
            pass

    def send(self, text):
        # ensure newline
        if not text.endswith("\n"):
            text += "\n"
        self._outq.put(text)

    def stop(self):
        self._stop_event.set()


# ---------------- Math helpers ----------------
# (omitting math helpers for brevity, assume they are unchanged)
def syringe_area_mm2(diameter_mm):
    """Area of syringe in mm^2."""
    return pi * (diameter_mm / 2.0) ** 2

def steps_per_mm(steps_per_rev=STEPS_PER_REV, pitch=LEAD_SCREW_PITCH_MM):
    """
    Calculates steps per millimeter based on the new fixed constants.
    steps/mm = STEPS_PER_REV / LEAD_SCREW_PITCH_MM
    """
    return steps_per_rev / pitch

def volume_per_step_uL(diameter_mm):
    """
    Calculates volume dispensed per step in uL.
    Volume_per_step = (Area_mm2 / steps_per_mm) * 1000 (mm^3/uL conversion)
    However, 1 mm^3 = 1 uL, so the * 1000 factor is cancelled by the uL unit.
    The simpler, correct formula is Area / steps_per_mm (uL/step).
    """
    A = syringe_area_mm2(diameter_mm)
    spmm = steps_per_mm()
    return A * spmm if spmm else 0.0

def flow_ul_per_min_to_steps_per_sec(flow_ul_per_min, diameter_mm):
    """
    Converts desired flow rate (uL/min) to pump speed (steps/sec).
    """

    A = syringe_area_mm2(diameter_mm)
    lead_pitch = 0.8
    steps_per_sec = (flow_ul_per_min * 3200) / (60*A*lead_pitch)
    
    return steps_per_sec

# ---------------- Pump panel ----------------
class PumpPanel(QGroupBox):
    # (omitting PumpPanel class for brevity, assume it is unchanged)
    def __init__(self, label, parent=None):
        super().__init__(label, parent)
        self._build_ui()

    def _build_ui(self):
        layout = QFormLayout()
        self.syringe_diameter = QDoubleSpinBox(); self.syringe_diameter.setRange(0.1,100.0); self.syringe_diameter.setValue(10.0)
        self.volume_to_expel = QDoubleSpinBox(); self.volume_to_expel.setRange(0.1,1e6); self.volume_to_expel.setValue(100.0)
        self.cpm_desired = QDoubleSpinBox(); self.cpm_desired.setRange(0.0,10000.0); self.cpm_desired.setValue(60.0)
        
        ################### REVISED: Target Pressure added here
        self.target_pressure = QDoubleSpinBox(); self.target_pressure.setRange(-1e6, 1e6); self.target_pressure.setValue(10.0)
        
        layout.addRow("Syringe Diameter (mm):", self.syringe_diameter)
        layout.addRow("Volume to Expel (µL):", self.volume_to_expel)
        layout.addRow("Target Pressure (mmHg):", self.target_pressure)
        self.init_btn = QPushButton("Initialize Pump Position (set pos=0)")
        layout.addRow(self.init_btn)
        self.setLayout(layout)

        

# ---------------- Control window ----------------
class ControlWindow(QWidget):
    run_command = QtCore.pyqtSignal(str)
    status_signal = QtCore.pyqtSignal(str)
    dispenser_changed = QtCore.pyqtSignal(int)
    # --- NEW: Signal to start CSV logging ---
    start_log_signal = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("TAS User Interface")
        # Initialize control elements *before* _build_ui so they exist for the controller
        self.pid_enable = QCheckBox("Enable Average-Based Feedback Control") 
        self.deadband = QDoubleSpinBox(); self.deadband.setRange(0.0, 10.0); self.deadband.setDecimals(3); self.deadband.setValue(DEADBAND_MMHG)
        self._build_ui()

    def _build_ui(self):
        main = QVBoxLayout()

        # Dispenser selection
        h = QHBoxLayout()
        h.addWidget(QLabel("Select TAS Model:"))
        self.dispenser_dropdown = QComboBox()
        self.dispenser_dropdown.addItems([f"TAS Model {i+1}" for i in range(NUM_DISPENSERS)])
        h.addWidget(self.dispenser_dropdown)
        main.addLayout(h)

        # Candy info
        form = QFormLayout()
        self.donor_id = QLineEdit()
        self.donor_age = QLineEdit()
        self.donor_race = QLineEdit()
        self.donor_sex = QLineEdit()
        form.addRow("Donor ID:", self.donor_id)
        form.addRow("Donor Age:", self.donor_age)
        form.addRow("Donor Race:", self.donor_race)
        form.addRow("Donor Sex:", self.donor_sex)
        
        # --- NEW: Start Log Button ---
        self.start_log_btn = QPushButton("Start CSV Log (Appends to 'pressure_log.csv')")
        form.addRow(self.start_log_btn)
        
        main.addLayout(form)

        # Flow control (global)
        flow_layout = QFormLayout()
        self.flow_rate = QDoubleSpinBox()
        self.flow_rate.setRange(0.0, 1e6)
        self.flow_rate.setDecimals(6)
        self.flow_rate.setSingleStep(0.001)
        self.flow_rate.setValue(DEFAULT_FLOW_UL_PER_MIN)

        flow_layout.addRow("Flow (µL/min):", self.flow_rate)

        main.addLayout(flow_layout)

        # ---------------- Pressure Calibration Controls ----------------
        cal_group = QGroupBox("Pressure Sensor Calibration")
        cal_form = QFormLayout()

        self.high_cal_pressure = QDoubleSpinBox()
        self.high_cal_pressure.setRange(1.0, 300.0)
        self.high_cal_pressure.setDecimals(2)
        self.high_cal_pressure.setSingleStep(1.0)
        self.high_cal_pressure.setValue(100.0)

        cal_form.addRow("High Reference Pressure (mmHg):", self.high_cal_pressure)

        cal_row1 = QHBoxLayout()
        self.zero1_btn = QPushButton("Set Sensor 1 Zero")
        self.high1_btn = QPushButton("Set Sensor 1 High")
        cal_row1.addWidget(self.zero1_btn)
        cal_row1.addWidget(self.high1_btn)
        cal_form.addRow(cal_row1)

        cal_row2 = QHBoxLayout()
        self.zero2_btn = QPushButton("Set Sensor 2 Zero")
        self.high2_btn = QPushButton("Set Sensor 2 High")
        cal_row2.addWidget(self.zero2_btn)
        cal_row2.addWidget(self.high2_btn)
        cal_form.addRow(cal_row2)

        self.clear_cal_btn = QPushButton("Clear Saved Calibration")
        cal_form.addRow(self.clear_cal_btn)

        cal_group.setLayout(cal_form)
        main.addWidget(cal_group)        

        # Pumps
        pumps_layout = QHBoxLayout()
        self.pump1_panel = PumpPanel("Pump 1")
        self.pump2_panel = PumpPanel("Pump 2")
        pumps_layout.addWidget(self.pump1_panel)
        pumps_layout.addWidget(self.pump2_panel)
        main.addLayout(pumps_layout)

        # Buttons
        g = QHBoxLayout()
        self.run_btn = QPushButton("Run")
        self.stop_btn = QPushButton("Stop")

        self.zero1_btn.clicked.connect(lambda: self.zero_sensor(1))
        self.zero2_btn.clicked.connect(lambda: self.zero_sensor(2))
        self.high1_btn.clicked.connect(lambda: self.high_sensor(1))
        self.high2_btn.clicked.connect(lambda: self.high_sensor(2))
        self.clear_cal_btn.clicked.connect(self.clear_calibration)

        g.addWidget(self.run_btn)
        g.addWidget(self.stop_btn)
        main.addLayout(g)
        
        ################################################# REVISED: Control Group Simplified
        # Re-add the control enable and deadband in a simplified group
        control_group = QGroupBox("Average-Based Feedback Control") 
        control_form = QFormLayout()
        
        # self.pid_enable and self.deadband are now attributes of ControlWindow
        control_form.addRow(self.pid_enable)
        control_form.addRow("Tolerance (mmHg):", self.deadband)
        #control_form.addRow(QLabel(f"Control runs every {int(DELAY_TIME)}s. Output: $\pm 1.0$ $\mu$L/min."))
        control_form.addRow(QLabel(f"Control runs every {int(DELAY_TIME)}s. Output: ± 10.0 µL/min."))
        
        control_group.setLayout(control_form)
        main.addWidget(control_group)
        ################################################# END REVISED

        # Log
        self.log = QTextEdit(); self.log.setReadOnly(True)
        main.addWidget(QLabel("Log / Status:"))
        main.addWidget(self.log)
        self.setLayout(main)

        # Set initial values for the new target pressure inputs
        self.pump1_panel.target_pressure.setValue(10.0)
        self.pump2_panel.target_pressure.setValue(10.0)

        # Per user's latest instruction: keep Initialize button visible but make it a no-op
        self.pump1_panel.init_btn.clicked.connect(lambda: self.initialize_pump(1))
        self.pump2_panel.init_btn.clicked.connect(lambda: self.initialize_pump(2))

        self.run_btn.clicked.connect(self.on_run_pressed)
        self.stop_btn.clicked.connect(self.on_stop_pressed)
        self.dispenser_dropdown.currentIndexChanged.connect(self.dispenser_changed.emit)
        # --- NEW: Connect Start Log Button ---
        self.start_log_btn.clicked.connect(self.on_start_log_pressed)

    def on_start_log_pressed(self):
        """Emits the signal to start logging with the current Donor ID."""
        donor_id = self.donor_id.text().strip()
        if not donor_id:
            self.log.append(f"[{time.strftime('%H:%M:%S')}] ERROR: Donor ID cannot be empty to start log.")
            return

        # Disable button until next experiment/reset to prevent multiple logs
        self.start_log_btn.setEnabled(False)
        self.log.append(f"[{time.strftime('%H:%M:%S')}] Attempting to start CSV log for Donor ID: **{donor_id}**")
        self.start_log_signal.emit(donor_id)


    def initialize_pump(self, pump_number):
        # method retained for compatibility but not connected to UI
        cmd = f"INIT,{pump_number},0,0"
        self.log.append(f"[{time.strftime('%H:%M:%S')}] INIT pump {pump_number} sent")
        self.run_command.emit(cmd)

    def on_run_pressed(self):
        flow = self.flow_rate.value()
        dia1 = self.pump1_panel.syringe_diameter.value()
        dia2 = self.pump2_panel.syringe_diameter.value()
        steps_sec1 = flow_ul_per_min_to_steps_per_sec(flow, dia1)
        steps_sec2 = flow_ul_per_min_to_steps_per_sec(flow, dia2)

        cmd1 = f"RUN,1,0,{steps_sec1:.6f}"
        cmd2 = f"RUN,2,0,{steps_sec2:.6f}"
        self.log.append(f"[{time.strftime('%H:%M:%S')}] RUN pump1 @ {flow:.6f} µL/min -> {steps_sec1:.6f} steps/s")
        self.log.append(f"[{time.strftime('%H:%M:%S')}] RUN pump2 @ {flow:.6f} µL/min -> {steps_sec2:.6f} steps/s")
        self.run_command.emit(cmd1)
        self.run_command.emit(cmd2)

    def on_stop_pressed(self):
        self.log.append(f"[{time.strftime('%H:%M:%S')}] STOP pumps")
        self.pid_enable.setChecked(False)
        self.run_command.emit("STOP,1,0,0")
        self.run_command.emit("STOP,2,0,0")

    def stop_all_for_calibration(self):
        """Stop pumps and disable feedback before calibration."""
        self.pid_enable.setChecked(False)
        self.run_command.emit("STOP,1,0,0")
        self.run_command.emit("STOP,2,0,0")

    def zero_sensor(self, sensor_number):
        self.stop_all_for_calibration()

        self.log.append(
            f"[{time.strftime('%H:%M:%S')}] ZERO sensor {sensor_number} sent. "
            "Make sure the line is vented to true 0 gauge pressure."
        )

        self.run_command.emit(f"ZERO,{sensor_number},0,0")

    def high_sensor(self, sensor_number):
        self.stop_all_for_calibration()

        high_pressure = self.high_cal_pressure.value()

        self.log.append(
            f"[{time.strftime('%H:%M:%S')}] HIGH calibration sensor {sensor_number} sent at "
            f"{high_pressure:.2f} mmHg. Make sure the sensor is actually at this pressure."
        )

        self.run_command.emit(f"HIGH,{sensor_number},0,{high_pressure:.2f}")

    def clear_calibration(self):
        self.stop_all_for_calibration()

        self.log.append(
            f"[{time.strftime('%H:%M:%S')}] CLEARCAL sent. Saved EEPROM calibration will be cleared."
        )

        self.run_command.emit("CLEARCAL,0,0,0")

# ---------------- CSVLogger (NEW CLASS) ----------------
class CSVLogger:
    """Handles writing the average pressure data to a CSV file."""
    def __init__(self, file_path: Path):
        self.file_path = file_path
        self.is_logging = False
        self.start_time = 0.0
        self.donor_id = ""
        self.log_count = 0

    def start_log(self, donor_id: str):
        """Initializes logging, clears the file, writes the header, and sets start time."""
        self.donor_id = donor_id
        self.start_time = time.time()
        self.log_count = 0 # Reset the count for the Time column
        self.is_logging = True
        
        # Define the exact column headers requested by the user
        self.headers = [
            "Donor ID", 
            "Time (min)",
            "channel 1 mean mmHg IOP", 
            "channel 2 mean mmHg ICP"
        ]

        try:
            # Overwrite the file for a new experiment
            with open(self.file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                # Write a blank row for separation
                writer.writerow([]) 
                # Write the header
                writer.writerow(self.headers)
            return f"Logging started. New data appended to **{self.file_path.name}**."
        except Exception as e:
            self.is_logging = False
            return f"ERROR starting CSV log: {e}"


    def log_data(self, s1_avg: float, s2_avg: float):
        """Appends a new row of data to the CSV file if logging is enabled."""
        if not self.is_logging:
            return

        self.log_count += 1
        
        # Calculate Time in minutes (incrementing by 1 min per log_count)
        time_min = self.log_count 
        
        # Format the data row
        row = [
            self.donor_id,
            time_min,
            f"{s1_avg:.2f}" if s1_avg == s1_avg else "NaN", # Handle NaN
            f"{s2_avg:.2f}" if s2_avg == s2_avg else "NaN"
        ]

        try:
            with open(self.file_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(row)
        except Exception as e:
            # Stop logging on error to prevent file corruption/crash
            self.is_logging = False
            return f"ERROR writing to CSV: {e}"

        return None # Return None on success

    def stop_log(self):
        """Stops the logging process."""
        self.is_logging = False
        self.donor_id = ""
        self.start_time = 0.0

# ---------------- SerialReader (QThread) and PressureGraph (integrated plotting) ----------------
PAIR_RE = re.compile(r"\(\s*([0-9]+)\s*,\s*([-+]?[0-9]*\.?[0-9]+)\s*\)")

class SerialReader(QtCore.QThread):
    # (omitting SerialReader class for brevity, assume it is unchanged)
    """QThread-based serial reader that emits dicts of {id: value}"""
    data_received = QtCore.pyqtSignal(dict)
    message_received = QtCore.pyqtSignal(str)
    error = QtCore.pyqtSignal(str)
    connected = QtCore.pyqtSignal(str)
    

    def __init__(self, port, baud, timeout=1.0, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._running = True
        self._ser = None
        self._send_lock = threading.Lock()

    def run(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            # allow Arduino reset
            time.sleep(1.2)
            self.connected.emit(f"Connected: {self.port} @ {self.baud}")
        except Exception as e:
            self.error.emit(f"Could not open serial port {self.port}: {e}")
            return

        while self._running:
            try:
                line = self._ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                pairs = PAIR_RE.findall(line)
                if pairs:
                    data = {}
                    for pid, val in pairs:
                        try:
                            data[int(pid)] = float(val)
                        except ValueError:
                            pass
                    if data:
                        self.data_received.emit(data)
                else:
                    self.message_received.emit(line)

            except Exception as e:
                self.error.emit(f"Serial read error: {e}")
                break

        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass

    def send(self, text):
        """Send text to the Arduino (thread-safe)."""
        with self._send_lock:
            try:
                if self._ser and self._ser.is_open:
                    if not text.endswith("\n"):
                        text += "\n"
                    self._ser.write(text.encode('utf-8'))
                else:
                    self.error.emit("Serial port not open for write")
            except Exception as e:
                self.error.emit(f"Serial write error: {e}")

    def stop(self):
        self._running = False
        self.wait(1000)

class PressureGraph(QWidget):
    """Widget to plot two sensors live (uses a SerialReader in main to feed on_serial_data)."""
    latest_values = {1: float('nan'), 2: float('nan')}
    latest_averages = {1: float('nan'), 2: float('nan')} # Expose latest averages

    # --- NEW: Accept csv_logger and donor_id_fn in constructor ---
    def __init__(self, control_window, csv_logger: 'CSVLogger'):
        super().__init__()
        self.setWindowTitle("Live Pressure Plot (Sensor 1 & 2)")
        self.control_window = control_window
        self.csv_logger = csv_logger # Store the logger instance
        
        # data buffers
        self.max_points = MAX_POINTS
        self.sample_index = 0
        self.x = deque(maxlen=self.max_points)
        self.avg_history = {
            1: deque(maxlen=5),
            2: deque(maxlen=5)
        }
        # per-sensor deques of values
        self.sensors = {1: deque(maxlen=self.max_points), 2: deque(maxlen=self.max_points)}
        # timestamps (parallel to x)
        self.times = deque(maxlen=self.max_points)

        # UI / plot
        self.plot_widget = pg.PlotWidget(title="Live Pressure (Sensor 1 & 2)")
        self.plot_widget.setBackground("w")
        self.plot_widget.addLegend()
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setLabel('left', 'Pressure (mmHg)')
        self.plot_widget.getAxis('left').enableAutoSIPrefix(False)
        self.plot_widget.setLabel('bottom', 'Time')

        self.curve1 = self.plot_widget.plot(pen=pg.mkPen('r', width=2), name="Sensor 1")
        self.curve2 = self.plot_widget.plot(pen=pg.mkPen('b', width=2, style=QtCore.Qt.DashLine), name="Sensor 2")

        layout = QVBoxLayout()
        layout.addWidget(self.plot_widget)
        self.setLayout(layout)

        # update timer (decoupled from serial reads)
        self.update_timer = QtCore.QTimer()
        self.update_timer.setInterval(UPDATE_INTERVAL_MS)
        self.update_timer.timeout.connect(self.update_plot)
        self.update_timer.start()

        # averaging
        self.last_aggregate_ts = time.time()

    @QtCore.pyqtSlot(dict)
    def on_serial_data(self, data_dict):
        """Accept dicts like {1:20.0, 2:12.0} from SerialReader."""
        now = time.time()
        self.sample_index += 1
        self.x.append(self.sample_index)
        self.times.append(now)
        for sid in (1, 2):
            if sid in data_dict:
                val = data_dict[sid]
                self.sensors[sid].append(val)
                PressureGraph.latest_values[sid] = val
            else:
                if len(self.sensors[sid]) > 0:
                    self.sensors[sid].append(self.sensors[sid][-1])
                else:
                    self.sensors[sid].append(float('nan'))

        # aggregation check
        if (now - self.last_aggregate_ts) >= DELAY_TIME:
            self.last_aggregate_ts = now
            self._aggregate_and_log(DELAY_TIME)

    def get_pressure_slope_per_min(self, sensor_id):
        hist = list(self.avg_history[sensor_id])

        if len(hist) < 2:
            return 0.0

        t_old, p_old = hist[-2]
        t_new, p_new = hist[-1]

        dt_min = (t_new - t_old) / 60.0

        if dt_min <= 0:
            return 0.0

        return (p_new - p_old) / dt_min

    @QtCore.pyqtSlot(str)
    def on_serial_error(self, msg):
        self.control_window.log.append(f"[SERIAL ERROR] {msg}")

    def _aggregate_and_log(self, window_seconds):
        t_now = time.time()
        start_t = t_now - window_seconds
        
        times_list = list(self.times)
        if not times_list:
            return
        
        first_idx = 0
        for i, ts in enumerate(times_list):
            if ts >= start_t:
                first_idx = i
                break
                
        per_sensor_avg = {}
        combined_vals = []
        for pid, buf in self.sensors.items():
            vals = list(buf)[first_idx:] if len(buf) > first_idx else []
            # Calculate average, ignoring NaNs
            valid_vals = [v for v in vals if not (v != v)]
            avg = sum(valid_vals) / len(valid_vals) if valid_vals else float('nan')
            per_sensor_avg[pid] = avg
            combined_vals.extend(valid_vals)

        combined_avg = sum(combined_vals)/len(combined_vals) if combined_vals else float('nan')
        
        # Store the averages for FlowController to use
        s1_avg = per_sensor_avg.get(1, float('nan'))
        s2_avg = per_sensor_avg.get(2, float('nan'))
        PressureGraph.latest_averages[1] = s1_avg
        PressureGraph.latest_averages[2] = s2_avg
        if s1_avg == s1_avg:
            self.avg_history[1].append((time.time(), s1_avg))

        if s2_avg == s2_avg:
            self.avg_history[2].append((time.time(), s2_avg))

        # Log to chat
        s1_avg_log = f"{s1_avg:.2f}" if s1_avg == s1_avg else "NaN"
        s2_avg_log = f"{s2_avg:.2f}" if s2_avg == s2_avg else "NaN"
        comb_avg_log = f"{combined_avg:.2f}" if combined_avg == combined_avg else "NaN"

        msg = (f"[{time.strftime('%H:%M:%S')}] Average over last {int(window_seconds)}s - "
               f"Sensor1: {s1_avg_log} mmHg, "
               f"Sensor2: {s2_avg_log} mmHg, "
               f"Combined: {comb_avg_log} mmHg")
        self.control_window.log.append(msg)
        print(msg)
        
        # --- NEW: Log to CSV ---
        csv_error = self.csv_logger.log_data(s1_avg, s2_avg)
        if csv_error:
            self.control_window.log.append(csv_error)

    def update_plot(self):
        x = list(self.x)
        if not x:
            return
            
        def pad_to_len(arr, target_len):
            if len(arr) < target_len:
                pad = [float('nan')] * (target_len - len(arr))
                return pad + arr
            return arr

        y1 = pad_to_len(list(self.sensors[1]), len(x))
        y2 = pad_to_len(list(self.sensors[2]), len(x))

        # --- NEW: Simple Moving Average filter for clean plotting ---
        #window_size = 20  # Smooths out 1 second of data (20 samples at 20Hz)
        window_size = 2  # Smooths out 1 second of data (20 samples at 2Hz)
        
        def smooth_data(data):
            smoothed = []
            for i in range(len(data)):
                # Get the last 20 points
                start_idx = max(0, i - window_size + 1)
                chunk = [val for val in data[start_idx:i+1] if val == val] # Excludes NaNs
                # Calculate the average of the chunk
                avg = sum(chunk) / len(chunk) if chunk else float('nan')
                smoothed.append(avg)
            return smoothed

        # Plot the smoothed data instead of the raw noisy data
        self.curve1.setData(x, smooth_data(y1))
        self.curve2.setData(x, smooth_data(y2))

        if len(x) < 30:
            self.plot_widget.enableAutoRange('y', True)


    def clear(self):
        # (omitting clear for brevity, assume it is unchanged)
        self.x.clear()
        self.times.clear()
        for s in self.sensors.values():
            s.clear()
        self.sample_index = 0
        self.curve1.clear()
        self.curve2.clear()


# Obsolete PID function removed

############################### REVISION: FlowController (Time-Averaged Incremental Control)
class FlowController(QtCore.QObject):
    # (omitting FlowController for brevity, assume it is unchanged)
    """
    Implements a simple time-averaged incremental controller with independent control
    for Pump 1 and Pump 2 based on their respective pressure averages.
    """
    def __init__(self, control_win: 'ControlWindow', reader: 'SerialReader', graph: 'PressureGraph'):
        super().__init__()
        self.control_win = control_win
        self.reader = reader
        self.graph = graph
        
        # Track the currently commanded flow rate for *each* pump
        initial_flow = float(self.control_win.flow_rate.value())
        self.current_flow_ul_per_min_1 = initial_flow 
        self.current_flow_ul_per_min_2 = initial_flow

        #Track number of attemps to regain pressure controls
        self.overpressure_attempts_1 = 0
        self.overpressure_attempts_2 = 0

        self.equalize_pause_until_1 = 0.0
        self.equalize_pause_until_2 = 0.0
        
        # Timer interval set to DELAY_TIME (the averaging window)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(int(DELAY_TIME * 1000))
        self.timer.timeout.connect(self._tick)

        # disengage control immediately when checkbox toggles
        self.control_win.pid_enable.toggled.connect(self._on_enable_toggle)

    def _on_enable_toggle(self, enabled):
        if enabled:
            initial_flow = float(self.control_win.flow_rate.value())
            # Set the starting flow rate for both pumps
            self.current_flow_ul_per_min_1 = initial_flow
            self.current_flow_ul_per_min_2 = initial_flow
            
            self.control_win.log.append(f"[{time.strftime('%H:%M:%S')}] Average-Based Feedback Control enabled. Starting flow (P1/P2): {initial_flow:.2f} µL/min.")
            
            # Immediately send the starting flow command for both pumps
            self._send_flow_command(1, self.current_flow_ul_per_min_1)
            self._send_flow_command(2, self.current_flow_ul_per_min_2)

        else:
            # stop pumps when disabling control
            self.reader.send("STOP,1,0,0")
            self.reader.send("STOP,2,0,0")
            self.control_win.log.append(f"[{time.strftime('%H:%M:%S')}] Feedback Control disabled → STOP all pumps")

    def start(self):
        self.timer.start()

    def start_pump_equalize_pause(self, pump_id, reason):
        pause_until = time.time() + EQUALIZE_PAUSE_SEC

        if pump_id == 1:
            self.equalize_pause_until_1 = pause_until
            self.current_flow_ul_per_min_1 = 0.0
            self.overpressure_attempts_1 = 0

        elif pump_id == 2:
            self.equalize_pause_until_2 = pause_until
            self.current_flow_ul_per_min_2 = 0.0
            self.overpressure_attempts_2 = 0

        self._send_flow_command(pump_id, 0.0)

        self.control_win.log.append(
            f"[{time.strftime('%H:%M:%S')}] PUMP {pump_id} EQUALIZATION PAUSE STARTED: "
            f"{reason} Pump {pump_id} stopped for 5 minutes."
        )
    def is_pump_paused(self, pump_id):
        now = time.time()

        if pump_id == 1:
            pause_until = self.equalize_pause_until_1
        else:
            pause_until = self.equalize_pause_until_2

        # No pause active
        if pause_until <= 0:
            return False

        # Pause still active
        if now < pause_until:
            remaining_min = (pause_until - now) / 60.0

            self._send_flow_command(pump_id, 0.0)

            self.control_win.log.append(
                f"[{time.strftime('%H:%M:%S')}] Pump {pump_id} equalizing. "
                f"Remaining: {remaining_min:.1f} min"
            )

            return True

        # Pause ended
        if pump_id == 1:
            self.equalize_pause_until_1 = 0.0
            self.current_flow_ul_per_min_1 = 0.0
            self.overpressure_attempts_1 = 0
        else:
            self.equalize_pause_until_2 = 0.0
            self.current_flow_ul_per_min_2 = 0.0
            self.overpressure_attempts_2 = 0

        self.control_win.log.append(
            f"[{time.strftime('%H:%M:%S')}] Pump {pump_id} equalization pause ended. "
            f"Pump {pump_id} resuming from 0 µL/min."
        )
        return False
    
    def _send_flow_command(self, pump_num, flow):
        """Helper to convert flow rate to steps/sec and send command for a specific pump."""
        if pump_num == 1:
            dia = self.control_win.pump1_panel.syringe_diameter.value()
        elif pump_num == 2:
            dia = self.control_win.pump2_panel.syringe_diameter.value()
        else:
            return 0.0

        steps_sec = flow_ul_per_min_to_steps_per_sec(flow, dia)
        self.reader.send(f"RUN,{pump_num},0,{steps_sec:.3f}")
        return steps_sec
        
    def _tick(self):
        if not self.control_win.pid_enable.isChecked():
            return

        # Read UI parameters
        sp1 = float(self.control_win.pump1_panel.target_pressure.value())
        sp2 = float(self.control_win.pump2_panel.target_pressure.value())
        deadband = float(self.control_win.deadband.value())
       #flow_change_mag = 1.0 # The fixed increment (1 uL/min)
        #flow_change_mag = 10.0 # The fixed increment (10 uL/min)

        # Use the latest AVERAGE as the Process Variable (PV)
        pv1_avg = PressureGraph.latest_averages.get(1, float('nan'))
        pv2_avg = PressureGraph.latest_averages.get(2, float('nan'))

        
        # Check for initial data availability
        if pv1_avg != pv1_avg or pv2_avg != pv2_avg: 
            self.control_win.log.append(f"[{time.strftime('%H:%M:%S')}] Waiting for valid average pressure value data...")
            return
        
        slope1 = self.graph.get_pressure_slope_per_min(1)
        slope2 = self.graph.get_pressure_slope_per_min(2)
        predicted_pv1 = pv1_avg + slope1 * PREDICT_LOOKAHEAD_MIN
        predicted_pv2 = pv2_avg + slope2 * PREDICT_LOOKAHEAD_MIN

        pump1_paused = self.is_pump_paused(1)
        pump2_paused = self.is_pump_paused(2)

        self.control_win.log.append(
            f"[{time.strftime('%H:%M:%S')}] CONTROL CHECK → "
            f"P1 avg={pv1_avg:.2f}, slope={slope1:+.2f} mmHg/min, predicted={predicted_pv1:.2f}, target={sp1:.2f} | "
            f"P2 avg={pv2_avg:.2f}, slope={slope2:+.2f} mmHg/min, predicted={predicted_pv2:.2f}, target={sp2:.2f}"
        )

        

        # --- Independent Control for Pump 1 ---
        error1 = sp1 - pv1_avg
        delta_flow_1 = 0.0
        
        # --- Independent Control for Pump 1 ---

        if pump1_paused:
            new_flow_1 = 0.0

        else:
            if pv1_avg > sp1 + FINE_CONTROL_BAND_MMHG:
                # Already too high: reduce aggressively
                delta_flow_1 = -HIGH_PRESSURE_DROP_STEP

                if slope1 >= 0:
                    self.overpressure_attempts_1 += 1
                else:
                    self.overpressure_attempts_1 = 0

            elif predicted_pv1 > sp1 + FINE_CONTROL_BAND_MMHG:
                # Not too high yet, but predicted to overshoot soon
                delta_flow_1 = -HIGH_PRESSURE_DROP_STEP / 2.0
                self.overpressure_attempts_1 = 0

            elif abs(error1) <= deadband:
                # Very close to target: hold flowrate
                delta_flow_1 = 0.0
                self.overpressure_attempts_1 = 0
            
            elif abs(error1) <= FINE_CONTROL_BAND_MMHG:
                #Within +/- 5mmHg: fine-tune gently
                self.overpressure_attempts_1 = 0

                if error1> deadband:
                    if slope1 < FINE_SLOPE_LIMIT_MMHG_PER_MIN:
                        delta_flow_1 = FINE_CONTROL_STEP
                    else:
                        delta_flow_1 = 0.0

                elif error1 < -deadband:
                    # Pressure is slightly above target
                    if slope1 > -FINE_SLOPE_LIMIT_MMHG_PER_MIN:
                        delta_flow_1 = -FINE_CONTROL_STEP
                    else:
                        delta_flow_1 = 0.0
                        

            
            elif error1 > FINE_CONTROL_BAND_MMHG and slope1 < SLOPE_LIMIT_MMHG_PER_MIN:
                # More than 5mmHg below target and not rising too fast
                delta_flow_1 = CONTROL_FLOW_STEP
                self.overpressure_attempts_1 = 0

            elif error1 > FINE_CONTROL_BAND_MMHG and slope1 >= SLOPE_LIMIT_MMHG_PER_MIN:
                # More than 5 mmHg below target but rising quickly
                delta_flow_1 = 0.0
                self.overpressure_attempts_1 = 0
            else:
                # Fallback safety hold
                delta_flow_1 = 0.0
                self.overpressure_attempts_1 = 0

            if self.overpressure_attempts_1 >= MAX_RECOVERY_ATTEMPTS_1:
                self.start_pump_equalize_pause( 1,"Pressure stayed high and continued rising despite recovery attempts.")
                new_flow_1 = 0.0
            else:
                new_flow_1 = max(0.0, self.current_flow_ul_per_min_1 + delta_flow_1)
       
        # --- Independent Control for Pump 2 ---
        error2 = sp2 - pv2_avg
        delta_flow_2 = 0.0

        # --- Independent Control for Pump 2 ---
        if pump2_paused:
            new_flow_2 = 0.0
        
        else:
            if pv2_avg > sp2 + FINE_CONTROL_BAND_MMHG:
                delta_flow_2 = -HIGH_PRESSURE_DROP_STEP
                
                if slope2 >= 0:
                    self.overpressure_attempts_2 += 1
                else:
                    self.overpressure_attempts_2 = 0


            elif predicted_pv2 > sp2 + FINE_CONTROL_BAND_MMHG:
                delta_flow_2 = -HIGH_PRESSURE_DROP_STEP / 2.0
                self.overpressure_attempts_2 = 0


            elif abs(error2) <= deadband:
                delta_flow_2 = 0.0
                self.overpressure_attempts_2 = 0

            elif abs(error2) <= FINE_CONTROL_BAND_MMHG:
                self.overpressure_attempts_2 = 0

                if error2 > deadband:
                    if slope2 < FINE_SLOPE_LIMIT_MMHG_PER_MIN:
                        delta_flow_2 = FINE_CONTROL_STEP
                    else:
                        delta_flow_2 = 0.0

                elif error2 < -deadband:
                    if slope2 > -FINE_SLOPE_LIMIT_MMHG_PER_MIN:
                        delta_flow_2 = -FINE_CONTROL_STEP
                    else:
                        delta_flow_2 = 0.0


            elif error2 > FINE_CONTROL_BAND_MMHG and slope2 < SLOPE_LIMIT_MMHG_PER_MIN:
                delta_flow_2 = CONTROL_FLOW_STEP
                self.overpressure_attempts_2 = 0


            elif error2 > FINE_CONTROL_BAND_MMHG and slope2 >= SLOPE_LIMIT_MMHG_PER_MIN:
                delta_flow_2 = 0.0
                self.overpressure_attempts_2 = 0
            else:
                #Fall back safety hold
                delta_flow_2 = 0.0
                self.overpressure_attempts_2 = 0

            if self.overpressure_attempts_2 >= MAX_RECOVERY_ATTEMPTS_2:
                self.start_pump_equalize_pause( 2, "Pressure stayed high and continued rising despite recovery attempts." )
                new_flow_2 = 0.0
            else:
                new_flow_2 = max(0.0, self.current_flow_ul_per_min_2 + delta_flow_2)


        # ============================================================
        # Send Pump 1 command/log once
        # ============================================================
        if abs(new_flow_1 - self.current_flow_ul_per_min_1) > 0.01:
            old_flow_1 = self.current_flow_ul_per_min_1
            self.current_flow_ul_per_min_1 = new_flow_1
            steps_sec_1 = self._send_flow_command(1, self.current_flow_ul_per_min_1)

            self.control_win.log.append(
                f"[{time.strftime('%H:%M:%S')}] PUMP 1 UPDATE (Avg PV) → "
                f"Flow change: Δ {new_flow_1 - old_flow_1:+.1f} µL/min"
                f" → New Flow: {new_flow_1:.2f} µL/min ({steps_sec_1:.3f} steps/s)"
            )
        else:
            self.control_win.log.append(
                f"[{time.strftime('%H:%M:%S')}] PUMP 1 (Avg PV) → "
                f"Flow maintained at {self.current_flow_ul_per_min_1:.2f} µL/min"
            )

        # ============================================================
        # Send Pump 2 command/log once
        # ============================================================
        if abs(new_flow_2 - self.current_flow_ul_per_min_2) > 0.01:
            old_flow_2 = self.current_flow_ul_per_min_2
            self.current_flow_ul_per_min_2 = new_flow_2
            steps_sec_2 = self._send_flow_command(2, self.current_flow_ul_per_min_2)

            self.control_win.log.append(
                f"[{time.strftime('%H:%M:%S')}] PUMP 2 UPDATE (Avg PV) → "
                f"Flow change: Δ {new_flow_2 - old_flow_2:+.1f} µL/min"
                f" → New Flow: {new_flow_2:.2f} µL/min ({steps_sec_2:.3f} steps/s)"
            )
        else:
            self.control_win.log.append(
                f"[{time.strftime('%H:%M:%S')}] PUMP 2 (Avg PV) → "
                f"Flow maintained at {self.current_flow_ul_per_min_2:.2f} µL/min"
            )

        #7/21 intital pressure code
        #if abs(error2) <= deadband:
        #    delta_flow_2 = 0.0 #PV within tolerance, keep at flowrate
        #elif error2 > deadband:
        #    delta_flow_2 = flow_change_mag #PV  too low: increase slowly
        #elif error2 < -deadband:
        #    delta_flow_2 = -2.0 * flow_change_mag
        #new_flow_2 = max(0.0, self.current_flow_ul_per_min_2 + delta_flow_2)



# ---------------- Main ----------------
def main():
    app = QApplication(sys.argv)
    control_win = ControlWindow()
    
    # --- NEW: Instantiate CSV Logger ---
    csv_logger = CSVLogger(CSV_FILE_PATH)
    
    # --- NEW: Pass logger to PressureGraph ---
    graph_win = PressureGraph(control_win, csv_logger)

    # SerialReader instance 
    port_to_use = autodetect_port(PORT)
    if not port_to_use:
        control_win.log.append(f"[{time.strftime('%H:%M:%S')}] ERROR: Could not autodetect serial port. Using default: {PORT}")
        port_to_use = PORT

    reader = SerialReader(port_to_use, BAUDRATE, timeout=SERIAL_TIMEOUT)
    # connect signals
    reader.data_received.connect(graph_win.on_serial_data)
    reader.error.connect(graph_win.on_serial_error)
    reader.connected.connect(lambda txt: control_win.log.append(f"[{time.strftime('%H:%M:%S')}] {txt}"))
    reader.error.connect(lambda txt: control_win.log.append(f"[{time.strftime('%H:%M:%S')}] {txt}"))
    reader.message_received.connect(lambda txt: control_win.log.append(f"[{time.strftime('%H:%M:%S')}] [ARDUINO] {txt}"))
    reader.start()

    # wire GUI commands to the reader.send method
    control_win.run_command.connect(reader.send)
    
    # --- NEW: Wire the Start Log signal to the CSVLogger ---
    def start_csv_log_handler(donor_id):
        result = csv_logger.start_log(donor_id)
        control_win.log.append(f"[{time.strftime('%H:%M:%S')}] {result}")
    
    control_win.start_log_signal.connect(start_csv_log_handler)


    ### Instantiate and start the FlowController
    flow_mgr = FlowController(control_win, reader, graph_win)
    flow_mgr.start()

    control_win.show()
    graph_win.show()

    def on_exit():
        try:
            reader.stop()
            # Optional: ensure logging stops on exit
            csv_logger.stop_log() 
        except Exception:
            pass

    app.aboutToQuit.connect(on_exit)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

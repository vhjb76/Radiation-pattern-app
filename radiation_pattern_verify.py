import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import queue
import socket
import re
import time
import csv
import math

# Safe import for serial; application falls back gracefully if not yet installed
try:
    import serial
except ImportError:
    serial = None


# ============================================================================
# 1. HPBW & RF METRICS CALCULATION ENGINE
# ============================================================================

_SPEED_OF_LIGHT_M_S = 299_792_458.0


def calculate_fspl_db(distance_m: float, freq_hz: float) -> float:
    """Free-space path loss (Friis) in dB for a given TX-RX distance and frequency."""
    return 20.0 * math.log10(4.0 * math.pi * distance_m * freq_hz / _SPEED_OF_LIGHT_M_S)


def calculate_rf_metrics(measured_points):
    """
    Computes Peak Power, 3 dB Half-Power Beamwidth (HPBW), and Front-to-Back (F/B) Ratio.
    Uses circular linear interpolation across discrete angle steps.
    
    :param measured_points: List of (angle_deg, power_dbm) tuples.
    :return: Dictionary of metrics or None if insufficient data.
    """
    if len(measured_points) < 8:
        return None

    angles = [pt[0] for pt in measured_points]
    powers = [pt[1] for pt in measured_points]
    n = len(powers)

    # 1. Identify Peak Power and its angle
    peak_power = max(powers)
    peak_idx = powers.index(peak_power)
    peak_angle = angles[peak_idx]
    target_3db = peak_power - 3.0

    # Helper: Angular linear interpolation between two adjacent samples
    def interpolate_angle(a1, p1, a2, p2, p_tgt):
        diff_a = (a2 - a1) % 360.0
        if diff_a > 180.0:
            diff_a -= 360.0
        frac = (p_tgt - p1) / (p2 - p1) if (p2 != p1) else 0.0
        return (a1 + frac * diff_a) % 360.0

    # 2. Search Counter-Clockwise (Left) from peak for -3 dB crossing
    left_angle = None
    for step in range(1, n // 2 + 1):
        idx_prev = (peak_idx - (step - 1)) % n
        idx_curr = (peak_idx - step) % n
        p_prev = powers[idx_prev]
        p_curr = powers[idx_curr]
        if p_prev >= target_3db and p_curr < target_3db:
            left_angle = interpolate_angle(angles[idx_prev], p_prev, angles[idx_curr], p_curr, target_3db)
            break

    # 3. Search Clockwise (Right) from peak for -3 dB crossing
    right_angle = None
    for step in range(1, n // 2 + 1):
        idx_prev = (peak_idx + (step - 1)) % n
        idx_curr = (peak_idx + step) % n
        p_prev = powers[idx_prev]
        p_curr = powers[idx_curr]
        if p_prev >= target_3db and p_curr < target_3db:
            right_angle = interpolate_angle(angles[idx_prev], p_prev, angles[idx_curr], p_curr, target_3db)
            break

    # 4. Compute Front-to-Back (F/B) Ratio (opposite side @ peak_angle + 180°)
    back_target_angle = (peak_angle + 180.0) % 360.0
    # Find closest measured sample to back angle
    closest_back_idx = min(range(n), key=lambda i: min(abs(angles[i] - back_target_angle), 
                                                        360.0 - abs(angles[i] - back_target_angle)))
    back_power = powers[closest_back_idx]
    fb_ratio = peak_power - back_power

    # 5. Calculate HPBW
    hpbw = None
    if left_angle is not None and right_angle is not None:
        hpbw = (right_angle - left_angle) % 360.0

    return {
        "peak_power": peak_power,
        "peak_angle": peak_angle,
        "target_3db": target_3db,
        "left_angle": left_angle,
        "right_angle": right_angle,
        "hpbw_deg": hpbw,
        "back_power": back_power,
        "fb_ratio_db": fb_ratio
    }


# ============================================================================
# 2. INSTRUMENT DRIVERS (Zero-Admin Required)
# ============================================================================

class FieldFoxSocket:
    """Keysight FieldFox raw TCP socket SCPI driver (Port 5025)."""
    def __init__(self, ip: str, port: int = 5025, timeout: float = 4.0):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.timeout)
        try:
            sock.connect((self.ip, self.port))
        except OSError as exc:
            sock.close()
            raise ConnectionError(f"Could not reach FieldFox at {self.ip}:{self.port}: {exc}") from exc
        self.sock = sock
        return self.query("*IDN?")

    def send_cmd(self, cmd: str):
        msg = (cmd.strip() + "\n").encode("ascii")
        self.sock.sendall(msg)

    def query(self, cmd: str) -> str:
        self.send_cmd(cmd)
        resp = b""
        while True:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("FieldFox closed the connection")
            resp += chunk
            if b"\n" in chunk:
                break
        return resp.decode("ascii").strip()

    def setup_cpm(self, center_freq_hz: float, span_hz: float = 100e3):
        """Initializes Channel Power Meter (Option 310). Returns any SCPI errors raised along the way."""
        self.drain_errors()  # flush stale errors left over from earlier sessions before attributing new ones
        self.send_cmd('INST:SEL "CPM"')
        time.sleep(0.3)
        self.send_cmd(f"SENS:FREQ:CENT {center_freq_hz}")
        self.send_cmd(f"SENS:FREQ:SPAN {span_hz}")
        self.send_cmd("AVER:COUN 4")
        self.send_cmd("AVER:STAT ON")
        time.sleep(0.2)
        return self.drain_errors()

    def drain_errors(self, limit: int = 10) -> list:
        """Reads the SCPI error queue until empty (or limit reached); returns non-"No error" entries."""
        errors = []
        for _ in range(limit):
            entry = self.query("SYST:ERR?")
            if entry.startswith("+0,") or entry.startswith("0,") or "No error" in entry:
                break
            errors.append(entry)
        return errors

    def read_power(self) -> float:
        """Query scalar power reading in dBm."""
        val_str = self.query("TRAC:DATA?")
        return float(val_str.split(",")[0])

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None


_RESPONSE_VALUE_RE = re.compile(r"^#\d+:[^=]+=(.*)$")
_NUMBER_RE = re.compile(r"-?\d+")


class ArcusIMXMotor:
    """Serial interface for the Arcus TITAN-IMX, using the confirmed TITAN-ASCII framing."""
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0, device_id: int = 1,
                 speed: float = 5000.0, acceleration: float = 1000.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.device_id = device_id
        self.speed = speed
        self.acceleration = acceleration
        self.ser = None
        self._position = 0

    def connect(self):
        if serial is None:
            raise RuntimeError("pyserial module not found. Run 'pip install pyserial' or check environment.")
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        # Fault-clear + enable handshake required before any motion is accepted.
        self.send_cmd("ECLEARX")
        self.send_cmd("SVON")
        # A move with speed/accel left at 0 silently does nothing, so seed sane defaults.
        self.send_cmd(f"HSPD={self.speed}")
        self.send_cmd(f"ACC={self.acceleration}")

    def send_cmd(self, cmd: str) -> str:
        frame = f"@{self.device_id:02d}:{cmd}"
        msg = (frame + "\r\n").encode("ascii")
        self.ser.write(msg)
        self.ser.flush()
        raw = self.ser.readline().decode("ascii", errors="replace").strip()
        match = _RESPONSE_VALUE_RE.match(raw)
        return match.group(1) if match else raw

    def step_relative(self, steps: int, axis: str = "X") -> str:
        if axis != "X":
            raise ValueError(f"Only the X axis is confirmed against real hardware, got: {axis!r}")
        # DX= (relative move) is unconfirmed on real hardware; X= (absolute) is confirmed working,
        # so relative stepping is emulated by tracking position and issuing absolute moves.
        self._position += steps
        return self.send_cmd(f"X={self._position}")

    def move_absolute(self, position: int) -> str:
        self._position = position
        return self.send_cmd(f"X={position}")

    def read_position(self) -> str:
        return self.send_cmd("EX")

    def sync_position(self) -> int:
        """Aligns our tracked position with the real encoder reading, without redefining 0°."""
        match = _NUMBER_RE.search(self.read_position())
        self._position = int(match.group()) if match else 0
        return self._position

    def home(self, direction: int = 1) -> str:
        """Hardware homing (seeks a home/limit switch) - unconfirmed, may error depending on firmware."""
        return self.send_cmd(f"HOMEX{'+' if direction >= 0 else '-'}")

    def zero_position(self) -> str:
        """Defines the current physical position as encoder 0 (no motion) and resets tracked position to match."""
        resp = self.send_cmd("EX=0")
        self._position = 0
        return resp

    def is_moving(self) -> bool:
        """Query MST (Motion Status). Bit layout is unconfirmed, so treat any '1' as moving."""
        res = self.send_cmd("MST")
        return "1" in res

    def stop(self):
        self.send_cmd("STOPX")

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                self.send_cmd("STOPX")
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None


# ============================================================================
# 3. BACKGROUND SWEEP WORKER THREAD
# ============================================================================

def _home_motor_worker(motor_port: str, motor_velocity: float, motor_acceleration: float, msg_queue: queue.Queue):
    """Standalone worker that connects, moves to absolute position 0, and disconnects."""
    motor = None
    try:
        msg_queue.put(("LOG", f"Homing: connecting to Arcus IMX on {motor_port}..."))
        motor = ArcusIMXMotor(port=motor_port, baudrate=115200, speed=motor_velocity, acceleration=motor_acceleration)
        motor.connect()
        resp = motor.move_absolute(0)
        msg_queue.put(("LOG", f"Homing: move to 0° resp={resp!r}"))
        while motor.is_moving():
            time.sleep(0.04)
        msg_queue.put(("LOG", f"Homing complete. encoder={motor.read_position()!r}"))
    except Exception as exc:
        msg_queue.put(("LOG", f"Homing failed: {exc}"))
    finally:
        if motor:
            motor.close()
        msg_queue.put(("HOME_DONE", None))


def sweep_worker(cfg: dict, msg_queue: queue.Queue, stop_event: threading.Event):
    """Executes motion, settle, acquisition, and logging on a separate thread."""
    ff = None
    motor = None
    csv_file = None

    try:
        msg_queue.put(("STATUS", "Connecting to instruments..."))
        msg_queue.put(("LOG", f"Sweep cut type: {cfg['cut_type']}"))

        # 1. Establish hardware connections or start simulation
        if not cfg["simulate"]:
            ff = FieldFoxSocket(ip=cfg["ff_ip"], port=cfg["ff_port"])
            idn = ff.connect()
            msg_queue.put(("LOG", f"FieldFox: {idn}"))
            if cfg["ff_bypass_config"]:
                msg_queue.put(("LOG", "Bypassing remote config - using FieldFox's current front-panel settings."))
            else:
                cpm_errors = ff.setup_cpm(center_freq_hz=cfg["center_freq_mhz"] * 1e6, span_hz=cfg["span_khz"] * 1e3)
                for err in cpm_errors:
                    msg_queue.put(("LOG", f"FieldFox SCPI error during CPM setup: {err}"))

            motor = ArcusIMXMotor(port=cfg["motor_port"], baudrate=cfg["motor_baud"],
                                   speed=cfg["motor_velocity"], acceleration=cfg["motor_acceleration"])
            motor.connect()
            msg_queue.put(("LOG", f"Arcus IMX connected on {cfg['motor_port']}"))

            if cfg["zero_at_start"]:
                # No homing search is performed - the antenna's current physical position becomes 0°.
                zero_resp = motor.zero_position()
                msg_queue.put(("LOG", f"Set current position as 0°. resp={zero_resp!r} encoder={motor.read_position()!r}"))
            else:
                synced_position = motor.sync_position()
                msg_queue.put(("LOG", f"Using existing motor position as 0° reference. encoder={synced_position}"))
        else:
            msg_queue.put(("LOG", "[SIMULATION MODE ENABLED] Generating cardioid RF pattern..."))

        # Path loss compensation (Friis): EIRP = Pr_measured - Gr + FSPL
        if cfg["pathloss_enabled"]:
            fspl_db = calculate_fspl_db(cfg["distance_m"], cfg["center_freq_mhz"] * 1e6)
            pathloss_offset_db = fspl_db - cfg["rx_gain_dbi"]
            msg_queue.put(("LOG", f"Path loss compensation enabled: FSPL={fspl_db:.2f} dB, "
                                  f"RX Gain={cfg['rx_gain_dbi']:.2f} dBi, Offset={pathloss_offset_db:+.2f} dB"))
        else:
            pathloss_offset_db = 0.0

        # 2. Prepare CSV Logging
        csv_file = open(cfg["csv_path"], mode="w", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        writer.writerow([f"# Cut Type: {cfg['cut_type']}"])
        writer.writerow([f"# Path Loss Compensation Offset (dB): {pathloss_offset_db:.2f}"])
        writer.writerow(["Angle_deg", "Power_dBm", "Power_dBm_Raw", "Timestamp"])
        csv_file.flush()

        current_angle = 0.0
        step_angle = cfg["step_angle"]
        total_angle = cfg["total_angle"]
        # A full 360° sweep re-measures the same physical position at 0 and 360 - drop the duplicate endpoint.
        is_full_circle = total_angle > 0 and abs(total_angle % 360.0) < 1e-6
        sweep_limit = (total_angle - step_angle) if is_full_circle else total_angle
        steps_per_increment = int(step_angle * cfg["steps_per_deg"])
        settling_time = max(cfg["settle_time_s"], 5.0)

        msg_queue.put(("STATUS", "Sweep running..."))

        # 3. Main Step & Acquire Loop
        while current_angle <= sweep_limit + 1e-6 and not stop_event.is_set():
            # A. Settle wait to avoid antenna vibration / cable flex errors
            time.sleep(settling_time)

            # B. Acquire RF Power
            if not cfg["simulate"]:
                raw_power_dbm = ff.read_power()
            else:
                # Simulated UHF RFID Cardioid beam centered at 0° with side lobes
                rad = math.radians(current_angle)
                sim_base = -15.0 + 14.0 * math.cos(rad) + 2.0 * math.cos(3 * rad)
                raw_power_dbm = sim_base + (time.time() % 0.3 - 0.15)
            power_dbm = raw_power_dbm + pathloss_offset_db

            # C. Record & Queue to GUI
            timestamp_str = time.strftime("%H:%M:%S")
            writer.writerow([f"{current_angle:.1f}", f"{power_dbm:.2f}", f"{raw_power_dbm:.2f}", timestamp_str])
            csv_file.flush()

            msg_queue.put(("DATA", current_angle, power_dbm))
            msg_queue.put(("LOG", f"Angle {current_angle:5.1f}° | Power: {power_dbm:6.2f} dBm"))

            if current_angle >= sweep_limit - 1e-6 or stop_event.is_set():
                break

            # D. Advance Stepper Motor
            if not cfg["simulate"]:
                move_resp = motor.step_relative(steps=steps_per_increment)
                msg_queue.put(("LOG", f"Motor move -> target={motor._position} resp={move_resp!r}"))
                # Brief delay so the controller's MST register reflects the move before we start polling it
                time.sleep(0.1)
                # Wait until mechanical motion is fully completed
                while motor.is_moving():
                    if stop_event.is_set():
                        motor.stop()
                        break
                    time.sleep(0.04)
                # Read the encoder only after motion settles - reading it right after the move
                # command just echoes the pre-move value since the axis hasn't moved yet.
                msg_queue.put(("LOG", f"Motor settled -> encoder={motor.read_position()!r}"))
            else:
                time.sleep(0.08)

            current_angle += step_angle

        if stop_event.is_set():
            msg_queue.put(("STATUS", "Sweep aborted by user."))
            msg_queue.put(("LOG", "Sweep aborted."))
        else:
            msg_queue.put(("STATUS", "Sweep completed successfully!"))
            msg_queue.put(("FINISHED", None))

    except Exception as exc:
        msg_queue.put(("ERROR", str(exc)))
    finally:
        if ff:
            ff.close()
        if motor:
            motor.close()
        if csv_file:
            csv_file.close()


# ============================================================================
# 4. TKINTER GUI APPLICATION WITH INTEGRATED POLAR & HPBW DISPLAY
# ============================================================================

class RadiationPatternApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("RFID UHF Radiation Pattern & HPBW Analyzer")
        self.geometry("1180x780")
        self.minsize(1020, 680)

        self.msg_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.worker_thread = None
        self.measured_points = []  # List of (angle_deg, power_dbm)
        self.current_cut_type = "Phi Cut (\u03c6)"
        self.is_full_circle_sweep = False

        self._setup_style()
        self._build_ui()
        self._after_poll_queue()

    def _setup_style(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        self.configure(bg="#f4f4f6")

    def _build_ui(self):
        main_frame = ttk.Frame(self, padding=12)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Left Column: Configuration, Controls, & Metrics
        left_col = ttk.Frame(main_frame, width=400)
        left_col.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 12))

        # Right Column: Live Polar Canvas
        right_col = ttk.Frame(main_frame)
        right_col.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # ----------------- Left: Hardware Setup -----------------
        cfg_box = ttk.LabelFrame(left_col, text=" Configuration ", padding=10)
        cfg_box.pack(fill=tk.X, pady=(0, 8))

        self.sim_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(cfg_box, text="Simulation Mode (Test without Hardware)", variable=self.sim_var).grid(
            row=0, column=0, columnspan=2, sticky=tk.W, pady=(0, 6)
        )

        self.ff_bypass_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            cfg_box, text="Bypass Remote Config (use FieldFox front-panel settings)", variable=self.ff_bypass_var
        ).grid(row=0, column=2, columnspan=2, sticky=tk.W, padx=(8, 0), pady=(0, 6))

        ttk.Label(cfg_box, text="FieldFox IP:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.entry_ip = ttk.Entry(cfg_box)
        self.entry_ip.insert(0, "192.168.1.10")
        self.entry_ip.grid(row=1, column=1, sticky=tk.EW, pady=2)

        ttk.Label(cfg_box, text="Port:").grid(row=1, column=2, sticky=tk.W, padx=(8, 2), pady=2)
        self.entry_ff_port = ttk.Entry(cfg_box, width=6)
        self.entry_ff_port.insert(0, "5025")
        self.entry_ff_port.grid(row=1, column=3, sticky=tk.W, pady=2)

        ttk.Label(cfg_box, text="Carrier Freq (MHz):").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.entry_freq = ttk.Entry(cfg_box)
        self.entry_freq.insert(0, "866.3")
        self.entry_freq.grid(row=2, column=1, sticky=tk.EW, pady=2)

        ttk.Label(cfg_box, text="Cut Type:").grid(row=2, column=2, sticky=tk.W, padx=(8, 2), pady=2)
        self.cut_type_var = tk.StringVar(value="Phi Cut (\u03c6)")
        self.combo_cut_type = ttk.Combobox(
            cfg_box, textvariable=self.cut_type_var, state="readonly", width=14,
            values=["Phi Cut (\u03c6)", "Theta Cut (\u03b8)"],
        )
        self.combo_cut_type.grid(row=2, column=3, sticky=tk.W, pady=2)

        ttk.Label(cfg_box, text="Span (kHz):").grid(row=3, column=0, sticky=tk.W, pady=2)
        self.entry_span = ttk.Entry(cfg_box)
        self.entry_span.insert(0, "100")
        self.entry_span.grid(row=3, column=1, sticky=tk.EW, pady=2)

        ttk.Label(cfg_box, text="Motor COM Port:").grid(row=4, column=0, sticky=tk.W, pady=2)
        self.entry_port = ttk.Entry(cfg_box)
        self.entry_port.insert(0, "COM16")
        self.entry_port.grid(row=4, column=1, sticky=tk.EW, pady=2)

        self.zero_home_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            cfg_box, text="Set Current Position as 0° (no homing search)", variable=self.zero_home_var
        ).grid(row=4, column=2, columnspan=2, sticky=tk.W, padx=(8, 0), pady=2)

        ttk.Label(cfg_box, text="Motor Velocity:").grid(row=5, column=0, sticky=tk.W, pady=2)
        self.entry_velocity = ttk.Entry(cfg_box)
        self.entry_velocity.insert(0, "10")
        self.entry_velocity.grid(row=5, column=1, sticky=tk.EW, pady=2)

        ttk.Label(cfg_box, text="Motor Acceleration:").grid(row=6, column=0, sticky=tk.W, pady=2)
        self.entry_acceleration = ttk.Entry(cfg_box)
        self.entry_acceleration.insert(0, "10")
        self.entry_acceleration.grid(row=6, column=1, sticky=tk.EW, pady=2)

        ttk.Label(cfg_box, text="Steps / Degree:").grid(row=7, column=0, sticky=tk.W, pady=2)
        self.entry_steps_deg = ttk.Entry(cfg_box)
        self.entry_steps_deg.insert(0, "80")
        self.entry_steps_deg.grid(row=7, column=1, sticky=tk.EW, pady=2)

        ttk.Label(cfg_box, text="Step Angle (°):").grid(row=8, column=0, sticky=tk.W, pady=2)
        self.entry_step_deg = ttk.Entry(cfg_box)
        self.entry_step_deg.insert(0, "5.0")
        self.entry_step_deg.grid(row=8, column=1, sticky=tk.EW, pady=2)

        ttk.Label(cfg_box, text="Total Angle (°):").grid(row=9, column=0, sticky=tk.W, pady=2)
        self.entry_total_deg = ttk.Entry(cfg_box)
        self.entry_total_deg.insert(0, "360.0")
        self.entry_total_deg.grid(row=9, column=1, sticky=tk.EW, pady=2)

        ttk.Label(cfg_box, text="Settle Time (s):").grid(row=10, column=0, sticky=tk.W, pady=2)
        self.entry_settle = ttk.Entry(cfg_box)
        self.entry_settle.insert(0, "5.0")
        self.entry_settle.grid(row=10, column=1, sticky=tk.EW, pady=2)

        ttk.Label(cfg_box, text="Output CSV:").grid(row=11, column=0, sticky=tk.W, pady=2)
        self.entry_csv = ttk.Entry(cfg_box)
        self.entry_csv.insert(0, "rfid_radiation_pattern.csv")
        self.entry_csv.grid(row=11, column=1, sticky=tk.EW, pady=2)

        # ----------------- Left: Path Loss Compensation (Friis) -----------------
        pathloss_box = ttk.LabelFrame(left_col, text=" Path Loss Compensation (Friis) ", padding=10)
        pathloss_box.pack(fill=tk.X, pady=(0, 8))

        self.pathloss_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(pathloss_box, text="Apply Path Loss Compensation", variable=self.pathloss_var).grid(
            row=0, column=0, columnspan=2, sticky=tk.W, pady=(0, 6)
        )

        ttk.Label(pathloss_box, text="TX-RX Distance (m):").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.entry_distance_m = ttk.Entry(pathloss_box)
        self.entry_distance_m.insert(0, "3.0")
        self.entry_distance_m.grid(row=1, column=1, sticky=tk.EW, pady=2)

        ttk.Label(pathloss_box, text="RX Antenna Gain (dBi):").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.entry_rx_gain_dbi = ttk.Entry(pathloss_box)
        self.entry_rx_gain_dbi.insert(0, "6.0")
        self.entry_rx_gain_dbi.grid(row=2, column=1, sticky=tk.EW, pady=2)

        ttk.Label(pathloss_box, text="TX Power (dBm):").grid(row=3, column=0, sticky=tk.W, pady=2)
        self.entry_tx_power_dbm = ttk.Entry(pathloss_box)
        self.entry_tx_power_dbm.insert(0, "30.0")
        self.entry_tx_power_dbm.grid(row=3, column=1, sticky=tk.EW, pady=2)

        # ----------------- Left: Live Metrics Readout -----------------
        metrics_box = ttk.LabelFrame(left_col, text=" Antenna RF Performance Metrics ", padding=10)
        metrics_box.pack(fill=tk.X, pady=(0, 8))

        self.lbl_peak_info = ttk.Label(metrics_box, text="Peak: -- dBm @ --°", font=("Segoe UI", 10, "bold"))
        self.lbl_peak_info.pack(anchor=tk.W, pady=2)

        self.lbl_hpbw = ttk.Label(metrics_box, text="HPBW (3 dB): Calculating...", font=("Segoe UI", 10, "bold"), foreground="#d9534f")
        self.lbl_hpbw.pack(anchor=tk.W, pady=2)

        self.lbl_fb_ratio = ttk.Label(metrics_box, text="Front-to-Back: -- dB", font=("Segoe UI", 9))
        self.lbl_fb_ratio.pack(anchor=tk.W, pady=2)

        self.lbl_peak_gain = ttk.Label(metrics_box, text="Peak Gain: -- dBi", font=("Segoe UI", 9))
        self.lbl_peak_gain.pack(anchor=tk.W, pady=2)

        # ----------------- Left: Execution & Status -----------------
        ctrl_box = ttk.LabelFrame(left_col, text=" Control ", padding=10)
        ctrl_box.pack(fill=tk.X, pady=(0, 8))

        btn_frame = ttk.Frame(ctrl_box)
        btn_frame.pack(fill=tk.X, pady=4)
        self.btn_start = ttk.Button(btn_frame, text="▶ Start Sweep", command=self.start_sweep)
        self.btn_start.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4))
        self.btn_stop = ttk.Button(btn_frame, text="⏹ Abort", command=self.stop_sweep, state=tk.DISABLED)
        self.btn_stop.pack(side=tk.RIGHT, expand=True, fill=tk.X)

        self.btn_home = ttk.Button(ctrl_box, text="⌂ Home 0°", command=self.home_motor)
        self.btn_home.pack(fill=tk.X, pady=(0, 4))

        self.lbl_status = ttk.Label(ctrl_box, text="Status: Ready", font=("Segoe UI", 9, "italic"))
        self.lbl_status.pack(anchor=tk.W, pady=2)

        self.progress_bar = ttk.Progressbar(ctrl_box, orient=tk.HORIZONTAL, mode="determinate")
        self.progress_bar.pack(fill=tk.X, pady=4)

        # ----------------- Left: Log -----------------
        log_box = ttk.LabelFrame(left_col, text=" Activity Log ", padding=6)
        log_box.pack(fill=tk.BOTH, expand=True)

        self.txt_log = tk.Text(log_box, height=8, width=35, font=("Consolas", 8), bg="#1e1e1e", fg="#d4d4d4")
        self.txt_log.pack(fill=tk.BOTH, expand=True)

        # ----------------- Right: Live Polar Display -----------------
        canvas_box = ttk.LabelFrame(right_col, text=" Polar Radiation Pattern & 3 dB Beamwidth ", padding=10)
        canvas_box.pack(fill=tk.BOTH, expand=True)

        self.polar_canvas = tk.Canvas(canvas_box, bg="#ffffff", highlightthickness=0)
        self.polar_canvas.pack(fill=tk.BOTH, expand=True)
        self.polar_canvas.bind("<Configure>", lambda e: self.redraw_polar())

    # ========================================================================
    # 5. POLAR DRAWING & HPBW VISUALIZATION
    # ========================================================================

    def redraw_polar(self):
        """Draws the polar grid, measured pattern, and HPBW overlays."""
        c = self.polar_canvas
        c.delete("all")

        w = c.winfo_width()
        h = c.winfo_height()
        if w < 50 or h < 50:
            return

        cx, cy = w / 2, h / 2
        radius = min(cx, cy) - 40
        if radius <= 15:
            return

        # Legend: identifies which spherical cut this trace represents
        c.create_text(10, 10, anchor="nw", text=f"Cut: {self.current_cut_type}", font=("Segoe UI", 9, "bold"), fill="#333333")

        # Power dynamic range scale - adapts to the measured data (+/-5 dBm headroom), falls back to -40..0 dBm
        powers = [pt[1] for pt in self.measured_points]
        if powers:
            min_pwr = min(powers) - 5.0
            max_pwr = max(powers) + 5.0
            if max_pwr - min_pwr < 1.0:
                max_pwr = min_pwr + 1.0
        else:
            min_pwr = -40.0
            max_pwr = 0.0

        # Draw concentric dBm rings
        rings = [min_pwr + i * (max_pwr - min_pwr) / 4.0 for i in range(5)]
        for r_pwr in rings:
            r_ratio = (r_pwr - min_pwr) / (max_pwr - min_pwr)
            r_px = r_ratio * radius
            c.create_oval(cx - r_px, cy - r_px, cx + r_px, cy + r_px, outline="#e0e0e0", width=1)
            c.create_text(cx + 4, cy - r_px + 8, text=f"{r_pwr:.1f} dBm", fill="#888888", font=("Segoe UI", 7))

        # Draw radial spokes (every 45°)
        for deg in range(0, 360, 45):
            rad = math.radians(deg)
            x2 = cx + radius * math.sin(rad)
            y2 = cy - radius * math.cos(rad)
            c.create_line(cx, cy, x2, y2, fill="#eaeaea", dash=(2, 2))
            # Text on perimeter
            lx = cx + (radius + 20) * math.sin(rad)
            ly = cy - (radius + 20) * math.cos(rad)
            c.create_text(lx, ly, text=f"{deg}°", font=("Segoe UI", 8, "bold"), fill="#555555")

        if not self.measured_points:
            return

        # Map points to canvas coordinates
        canvas_coords = []
        for angle, pwr in self.measured_points:
            clamped_pwr = max(min_pwr, min(max_pwr, pwr))
            norm = (clamped_pwr - min_pwr) / (max_pwr - min_pwr)
            pt_r = norm * radius
            pt_rad = math.radians(angle)
            pt_x = cx + pt_r * math.sin(pt_rad)
            pt_y = cy - pt_r * math.cos(pt_rad)
            canvas_coords.append((pt_x, pt_y))

        # Connect measured trace with blue line
        for i in range(len(canvas_coords) - 1):
            x1, y1 = canvas_coords[i]
            x2, y2 = canvas_coords[i + 1]
            c.create_line(x1, y1, x2, y2, fill="#007acc", width=2)

        # Close the loop back to the start (e.g. 355° -> 0°) for full 360° sweeps
        if self.is_full_circle_sweep and len(canvas_coords) >= 2:
            x1, y1 = canvas_coords[-1]
            x2, y2 = canvas_coords[0]
            c.create_line(x1, y1, x2, y2, fill="#007acc", width=2)

        # Draw measurement points
        for x, y in canvas_coords:
            c.create_oval(x - 3, y - 3, x + 3, y + 3, fill="#e51400", outline="")

        # ----------------- OVERLAY HPBW & 3 dB CONTOUR -----------------
        metrics = calculate_rf_metrics(self.measured_points)
        if metrics and metrics["hpbw_deg"] is not None:
            left_a = metrics["left_angle"]
            right_a = metrics["right_angle"]
            p_3db = metrics["target_3db"]

            # Draw 3 dB threshold dashed circle
            clamped_3db = max(min_pwr, min(max_pwr, p_3db))
            norm_3db = (clamped_3db - min_pwr) / (max_pwr - min_pwr)
            r_3db = norm_3db * radius

            c.create_oval(
                cx - r_3db, cy - r_3db, cx + r_3db, cy + r_3db,
                outline="#ff8800", width=1, dash=(3, 3)
            )

            # Draw radial lines for left and right -3 dB cut-offs
            for a_val, color, lbl in [(left_a, "#d9534f", "Left"), (right_a, "#d9534f", "Right")]:
                rad = math.radians(a_val)
                x_end = cx + radius * math.sin(rad)
                y_end = cy - radius * math.cos(rad)
                c.create_line(cx, cy, x_end, y_end, fill=color, width=2, dash=(4, 2))

                # Text label on perimeter
                lx = cx + (radius + 22) * math.sin(rad)
                ly = cy - (radius + 22) * math.cos(rad)
                c.create_text(lx, ly, text=f"-3dB ({a_val:.1f}°)", font=("Segoe UI", 7, "bold"), fill=color)

            # Draw peak indicator ray
            pk_rad = math.radians(metrics["peak_angle"])
            pk_x = cx + radius * math.sin(pk_rad)
            pk_y = cy - radius * math.cos(pk_rad)
            c.create_line(cx, cy, pk_x, pk_y, fill="#28a745", width=1, dash=(2, 2))

    # ========================================================================
    # 6. THREAD SYNCHRONIZATION & EVENT HANDLING
    # ========================================================================

    def start_sweep(self):
        try:
            cfg = {
                "simulate": self.sim_var.get(),
                "ff_ip": self.entry_ip.get().strip(),
                "ff_port": int(self.entry_ff_port.get().strip()),
                "ff_bypass_config": self.ff_bypass_var.get(),
                "center_freq_mhz": float(self.entry_freq.get()),
                "span_khz": float(self.entry_span.get()),
                "cut_type": self.cut_type_var.get(),
                "motor_port": self.entry_port.get().strip(),
                "motor_baud": 115200,
                "zero_at_start": self.zero_home_var.get(),
                "motor_velocity": float(self.entry_velocity.get()),
                "motor_acceleration": float(self.entry_acceleration.get()),
                "steps_per_deg": float(self.entry_steps_deg.get()),
                "step_angle": float(self.entry_step_deg.get()),
                "total_angle": float(self.entry_total_deg.get()),
                "settle_time_s": float(self.entry_settle.get()),
                "csv_path": self.entry_csv.get().strip(),
                "pathloss_enabled": self.pathloss_var.get(),
                "distance_m": float(self.entry_distance_m.get()),
                "rx_gain_dbi": float(self.entry_rx_gain_dbi.get()),
            }
        except ValueError as err:
            messagebox.showerror("Invalid Input", f"Please check parameters:\n{err}")
            return

        self.measured_points.clear()
        self.current_cut_type = cfg["cut_type"]
        self.is_full_circle_sweep = cfg["total_angle"] > 0 and abs(cfg["total_angle"] % 360.0) < 1e-6
        self.redraw_polar()
        self.progress_bar["maximum"] = cfg["total_angle"]
        self.progress_bar["value"] = 0

        self.btn_start.config(state=tk.DISABLED)
        self.btn_stop.config(state=tk.NORMAL)
        self.stop_event.clear()

        self.worker_thread = threading.Thread(
            target=sweep_worker,
            args=(cfg, self.msg_queue, self.stop_event),
            daemon=True
        )
        self.worker_thread.start()

    def stop_sweep(self):
        if self.worker_thread and self.worker_thread.is_alive():
            self.stop_event.set()
            self.lbl_status.config(text="Status: Stopping...")
            self.btn_stop.config(state=tk.DISABLED)

    def _after_poll_queue(self):
        """Processes messages from background thread every 50ms."""
        try:
            while True:
                msg_type, *payload = self.msg_queue.get_nowait()
                if msg_type == "STATUS":
                    self.lbl_status.config(text=f"Status: {payload[0]}")
                elif msg_type == "LOG":
                    self.txt_log.insert(tk.END, f"{payload[0]}\n")
                    self.txt_log.see(tk.END)
                elif msg_type == "DATA":
                    angle, pwr = payload[0], payload[1]
                    self.progress_bar["value"] = angle
                    self.measured_points.append((angle, pwr))

                    # Update RF metrics & HPBW
                    metrics = calculate_rf_metrics(self.measured_points)
                    if metrics:
                        self.lbl_peak_info.config(
                            text=f"Peak: {metrics['peak_power']:.2f} dBm @ {metrics['peak_angle']:.1f}°"
                        )
                        if metrics["hpbw_deg"] is not None:
                            self.lbl_hpbw.config(
                                text=f"HPBW (3 dB): {metrics['hpbw_deg']:.1f}° "
                                     f"[{metrics['left_angle']:.1f}° to {metrics['right_angle']:.1f}°]"
                            )
                        self.lbl_fb_ratio.config(
                            text=f"Front-to-Back: {metrics['fb_ratio_db']:.1f} dB (Back: {metrics['back_power']:.2f} dBm)"
                        )

                        # Peak Gain (dBi) = (Peak Power - TX Power) in dBd, +2.15 dB to convert dBd -> dBi
                        try:
                            tx_power_dbm = float(self.entry_tx_power_dbm.get())
                            gain_dbi = (metrics["peak_power"] - tx_power_dbm) + 2.15
                            self.lbl_peak_gain.config(text=f"Peak Gain: {gain_dbi:.2f} dBi (TX={tx_power_dbm:.1f} dBm)")
                        except ValueError:
                            pass

                    self.redraw_polar()
                elif msg_type == "ERROR":
                    messagebox.showerror("Runtime Error", payload[0])
                    self._reset_controls()
                elif msg_type == "FINISHED":
                    self._reset_controls()
                    messagebox.showinfo("Sweep Complete", "Radiation pattern sweep and HPBW calculation finished!")
                elif msg_type == "HOME_DONE":
                    self.btn_home.config(state=tk.NORMAL)
        except queue.Empty:
            pass

        self.after(50, self._after_poll_queue)

    def _reset_controls(self):
        self.btn_start.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)

    def home_motor(self):
        if self.worker_thread and self.worker_thread.is_alive():
            messagebox.showwarning("Sweep in progress", "Stop the current sweep before homing the motor.")
            return
        if self.sim_var.get():
            messagebox.showinfo("Simulation Mode", "Disable Simulation Mode to home the real motor.")
            return
        try:
            motor_port = self.entry_port.get().strip()
            motor_velocity = float(self.entry_velocity.get())
            motor_acceleration = float(self.entry_acceleration.get())
        except ValueError as err:
            messagebox.showerror("Invalid Input", f"Please check motor parameters:\n{err}")
            return

        self.btn_home.config(state=tk.DISABLED)
        threading.Thread(
            target=_home_motor_worker,
            args=(motor_port, motor_velocity, motor_acceleration, self.msg_queue),
            daemon=True
        ).start()


if __name__ == "__main__":
    app = RadiationPatternApp()
    app.mainloop()

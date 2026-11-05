"""Tkinter GUI for the TITAN-IMX stepper motor demo."""
from __future__ import annotations

import threading
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Callable, Optional

from .protocol import TitanProtocol
from .serial_link import SerialLink, SerialSettings, list_available_ports

POLL_INTERVAL_MS = 250
DEFAULT_PORT = "COM16"


class TitanDemoApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("TITAN-IMX Stepper Motor Demo")
        self.root.geometry("640x520")

        self._link = SerialLink(on_unsolicited_line=self._on_unsolicited_line, on_tx=self._on_tx)
        self._protocol = TitanProtocol(self._link)
        self._polling = False

        self._build_connection_panel()
        self._build_notebook()
        self._build_status_bar()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        ttk.Label(
            self.root,
            text=(
                "Note: USB only powers the controller logic/COM port - the motor coils need "
                "a separate 24-48 VDC supply connected to the drive to actually move."
            ),
            foreground="#8a6d00", wraplength=620, justify="left",
        ).pack(fill="x", padx=8, pady=(0, 4))

    # ----- layout -------------------------------------------------------
    def _build_connection_panel(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Connection")
        frame.pack(fill="x", padx=8, pady=6)

        ttk.Label(frame, text="Port:").grid(row=0, column=0, padx=4, pady=4)
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, width=10)
        self.port_combo.grid(row=0, column=1, padx=4, pady=4)
        self._refresh_ports()

        ttk.Button(frame, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=4)

        ttk.Label(frame, text="Baud:").grid(row=0, column=3, padx=4)
        self.baud_var = tk.StringVar(value="115200")
        baud_combo = ttk.Combobox(
            frame, textvariable=self.baud_var, width=8,
            values=["9600", "19200", "38400", "57600", "115200"],
        )
        baud_combo.grid(row=0, column=4, padx=4)

        ttk.Label(frame, text="EOL:").grid(row=0, column=5, padx=4)
        self.terminator_var = tk.StringVar(value="CRLF")
        ttk.Combobox(
            frame, textvariable=self.terminator_var, width=5,
            values=["CR", "LF", "CRLF"], state="readonly",
        ).grid(row=0, column=6, padx=4)

        ttk.Label(frame, text="Device ID:").grid(row=0, column=7, padx=4)
        self.device_id_var = tk.StringVar(value="1")
        ttk.Entry(frame, textvariable=self.device_id_var, width=4).grid(row=0, column=8, padx=4)

        self.connect_button = ttk.Button(frame, text="Connect", command=self._toggle_connection)
        self.connect_button.grid(row=0, column=9, padx=8)

        self.connection_indicator = ttk.Label(frame, text="Disconnected", foreground="red")
        self.connection_indicator.grid(row=0, column=10, padx=8)

    def _build_notebook(self) -> None:
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=8, pady=6)

        notebook.add(self._build_move_tab(notebook), text="Jog / Move")
        notebook.add(self._build_home_tab(notebook), text="Homing")
        notebook.add(self._build_io_tab(notebook), text="Digital I/O")
        notebook.add(self._build_terminal_tab(notebook), text="Raw Terminal")

    def _build_move_tab(self, parent: ttk.Notebook) -> ttk.Frame:
        tab = ttk.Frame(parent)

        enable_frame = ttk.LabelFrame(tab, text="Servo / amplifier")
        enable_frame.pack(fill="x", padx=8, pady=6)
        ttk.Button(enable_frame, text="Clear Fault", command=self._clear_fault).grid(row=0, column=0, padx=6, pady=4)
        ttk.Button(enable_frame, text="Enable (SVON)", command=self._enable).grid(row=0, column=1, padx=6, pady=4)
        ttk.Button(enable_frame, text="Disable (SVOFF)", command=self._disable).grid(row=0, column=2, padx=6, pady=4)
        ttk.Label(enable_frame, text="Clear faults then Enable before any motion will be accepted.").grid(
            row=0, column=3, padx=8
        )

        params = ttk.LabelFrame(tab, text="Motion parameters")
        params.pack(fill="x", padx=8, pady=6)
        self.velocity_var = tk.StringVar(value="1000")
        self.accel_var = tk.StringVar(value="500")
        ttk.Label(params, text="Velocity:").grid(row=0, column=0, padx=4, pady=4)
        ttk.Entry(params, textvariable=self.velocity_var, width=10).grid(row=0, column=1)
        ttk.Button(params, text="Set", command=self._set_velocity).grid(row=0, column=2, padx=4)
        ttk.Label(params, text="Acceleration:").grid(row=0, column=3, padx=4)
        ttk.Entry(params, textvariable=self.accel_var, width=10).grid(row=0, column=4)
        ttk.Button(params, text="Set", command=self._set_acceleration).grid(row=0, column=5, padx=4)

        jog = ttk.LabelFrame(tab, text="Jog")
        jog.pack(fill="x", padx=8, pady=6)
        jog_minus = ttk.Button(jog, text="◀ Jog -")
        jog_minus.grid(row=0, column=0, padx=6, pady=6)
        jog_plus = ttk.Button(jog, text="Jog + ▶")
        jog_plus.grid(row=0, column=1, padx=6, pady=6)
        self._bind_hold(
            jog_minus,
            lambda: self._move_with_params(lambda: self._protocol.jog(-1)),
            self._log_move_result,
            self._protocol.stop,
            self._log_result,
        )
        self._bind_hold(
            jog_plus,
            lambda: self._move_with_params(lambda: self._protocol.jog(1)),
            self._log_move_result,
            self._protocol.stop,
            self._log_result,
        )

        move = ttk.LabelFrame(tab, text="Move (encoder counts)")
        move.pack(fill="x", padx=8, pady=6)
        self.abs_position_var = tk.StringVar(value="0")
        self.rel_distance_var = tk.StringVar(value="1000")
        ttk.Label(move, text="Absolute position:").grid(row=0, column=0, padx=4, pady=4)
        ttk.Entry(move, textvariable=self.abs_position_var, width=10).grid(row=0, column=1)
        ttk.Button(move, text="Move Absolute", command=self._move_absolute).grid(row=0, column=2, padx=4)
        ttk.Label(move, text="Relative distance:").grid(row=1, column=0, padx=4, pady=4)
        ttk.Entry(move, textvariable=self.rel_distance_var, width=10).grid(row=1, column=1)
        ttk.Button(move, text="Move Relative", command=self._move_relative).grid(row=1, column=2, padx=4)

        angle = ttk.LabelFrame(tab, text="Move by angle (degrees)")
        angle.pack(fill="x", padx=8, pady=6)
        self.counts_per_rev_var = tk.StringVar(value="32000")
        ttk.Label(angle, text="Counts/revolution:").grid(row=0, column=0, padx=4, pady=4)
        ttk.Entry(angle, textvariable=self.counts_per_rev_var, width=8).grid(row=0, column=1)
        ttk.Label(angle, text="(from Arcus spec - correct if your motor/encoder differs)").grid(
            row=0, column=2, columnspan=2, padx=4, sticky="w"
        )

        self.abs_angle_var = tk.StringVar(value="32")
        ttk.Label(angle, text="Absolute angle (°):").grid(row=1, column=0, padx=4, pady=4)
        ttk.Entry(angle, textvariable=self.abs_angle_var, width=8).grid(row=1, column=1)
        ttk.Button(angle, text="Move To Angle", command=self._move_absolute_angle).grid(row=1, column=2, padx=4)
        ttk.Button(angle, text="Home (0°)", command=self._go_home).grid(row=1, column=3, padx=4)

        self.rel_angle_var = tk.StringVar(value="32")
        ttk.Label(angle, text="Relative angle (°):").grid(row=2, column=0, padx=4, pady=4)
        ttk.Entry(angle, textvariable=self.rel_angle_var, width=8).grid(row=2, column=1)
        ttk.Button(angle, text="Move By Angle", command=self._move_relative_angle).grid(row=2, column=2, padx=4)

        stop_frame = ttk.Frame(tab)
        stop_frame.pack(fill="x", padx=8, pady=10)
        stop_button = tk.Button(
            stop_frame, text="STOP", bg="#c0392b", fg="white",
            font=("Segoe UI", 12, "bold"), command=self._stop,
        )
        stop_button.pack(fill="x", ipady=8)

        return tab

    def _build_home_tab(self, parent: ttk.Notebook) -> ttk.Frame:
        tab = ttk.Frame(parent)

        simple = ttk.LabelFrame(tab, text="Recommended (works without a wired home switch)")
        simple.pack(fill="x", padx=8, pady=6)
        ttk.Button(simple, text="Zero here (set current position = 0°)", command=self._zero_position).grid(
            row=0, column=0, padx=6, pady=6
        )
        ttk.Button(simple, text="Go to Home (0°)", command=self._go_home).grid(row=0, column=1, padx=6, pady=6)

        advanced = ttk.LabelFrame(tab, text="Advanced: hardware homing (seeks home/limit switch)")
        advanced.pack(fill="x", padx=8, pady=6)
        ttk.Label(advanced, text="Direction:").grid(row=0, column=0, padx=4, pady=4)
        self.home_direction_var = tk.StringVar(value="+")
        ttk.Combobox(advanced, textvariable=self.home_direction_var, width=4, values=["+", "-"]).grid(row=0, column=1)
        ttk.Button(advanced, text="Home", command=self._home).grid(row=0, column=2, padx=8)
        ttk.Label(
            advanced,
            text="Previously returned COMERR3 with the old =mode syntax; this direction-suffix "
            "form is an unconfirmed guess matching JOGX+/-. May need a wired home/limit switch.",
            wraplength=480, justify="left",
        ).grid(row=1, column=0, columnspan=4, padx=4, pady=4, sticky="w")

        self.home_status_label = ttk.Label(tab, text="Status: -")
        self.home_status_label.pack(anchor="w", padx=8, pady=6)
        return tab

    def _build_io_tab(self, parent: ttk.Notebook) -> ttk.Frame:
        tab = ttk.Frame(parent)

        inputs_frame = ttk.LabelFrame(tab, text="Digital inputs (polled)")
        inputs_frame.pack(fill="x", padx=8, pady=6)
        self.inputs_label = ttk.Label(inputs_frame, text="-")
        self.inputs_label.pack(anchor="w", padx=4, pady=4)

        outputs_frame = ttk.LabelFrame(tab, text="Digital outputs")
        outputs_frame.pack(fill="x", padx=8, pady=6)
        self.output_vars: list[tk.BooleanVar] = []
        for ch in range(4):
            var = tk.BooleanVar(value=False)
            self.output_vars.append(var)
            ttk.Checkbutton(
                outputs_frame, text=f"OUT{ch}", variable=var,
                command=lambda c=ch, v=var: self._set_output(c, v.get()),
            ).grid(row=0, column=ch, padx=8, pady=4)

        return tab

    def _build_terminal_tab(self, parent: ttk.Notebook) -> ttk.Frame:
        tab = ttk.Frame(parent)

        self.terminal_log = tk.Text(tab, state="disabled", height=18, wrap="none")
        self.terminal_log.pack(fill="both", expand=True, padx=8, pady=6)

        entry_frame = ttk.Frame(tab)
        entry_frame.pack(fill="x", padx=8, pady=6)
        self.terminal_entry_var = tk.StringVar()
        entry = ttk.Entry(entry_frame, textvariable=self.terminal_entry_var)
        entry.pack(side="left", fill="x", expand=True)
        entry.bind("<Return>", lambda _e: self._send_raw_command())
        ttk.Button(entry_frame, text="Send", command=self._send_raw_command).pack(side="left", padx=4)

        return tab

    def _build_status_bar(self) -> None:
        bar = ttk.Frame(self.root)
        bar.pack(fill="x", side="bottom", padx=8, pady=4)
        self.status_label = ttk.Label(bar, text="Position: -")
        self.status_label.pack(side="left")
        self.fault_label = ttk.Label(bar, text="Fault: -")
        self.fault_label.pack(side="left", padx=16)

    # ----- connection handling ------------------------------------------
    def _refresh_ports(self) -> None:
        ports = list_available_ports()
        if DEFAULT_PORT not in ports:
            ports = [DEFAULT_PORT] + ports
        self.port_combo["values"] = ports

    def _toggle_connection(self) -> None:
        if self._link.is_connected:
            self._polling = False
            self._link.disconnect()
            self.connect_button.config(text="Connect")
            self.connection_indicator.config(text="Disconnected", foreground="red")
            return

        terminator = {"CR": "\r", "LF": "\n", "CRLF": "\r\n"}[self.terminator_var.get()]
        settings = SerialSettings(
            port=self.port_var.get(), baudrate=int(self.baud_var.get()), terminator=terminator
        )
        try:
            self._link.connect(settings)
        except Exception as exc:  # noqa: BLE001 - surface any connection failure to the user
            messagebox.showerror("Connection failed", str(exc))
            return

        self._protocol.device_id = int(self.device_id_var.get())
        self.connect_button.config(text="Disconnect")
        self.connection_indicator.config(text=f"Connected ({settings.port})", foreground="green")
        self._polling = True
        self._poll_status()

    def _on_close(self) -> None:
        self._polling = False
        self._link.disconnect()
        self.root.destroy()

    # ----- background execution helper -----------------------------------
    def _run_async(self, func: Callable[[], object], on_done: Optional[Callable[[object], None]] = None) -> None:
        def worker() -> None:
            try:
                result = func()
            except TimeoutError:
                self._append_terminal_line("<< (no response within timeout - command was still sent)")
                return
            except Exception as exc:  # noqa: BLE001 - report to the log/terminal instead of crashing
                self._append_terminal_line(f"ERROR: {exc}")
                return
            if on_done is not None:
                self.root.after(0, lambda: on_done(result))

        threading.Thread(target=worker, daemon=True).start()

    def _bind_hold(
        self,
        widget: tk.Widget,
        on_press: Callable[[], object],
        on_press_done: Optional[Callable[[object], None]],
        on_release: Callable[[], object],
        on_release_done: Optional[Callable[[object], None]],
    ) -> None:
        widget.bind("<ButtonPress-1>", lambda _e: self._run_async(on_press, on_press_done))
        widget.bind("<ButtonRelease-1>", lambda _e: self._run_async(on_release, on_release_done))

    # ----- action handlers ------------------------------------------------
    def _require_connected(self) -> bool:
        if not self._link.is_connected:
            messagebox.showwarning("Not connected", "Connect to the motor first.")
            return False
        return True

    def _move_absolute(self) -> None:
        if not self._require_connected():
            return
        position = int(self.abs_position_var.get())
        self._run_async(lambda: self._move_with_params(lambda: self._protocol.move_absolute(position)), self._log_move_result)

    def _move_relative(self) -> None:
        if not self._require_connected():
            return
        distance = int(self.rel_distance_var.get())
        self._run_async(lambda: self._move_with_params(lambda: self._protocol.move_relative(distance)), self._log_move_result)

    def _degrees_to_counts(self, degrees: float) -> int:
        counts_per_rev = float(self.counts_per_rev_var.get())
        return round(degrees * counts_per_rev / 360.0)

    def _move_absolute_angle(self) -> None:
        if not self._require_connected():
            return
        position = self._degrees_to_counts(float(self.abs_angle_var.get()))
        self._run_async(lambda: self._move_with_params(lambda: self._protocol.move_absolute(position)), self._log_move_result)

    def _move_relative_angle(self) -> None:
        if not self._require_connected():
            return
        distance = self._degrees_to_counts(float(self.rel_angle_var.get()))
        self._run_async(lambda: self._move_with_params(lambda: self._protocol.move_relative(distance)), self._log_move_result)

    def _go_home(self) -> None:
        if not self._require_connected():
            return
        self._run_async(lambda: self._move_with_params(self._protocol.go_home), self._log_move_result)

    def _move_with_params(self, mover: Callable[[], str]) -> tuple[str, str]:
        """Push the current speed/acceleration before every move - a move with speed=0 looks like 'nothing happened'."""
        self._protocol.set_speed(float(self.velocity_var.get()))
        self._protocol.set_acceleration(float(self.accel_var.get()))
        response = mover()
        fault = self._protocol.read_fault()
        return response, fault

    def _log_move_result(self, result: tuple[str, str]) -> None:
        response, fault = result
        self._append_terminal_line(f"<< move: {response}")
        self._append_terminal_line(f"<< fault register: {fault}")

    def _clear_fault(self) -> None:
        if not self._require_connected():
            return
        self._run_async(self._protocol.clear_fault, self._log_result)

    def _enable(self) -> None:
        if not self._require_connected():
            return
        self._run_async(self._protocol.initialize, lambda _r: self._append_terminal_line("Servo enabled."))

    def _disable(self) -> None:
        if not self._require_connected():
            return
        self._run_async(self._protocol.disable, self._log_result)

    def _set_velocity(self) -> None:
        if not self._require_connected():
            return
        self._run_async(lambda: self._protocol.set_speed(float(self.velocity_var.get())), self._log_result)

    def _set_acceleration(self) -> None:
        if not self._require_connected():
            return
        self._run_async(lambda: self._protocol.set_acceleration(float(self.accel_var.get())), self._log_result)

    def _stop(self) -> None:
        if not self._link.is_connected:
            return
        self._run_async(self._protocol.stop, self._log_result)

    def _home(self) -> None:
        if not self._require_connected():
            return
        direction = 1 if self.home_direction_var.get() == "+" else -1
        self._run_async(
            lambda: self._protocol.home(direction), lambda r: self.home_status_label.config(text=f"Status: {r}")
        )

    def _zero_position(self) -> None:
        if not self._require_connected():
            return
        self._run_async(
            self._protocol.zero_position, lambda r: self.home_status_label.config(text=f"Zeroed: {r}")
        )

    def _set_output(self, channel: int, state: bool) -> None:
        if not self._require_connected():
            return
        self._run_async(lambda: self._protocol.set_output(channel, state), self._log_result)

    def _send_raw_command(self) -> None:
        if not self._require_connected():
            return
        command = self.terminal_entry_var.get().strip()
        if not command:
            return
        self.terminal_entry_var.set("")
        # Auto-apply the @<ID>: frame unless the user already typed their own.
        framed = command if command.startswith("@") else f"@{self._protocol.device_id:02d}:{command}"
        self._run_async(
            lambda: self._link.send_command(framed),
            lambda response: self._append_terminal_line(f"<< {response}"),
        )

    # ----- polling / status -------------------------------------------------
    def _poll_status(self) -> None:
        if not self._polling or not self._link.is_connected:
            return
        self._run_async(self._protocol.read_position, self._on_position)
        self._run_async(self._protocol.read_inputs, lambda r: self.inputs_label.config(text=r))
        self._run_async(self._protocol.read_fault, lambda r: self.fault_label.config(text=f"Fault: {r}"))
        self.root.after(POLL_INTERVAL_MS, self._poll_status)

    def _on_position(self, position: Optional[int]) -> None:
        self.status_label.config(text=f"Position: {position if position is not None else '-'}")

    def _log_result(self, response: str) -> None:
        self._append_terminal_line(f"<< {response}")

    def _append_terminal_line(self, text: str) -> None:
        self.terminal_log.config(state="normal")
        self.terminal_log.insert("end", text + "\n")
        self.terminal_log.see("end")
        self.terminal_log.config(state="disabled")

    def _on_unsolicited_line(self, text: str) -> None:
        self.root.after(0, lambda: self._append_terminal_line(f"[async] {text}"))

    def _on_tx(self, command: str) -> None:
        self.root.after(0, lambda: self._append_terminal_line(f">> {command}"))

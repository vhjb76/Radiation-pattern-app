"""Typed command API for the TITAN-IMX, built on top of SerialLink + commands.py.

TITAN-ASCII frames every command as "@<ID>:<CMD>" and replies as
"#<ID>:<CMD>=<VALUE>" - this class applies/strips that framing so callers just
deal with plain values.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional

from . import commands
from .serial_link import SerialLink

# Distance (encoder counts) used to emulate continuous jogging via a long move.
_JOG_STEP = 10_000_000


@dataclass
class MotorStatus:
    raw: str
    moving: Optional[bool] = None
    error: Optional[bool] = None


class TitanProtocol:
    """Builds/sends framed TITAN-IMX commands and parses their responses."""

    def __init__(self, link: SerialLink, device_id: int = 1):
        self._link = link
        self.device_id = device_id

    def _send(self, cmd: str) -> str:
        frame = f"@{self.device_id:02d}:{cmd}"
        response = self._link.send_command(frame)
        return _strip_response_value(response)

    def initialize(self) -> None:
        """Required handshake before motion commands are accepted: clear faults, then enable."""
        self.clear_fault()
        self.enable()

    def clear_fault(self) -> str:
        return self._send(commands.CLEAR_FAULT)

    def enable(self) -> str:
        return self._send(commands.SERVO_ON)

    def disable(self) -> str:
        return self._send(commands.SERVO_OFF)

    def move_absolute(self, position: int) -> str:
        cmd = commands.MOVE_ABSOLUTE.format(position=position)
        return self._send(cmd)

    def move_relative(self, distance: int) -> str:
        cmd = commands.MOVE_RELATIVE.format(distance=distance)
        return self._send(cmd)

    def jog(self, direction: int) -> str:
        """direction: +1 or -1.

        "JOGX+"/"JOGX-" returned COMERR2 (unrecognized command) on real hardware,
        so jog is emulated using the confirmed-working X= absolute move: move far
        in the requested direction, then STOPX on release cancels it early.
        """
        current = self.read_position() or 0
        target = current + (_JOG_STEP if direction >= 0 else -_JOG_STEP)
        return self.move_absolute(target)

    def stop(self) -> str:
        return self._send(commands.STOP)

    def abort(self) -> str:
        return self._send(commands.ABORT)

    def set_speed(self, speed: float) -> str:
        cmd = commands.SET_SPEED.format(speed=speed)
        return self._send(cmd)

    def set_acceleration(self, acceleration: float) -> str:
        cmd = commands.SET_ACCELERATION.format(acceleration=acceleration)
        return self._send(cmd)

    def set_deceleration(self, deceleration: float) -> str:
        cmd = commands.SET_DECELERATION.format(deceleration=deceleration)
        return self._send(cmd)

    def home(self, direction: int = 1) -> str:
        """Hardware homing (seeks a home/limit switch) - unconfirmed, may error depending on firmware."""
        cmd = commands.HOME_START.format(direction="+" if direction >= 0 else "-")
        return self._send(cmd)

    def zero_position(self) -> str:
        """Defines the current physical position as 0 without any motion - no switch required."""
        return self._send(commands.ZERO_POSITION)

    def go_home(self) -> str:
        """Moves to the already-confirmed-working absolute position 0."""
        return self.move_absolute(0)

    def read_position(self) -> Optional[int]:
        return _extract_int(self._send(commands.QUERY_POSITION))

    def read_status(self) -> MotorStatus:
        return MotorStatus(raw=self._send(commands.QUERY_STATUS))

    def read_fault(self) -> str:
        return self._send(commands.QUERY_FAULT)

    def read_inputs(self) -> str:
        return self._send(commands.QUERY_INPUTS)

    def read_outputs(self) -> str:
        return self._send(commands.QUERY_OUTPUTS)

    def set_output(self, channel: int, state: bool) -> str:
        cmd = commands.SET_OUTPUT.format(channel=channel, state=1 if state else 0)
        return self._send(cmd)

    def read_firmware_version(self) -> str:
        return self._send(commands.QUERY_FIRMWARE_VERSION)


_NUMBER_RE = re.compile(r"-?\d+(\.\d+)?")
_RESPONSE_VALUE_RE = re.compile(r"^#\d+:[^=]+=(.*)$")


def _strip_response_value(response: str) -> str:
    """Reduce "#01:CMD=VALUE" down to just VALUE; passes through anything else unchanged."""
    match = _RESPONSE_VALUE_RE.match(response.strip())
    return match.group(1) if match else response


def _extract_int(response: str) -> Optional[int]:
    match = _NUMBER_RE.search(response)
    return int(float(match.group())) if match else None


def _extract_float(response: str) -> Optional[float]:
    match = _NUMBER_RE.search(response)
    return float(match.group()) if match else None


"""TITAN-ASCII command keywords for the TITAN-IMX.

*** IMPORTANT ***
Based on a community-sourced protocol writeup (not the official Arcus manual,
which remains login-gated at arcusservo.com/titan-downloads and inaccessible
to the assistant). The protocol frames every command as "@<ID>:<CMD>\\r\\n" and
replies come back as "#<ID>:<CMD>=<VALUE>\\r\\n"; framing/parsing is handled by
`protocol.TitanProtocol`. These constants are just the bare command keywords
and are still unverified against the real hardware - use the Raw Terminal tab
(which applies the same framing) to confirm/correct any of these.
"""

# Fault / enable handshake - required before any motion will be accepted.
CLEAR_FAULT = "ECLEARX"
SERVO_ON = "SVON"
SERVO_OFF = "SVOFF"

# Motion parameters
SET_SPEED = "HSPD={speed}"
SET_ACCELERATION = "ACC={acceleration}"
SET_DECELERATION = "DEC={deceleration}"
# Motion - "X=" both sets the target AND starts the move (no separate execute step).
# Confirmed working on real hardware.
MOVE_ABSOLUTE = "X={position}"
# Relative move keyword is unconfirmed - correct via Raw Terminal if wrong.
MOVE_RELATIVE = "DX={distance}"
# Confirmed NOT working (COMERR2 = unrecognized command) - kept only for reference.
# protocol.TitanProtocol.jog() emulates jogging via MOVE_ABSOLUTE instead.
JOG_PLUS = "JOGX+"
JOG_MINUS = "JOGX-"
STOP = "STOPX"
ABORT = "ABORTX"

# Zero the actual encoder position register - defines "here" as position 0/home,
# without any physical motion. Confirmed-safe alternative to hardware homing.
ZERO_POSITION = "EX=0"

# Hardware homing (drive physically seeks a home/limit switch) - "HOMEX=<mode>" returned
# COMERR3 (likely "invalid parameter"), so this direction-suffix form is an unconfirmed
# alternative guess, matching the JOGX+/- convention used by other action commands.
HOME_START = "HOMEX{direction}"

# Status / queries
QUERY_STATUS = "MST"
QUERY_FAULT = "FLT"
QUERY_POSITION = "EX"

# Digital I/O - unconfirmed keywords.
QUERY_INPUTS = "IN"
QUERY_OUTPUTS = "OUT"
SET_OUTPUT = "OUT{channel}={state}"

# Identification
QUERY_FIRMWARE_VERSION = "VER"


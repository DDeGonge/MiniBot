"""
comms/protocol.py  —  MiniBot Chess Swarm Coordinator

Numeric CSV-over-serial protocol (newline-terminated).
All messages are prefixed with '>'.

Host → ESP32 commands:
    >0,{id},{mode},{duty1},{duty2}                       Motor test
    >1,{id},{x_mm},{y_mm},{theta_rad},{duration_ms}      Position command
    >2,{id}                                              Position request
    >5,{id}                                              Mag field request
    >7                                                   Sync broadcast
    >254,{0|1}                                           Electromagnet enable/disable
    >255                                                 Ping

ESP32 → Host responses:
    >3,{id},{x_mm},{y_mm},{theta_rad},{timestamp_ms},{battery_v}   ACK / position update
    >4,{id},{err_type},{timestamp_ms}                              NACK
    >6,{id},{bx},{by},{bz},{timestamp_ms}                          Mag field response
    >255                                                           Pong
    >ERR,{message}                                                 Parse error

Theta is transmitted in radians on the wire.  Builder functions accept degrees
and convert internally; parse_line converts back to degrees so the rest of the
application can remain degree-based.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Union

from config import COMM

# ---------------------------------------------------------------------------
# Parsed message dataclass
# ---------------------------------------------------------------------------


@dataclass
class ParsedMessage:
    """Structured representation of one line received from the ESP32.

    Attributes:
        msg_type:     One of the COMM.RESP_* constants (int or 'ERR'), or
                      'UNKNOWN' for unrecognised lines.
        piece_id:     Integer piece ID, present for ACK / NACK / mag field.
        x_mm:         X position (mm), present for ACK only.
        y_mm:         Y position (mm), present for ACK only.
        theta_deg:    Orientation in degrees (converted from wire radians),
                      present for ACK only.
        timestamp_ms: ESP internal timestamp (ms), present for ACK / NACK /
                      mag field responses.
        battery_v:    Battery voltage (V), present for ACK only.
        err_type:     Numeric error code, present for NACK only.
        reason:       String representation of err_type for display / logging.
        mag_x:        Magnetic field X component, present for mag field only.
        mag_y:        Magnetic field Y component, present for mag field only.
        mag_z:        Magnetic field Z component, present for mag field only.
        parse_error:  Error message text, present for RESP_PARSE_ERROR only.
        raw:          The original raw string for logging / debug.
    """

    msg_type: Union[int, str]
    piece_id: Optional[int] = None
    x_mm: Optional[float] = None
    y_mm: Optional[float] = None
    theta_deg: Optional[float] = None
    timestamp_ms: Optional[int] = None
    battery_v: Optional[float] = None
    err_type: Optional[int] = None
    reason: Optional[str] = None
    mag_x: Optional[float] = None
    mag_y: Optional[float] = None
    mag_z: Optional[float] = None
    parse_error: Optional[str] = None
    raw: str = ""


# ---------------------------------------------------------------------------
# Command builders  (host → ESP32)
# ---------------------------------------------------------------------------


def build_motor_test(
    piece_id: int,
    mode: int,
    duty1: int,
    duty2: int,
) -> bytes:
    """Build a motor test command (cmd_id=0).

    Format: ``>0,{id},{mode},{duty1},{duty2}\\n``
    """
    line = (
        f"{COMM.MSG_PREFIX}{COMM.CMD_MOTOR_TEST}"
        f"{COMM.DELIMITER}{piece_id}"
        f"{COMM.DELIMITER}{mode}"
        f"{COMM.DELIMITER}{duty1}"
        f"{COMM.DELIMITER}{duty2}"
        f"{COMM.TERMINATOR}"
    )
    return line.encode(COMM.ENCODING)


def build_position_command(
    piece_id: int,
    x_mm: float,
    y_mm: float,
    theta_deg: float,
    duration_ms: int,
) -> bytes:
    """Build a position command (cmd_id=1).

    Theta is accepted in degrees and converted to radians for the wire.

    Format: ``>1,{id},{x_mm:.2f},{y_mm:.2f},{theta_rad:.4f},{duration_ms}\\n``
    """
    theta_rad = math.radians(theta_deg)
    line = (
        f"{COMM.MSG_PREFIX}{COMM.CMD_POSITION}"
        f"{COMM.DELIMITER}{piece_id}"
        f"{COMM.DELIMITER}{x_mm:.2f}"
        f"{COMM.DELIMITER}{y_mm:.2f}"
        f"{COMM.DELIMITER}{theta_rad:.4f}"
        f"{COMM.DELIMITER}{duration_ms}"
        f"{COMM.TERMINATOR}"
    )
    return line.encode(COMM.ENCODING)


def build_position_request(piece_id: int) -> bytes:
    """Build a position request for a single piece (cmd_id=2).

    Format: ``>2,{id}\\n``
    """
    line = (
        f"{COMM.MSG_PREFIX}{COMM.CMD_POSITION_REQUEST}"
        f"{COMM.DELIMITER}{piece_id}"
        f"{COMM.TERMINATOR}"
    )
    return line.encode(COMM.ENCODING)


def build_mag_field_request(piece_id: int) -> bytes:
    """Build a magnetic field request for a single piece (cmd_id=5).

    Format: ``>5,{id}\\n``
    """
    line = (
        f"{COMM.MSG_PREFIX}{COMM.CMD_MAG_FIELD_REQUEST}"
        f"{COMM.DELIMITER}{piece_id}"
        f"{COMM.TERMINATOR}"
    )
    return line.encode(COMM.ENCODING)


def build_sync() -> bytes:
    """Build a sync broadcast command (cmd_id=7).

    Format: ``>7\\n``
    """
    return f"{COMM.MSG_PREFIX}{COMM.CMD_SYNC}{COMM.TERMINATOR}".encode(COMM.ENCODING)


def build_electromagnet(state: int) -> bytes:
    """Build an electromagnet enable/disable command (cmd_id=254).

    Args:
        state: COMM.MAG_OFF (0) to disable, COMM.MAG_ON (1) to enable.

    Format: ``>254,{0|1}\\n``
    """
    if state not in (COMM.MAG_OFF, COMM.MAG_ON):
        raise ValueError(f"Invalid electromagnet state: {state!r}")
    line = (
        f"{COMM.MSG_PREFIX}{COMM.CMD_ELECTROMAGNET}"
        f"{COMM.DELIMITER}{state}"
        f"{COMM.TERMINATOR}"
    )
    return line.encode(COMM.ENCODING)


def build_ping() -> bytes:
    """Build a ping command (cmd_id=255).

    Format: ``>255\\n``
    """
    return f"{COMM.MSG_PREFIX}{COMM.CMD_PING}{COMM.TERMINATOR}".encode(COMM.ENCODING)


# ---------------------------------------------------------------------------
# Response parser  (ESP32 → host)
# ---------------------------------------------------------------------------


def parse_line(raw: Union[str, bytes]) -> ParsedMessage:
    """Parse one newline-terminated line from the ESP32.

    Returns a ParsedMessage with msg_type='UNKNOWN' for lines that do not
    match the expected protocol — these are logged by the caller but do not
    raise exceptions, ensuring robustness against stray debug prints.

    Args:
        raw: A single line (str or bytes).  Trailing whitespace is stripped.

    Returns:
        ParsedMessage with populated fields.  Theta is converted from wire
        radians to degrees.
    """
    if isinstance(raw, bytes):
        try:
            raw = raw.decode(COMM.ENCODING)
        except UnicodeDecodeError:
            return ParsedMessage(msg_type="UNKNOWN", raw=repr(raw))

    raw = raw.strip()

    if not raw:
        return ParsedMessage(msg_type="UNKNOWN", raw=raw)

    # All valid frames start with '>'
    if not raw.startswith(COMM.MSG_PREFIX):
        return ParsedMessage(msg_type="UNKNOWN", raw=raw)

    body = raw[len(COMM.MSG_PREFIX) :]
    parts = body.split(COMM.DELIMITER)

    # Parse error from ESP: >ERR,{message}
    if parts[0] == COMM.RESP_PARSE_ERROR:
        message = COMM.DELIMITER.join(parts[1:]) if len(parts) > 1 else ""
        return ParsedMessage(
            msg_type=COMM.RESP_PARSE_ERROR,
            parse_error=message,
            raw=raw,
        )

    try:
        msg_id = int(parts[0])
    except (ValueError, IndexError):
        return ParsedMessage(msg_type="UNKNOWN", raw=raw)

    try:
        if msg_id == COMM.RESP_ACK and len(parts) == 7:
            # >3,{id},{x_mm},{y_mm},{theta_rad},{timestamp_ms},{battery_v}
            theta_deg = math.degrees(float(parts[4]))
            return ParsedMessage(
                msg_type=COMM.RESP_ACK,
                piece_id=int(parts[1], 0),
                x_mm=float(parts[2]),
                y_mm=float(parts[3]),
                theta_deg=theta_deg,
                timestamp_ms=int(parts[5]),
                battery_v=float(parts[6]),
                raw=raw,
            )

        if msg_id == COMM.RESP_NACK and len(parts) == 4:
            # >4,{id},{err_type},{timestamp_ms}
            err_type_val = int(parts[2])
            return ParsedMessage(
                msg_type=COMM.RESP_NACK,
                piece_id=int(parts[1], 0),
                err_type=err_type_val,
                reason=str(err_type_val),
                timestamp_ms=int(parts[3]),
                raw=raw,
            )

        if msg_id == COMM.RESP_MAG_FIELD and len(parts) == 6:
            # >6,{id},{bx},{by},{bz},{timestamp_ms}
            return ParsedMessage(
                msg_type=COMM.RESP_MAG_FIELD,
                piece_id=int(parts[1], 0),
                mag_x=float(parts[2]),
                mag_y=float(parts[3]),
                mag_z=float(parts[4]),
                timestamp_ms=int(parts[5]),
                raw=raw,
            )

        if msg_id == COMM.RESP_PONG and len(parts) == 1:
            # >255
            return ParsedMessage(msg_type=COMM.RESP_PONG, raw=raw)

    except (ValueError, IndexError):
        pass

    return ParsedMessage(msg_type="UNKNOWN", raw=raw)

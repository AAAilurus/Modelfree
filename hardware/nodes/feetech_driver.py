#!/usr/bin/env python3
"""
Feetech STS3215 servo driver for SO-100 / SO-101 2-DOF arms.

Implements the Feetech packet protocol (v1, same wire format as
Dynamixel Protocol 1) over a single RS-485 / UART serial port.

Key registers used:
  TORQUE_ENABLE    = 40  (0x28) – 1 byte
  GOAL_POSITION_L  = 42  (0x2A) – 2 bytes (little-endian)
  GOAL_SPEED_L     = 46  (0x2E) – 2 bytes (little-endian, speed 0=max)
  PRESENT_POSITION = 56  (0x38) – 2 bytes
  PRESENT_SPEED    = 58  (0x3A) – 2 bytes (bit-15 = direction)

Position encoding:
  raw 0    ↔  -π rad
  raw 2048 ↔   0 rad
  raw 4095 ↔  +π rad
"""

import math
import time
import logging

try:
    import serial
except ImportError:
    serial = None  # allow import without pyserial for unit-testing stubs

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Instruction codes
# ---------------------------------------------------------------------------
INST_PING       = 0x01
INST_READ       = 0x02
INST_WRITE      = 0x03
INST_SYNC_WRITE = 0x83

# ---------------------------------------------------------------------------
# Register addresses (STS3215 / STS3032)
# ---------------------------------------------------------------------------
ADDR_TORQUE_ENABLE   = 40   # 0x28
ADDR_GOAL_POSITION_L = 42   # 0x2A  (2 bytes, LE)
ADDR_GOAL_SPEED_L    = 46   # 0x2E  (2 bytes, LE)
ADDR_PRESENT_POS_L   = 56   # 0x38  (2 bytes, LE)
ADDR_PRESENT_SPD_L   = 58   # 0x3A  (2 bytes, LE, bit-15 = direction)

# ---------------------------------------------------------------------------
# Position range
# ---------------------------------------------------------------------------
SERVO_MIN    = 0
SERVO_MAX    = 4095
SERVO_CENTER = 2048   # corresponds to 0 rad


# ---------------------------------------------------------------------------
# Unit-conversion helpers
# ---------------------------------------------------------------------------

def rad_to_raw(angle_rad: float) -> int:
    """Convert joint angle (radians) to servo raw count [0..4095]."""
    raw = int(round((angle_rad / math.pi) * 2047.0 + SERVO_CENTER))
    return max(SERVO_MIN, min(SERVO_MAX, raw))


def raw_to_rad(raw: int) -> float:
    """Convert servo raw count [0..4095] to joint angle (radians)."""
    return (raw - SERVO_CENTER) / 2047.0 * math.pi


def raw_speed_to_rad_s(raw: int, resolution: float = 0.00290888) -> float:
    """
    Convert servo raw speed count to rad/s.

    Bit-15 of raw encodes direction; bits 0-14 encode magnitude.
    Default resolution ≈ 0.0029 rad/s per unit (empirical for STS3215 at 1 Mbaud).
    """
    direction = 1 if (raw & 0x8000) == 0 else -1
    magnitude = raw & 0x7FFF
    return direction * magnitude * resolution


# ---------------------------------------------------------------------------
# Packet builder / parser
# ---------------------------------------------------------------------------

def _checksum(packet_bytes: bytes) -> int:
    """Feetech checksum: ~(sum of bytes from ID to last param) & 0xFF."""
    return (~sum(packet_bytes)) & 0xFF


def build_write_packet(servo_id: int, address: int, data: bytes) -> bytes:
    """Build a WRITE instruction packet."""
    length = len(data) + 3  # INST + ADDR + CHECKSUM
    body = bytes([servo_id, length, INST_WRITE, address]) + data
    cs = _checksum(body)
    return b'\xff\xff' + body + bytes([cs])


def build_read_packet(servo_id: int, address: int, length: int) -> bytes:
    """Build a READ instruction packet."""
    body = bytes([servo_id, 4, INST_READ, address, length])
    cs = _checksum(body)
    return b'\xff\xff' + body + bytes([cs])


def parse_response(data: bytes, expected_params: int):
    """
    Parse a servo STATUS packet.

    Returns (error_code, param_bytes) or raises ValueError on bad checksum.
    """
    # minimum packet: 0xFF 0xFF ID LEN ERROR CHECKSUM = 6 bytes
    if len(data) < 6:
        raise ValueError(f"Response too short: {len(data)} bytes")

    servo_id   = data[2]
    length     = data[3]
    error      = data[4]
    params     = data[5:5 + length - 2]   # LEN = n_params + 2
    checksum   = data[5 + length - 2]

    body = data[2:5 + length - 2]
    expected_cs = _checksum(body)
    if checksum != expected_cs:
        raise ValueError(
            f"Checksum mismatch: got {checksum:#04x}, expected {expected_cs:#04x}"
        )
    return error, params


# ---------------------------------------------------------------------------
# Main driver class
# ---------------------------------------------------------------------------

class FeetechDriver:
    """
    Low-level Feetech serial driver for a single bus (one serial port,
    potentially multiple servos on the same RS-485 line).

    Usage::

        drv = FeetechDriver('/dev/serial/by-id/usb-...-if00', baud=1_000_000)
        drv.open()
        drv.enable_torque(servo_id=1)
        pos_rad = drv.read_position(servo_id=1)
        drv.write_position(servo_id=1, angle_rad=0.5)
        drv.close()
    """

    def __init__(
        self,
        port: str,
        baud: int = 1_000_000,
        read_timeout: float = 0.10,
        max_retries: int = 2,
    ):
        if serial is None:
            raise RuntimeError(
                "pyserial is not installed.  Run: pip install pyserial"
            )
        self.port         = port
        self.baud         = baud
        self.read_timeout = read_timeout
        self.max_retries  = max_retries
        self._ser         = None

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def open(self):
        """Open the serial port."""
        self._ser = serial.Serial(
            port        = self.port,
            baudrate    = self.baud,
            bytesize    = serial.EIGHTBITS,
            parity      = serial.PARITY_NONE,
            stopbits    = serial.STOPBITS_ONE,
            timeout     = self.read_timeout,
        )
        logger.debug("Opened serial port %s @ %d baud", self.port, self.baud)

    def close(self):
        """Close the serial port."""
        if self._ser and self._ser.is_open:
            self._ser.close()
            logger.debug("Closed serial port %s", self.port)

    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    # ------------------------------------------------------------------
    # Low-level send / receive
    # ------------------------------------------------------------------

    def _send(self, packet: bytes):
        self._ser.reset_input_buffer()
        self._ser.write(packet)

    def _recv(self, n_params: int) -> tuple:
        """Read one status packet and return (error, params).

        Retries up to ``self.max_retries`` times on short reads so that
        transient bus timing glitches do not immediately propagate as errors.
        """
        expected = 6 + n_params  # header(2) + ID + LEN + ERROR + params + CS
        raw = self._ser.read(expected)
        # Retry on short reads (transient bus timing issue)
        for _ in range(self.max_retries):
            if len(raw) >= expected:
                break
            time.sleep(0.005)
            raw += self._ser.read(expected - len(raw))
        if len(raw) < 6:
            raise IOError(
                f"Short read: expected {expected} bytes, got {len(raw)}"
            )
        return parse_response(raw, n_params)

    # ------------------------------------------------------------------
    # Higher-level servo commands
    # ------------------------------------------------------------------

    def ping(self, servo_id: int) -> bool:
        """Ping a servo; returns True if it responds."""
        pkt = b'\xff\xff' + bytes([servo_id, 2, INST_PING])
        pkt += bytes([_checksum(bytes([servo_id, 2, INST_PING]))])
        try:
            self._send(pkt)
            err, _ = self._recv(0)
            return err == 0
        except (IOError, ValueError):
            return False

    def enable_torque(self, servo_id: int):
        """Enable servo torque (motor powered)."""
        pkt = build_write_packet(servo_id, ADDR_TORQUE_ENABLE, bytes([1]))
        self._send(pkt)
        try:
            self._recv(0)
        except IOError:
            pass  # some servos don't send a status response

    def disable_torque(self, servo_id: int):
        """Disable servo torque (motor free to rotate)."""
        pkt = build_write_packet(servo_id, ADDR_TORQUE_ENABLE, bytes([0]))
        self._send(pkt)
        try:
            self._recv(0)
        except IOError:
            pass

    def read_position(self, servo_id: int) -> float:
        """
        Read present position.

        Returns joint angle in radians, or None on failure.
        """
        pkt = build_read_packet(servo_id, ADDR_PRESENT_POS_L, 2)
        self._send(pkt)
        try:
            err, params = self._recv(2)
            if err != 0:
                logger.warning("Servo %d position read error: %d", servo_id, err)
                return None
            raw = params[0] | (params[1] << 8)
            return raw_to_rad(raw)
        except (IOError, ValueError) as e:
            logger.warning("read_position servo %d: %s", servo_id, e)
            return None

    def read_speed(self, servo_id: int) -> float:
        """
        Read present speed.

        Returns angular velocity in rad/s, or 0.0 on failure.
        """
        pkt = build_read_packet(servo_id, ADDR_PRESENT_SPD_L, 2)
        self._send(pkt)
        try:
            err, params = self._recv(2)
            if err != 0:
                return 0.0
            raw = params[0] | (params[1] << 8)
            return raw_speed_to_rad_s(raw)
        except (IOError, ValueError):
            return 0.0

    def read_pos_and_speed(self, servo_id: int):
        """
        Read present position AND speed in one request (4 consecutive bytes
        starting at ADDR_PRESENT_POS_L).

        Returns (angle_rad, speed_rad_s) or (None, 0.0) on failure.
        """
        pkt = build_read_packet(servo_id, ADDR_PRESENT_POS_L, 4)
        self._send(pkt)
        try:
            err, params = self._recv(4)
            if err != 0 or len(params) < 4:
                return None, 0.0
            raw_pos = params[0] | (params[1] << 8)
            raw_spd = params[2] | (params[3] << 8)
            return raw_to_rad(raw_pos), raw_speed_to_rad_s(raw_spd)
        except (IOError, ValueError) as e:
            logger.warning("read_pos_and_speed servo %d: %s", servo_id, e)
            return None, 0.0

    def write_position(
        self,
        servo_id: int,
        angle_rad: float,
        speed_raw: int = 0,
    ):
        """
        Write goal position.

        Parameters
        ----------
        angle_rad : desired joint angle in radians
        speed_raw : goal speed 0 = max speed, 1-32767 = limited speed
        """
        raw_pos = rad_to_raw(angle_rad)
        pos_lo  = raw_pos & 0xFF
        pos_hi  = (raw_pos >> 8) & 0xFF
        spd_lo  = speed_raw & 0xFF
        spd_hi  = (speed_raw >> 8) & 0xFF
        # Write goal position + goal time (4 bytes from 0x2A)
        data = bytes([pos_lo, pos_hi, 0, 0, spd_lo, spd_hi])
        pkt = build_write_packet(servo_id, ADDR_GOAL_POSITION_L, data)
        self._send(pkt)
        try:
            self._recv(0)
        except IOError:
            pass

    def sync_write_positions(self, id_angle_pairs: list, speed_raw: int = 0):
        """
        Write goal positions to multiple servos in a single SYNC_WRITE packet
        (more efficient than individual writes).

        Parameters
        ----------
        id_angle_pairs : list of (servo_id, angle_rad)
        speed_raw      : goal speed (0 = max)
        """
        if not id_angle_pairs:
            return

        # Each servo entry: ID + pos_lo + pos_hi + time_lo + time_hi + spd_lo + spd_hi = 7 bytes
        data_len = 6  # bytes per servo: pos(2) + time(2) + spd(2)
        length = 4 + len(id_angle_pairs) * (data_len + 1)  # INST + ADDR + DATA_LEN + N*(ID+data)

        body_parts = [INST_SYNC_WRITE, ADDR_GOAL_POSITION_L, data_len]
        for servo_id, angle_rad in id_angle_pairs:
            raw_pos = rad_to_raw(angle_rad)
            body_parts += [
                servo_id,
                raw_pos & 0xFF, (raw_pos >> 8) & 0xFF,  # goal position
                0, 0,                                     # goal time (not used)
                speed_raw & 0xFF, (speed_raw >> 8) & 0xFF,  # goal speed
            ]

        # SYNC_WRITE uses broadcast ID 0xFE
        header = bytes([0xFE, length])
        body   = bytes(body_parts)
        cs     = _checksum(header + body)
        pkt    = b'\xff\xff' + header + body + bytes([cs])
        self._send(pkt)
        # No status response for broadcast packets

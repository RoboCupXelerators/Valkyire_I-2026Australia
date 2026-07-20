"""Standalone Pi USB CDC driver for the C-board and four DM2325 motors.

This module intentionally contains its own minimal frame codec, single-owner
serial transport, IMU snapshot model, and four-wheel status model. Its only
non-standard runtime dependency is the already-required pyserial package.
"""
from __future__ import annotations
import math
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional
PROTOCOL_VERSION = 1
FRAME_HEADER = b'U\xaa'
FRAME_TAIL = 13
MAX_PAYLOAD = 250

# Production 80/100 degree chassis geometry. Wheel order is always FL, FR,
# RL, RR; positive motor speed follows each wheel's configured drive vector.
CHASSIS_WHEEL_RADIUS_M = 0.03
CHASSIS_WHEEL_X_M = (0.046345, 0.046345, -0.038889, -0.038889)
CHASSIS_WHEEL_Y_M = (0.038889, -0.038889, 0.046345, -0.046345)
CHASSIS_DRIVE_ANGLE_RAD = (2.268928028, 0.872664626, 3.839724354, -0.698131701)

class MsgType:
    PING = 1
    PONG = 2
    ACK = 10
    NACK = 11
    ERROR = 12
    CHASSIS_TWIST = 24
    CHASSIS_ARM = 25
    CHASSIS_STOP = 26
    CHASSIS_DISABLE = 27
    GET_CHASSIS_STATUS = 28
    CHASSIS_STATUS = 29
    CHASSIS_REFRESH_DISABLED = 35
    CHASSIS_CLEAR_FAULT = 36
    CHASSIS_EMERGENCY_STOP = 37
    GET_IMU_STATUS = 38
    IMU_STATUS = 39
    IMU_ZERO_YAW = 40
    IMU_SET_REFERENCE = 41
    CHASSIS_SET_PROFILE = 42
    GET_CHASSIS_DIAGNOSTICS = 43
    CHASSIS_DIAGNOSTICS = 44
    _names = {PING: 'PING', PONG: 'PONG', ACK: 'ACK', NACK: 'NACK', ERROR: 'ERROR', CHASSIS_TWIST: 'CHASSIS_TWIST', CHASSIS_ARM: 'CHASSIS_ARM', CHASSIS_STOP: 'CHASSIS_STOP', CHASSIS_DISABLE: 'CHASSIS_DISABLE', GET_CHASSIS_STATUS: 'GET_CHASSIS_STATUS', CHASSIS_STATUS: 'CHASSIS_STATUS', CHASSIS_REFRESH_DISABLED: 'CHASSIS_REFRESH_DISABLED', CHASSIS_CLEAR_FAULT: 'CHASSIS_CLEAR_FAULT', CHASSIS_EMERGENCY_STOP: 'CHASSIS_EMERGENCY_STOP', GET_IMU_STATUS: 'GET_IMU_STATUS', IMU_STATUS: 'IMU_STATUS', IMU_ZERO_YAW: 'IMU_ZERO_YAW', IMU_SET_REFERENCE: 'IMU_SET_REFERENCE', CHASSIS_SET_PROFILE: 'CHASSIS_SET_PROFILE', GET_CHASSIS_DIAGNOSTICS: 'GET_CHASSIS_DIAGNOSTICS', CHASSIS_DIAGNOSTICS: 'CHASSIS_DIAGNOSTICS'}

    @classmethod
    def name(cls, value):
        return cls._names.get(value, f'UNKNOWN(0x{value:02X})')

class NackReason:
    CRC_ERROR = 1
    BAD_VERSION = 2
    UNKNOWN_MSG = 3
    BAD_PAYLOAD = 4
    BAD_MOTOR_ID = 5
    NOT_ENABLED = 6
    OUT_OF_RANGE = 7
    BUSY = 8
    INTERNAL_ERROR = 9
    WATCHDOG_TIMEOUT = 10
    _names = {1: 'CRC_ERROR', 2: 'BAD_VERSION', 3: 'UNKNOWN_MSG', 4: 'BAD_PAYLOAD', 5: 'BAD_MOTOR_ID', 6: 'NOT_ENABLED', 7: 'OUT_OF_RANGE', 8: 'BUSY', 9: 'INTERNAL_ERROR', 10: 'WATCHDOG_TIMEOUT'}

    @classmethod
    def name(cls, val: int) -> str:
        return cls._names.get(val, f'UNKNOWN(0x{val:02X})')

def crc16(data: bytes, init: int=65535) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 32768:
                crc = crc << 1 ^ 4129
            else:
                crc <<= 1
            crc &= 65535
    return crc

@dataclass
class Frame:
    version: int = PROTOCOL_VERSION
    msg_type: int = 0
    seq: int = 0
    payload: bytes = b''

    def __post_init__(self):
        if self.payload is None:
            self.payload = b''
        if len(self.payload) > MAX_PAYLOAD:
            raise ValueError(f'Payload too large: {len(self.payload)} > {MAX_PAYLOAD}')

    @property
    def msg_name(self) -> str:
        return MsgType.name(self.msg_type)

    def encode(self) -> bytes:
        body = struct.pack('<BBBB', self.version, self.msg_type, self.seq & 255, len(self.payload))
        pre_crc = FRAME_HEADER + body + self.payload
        crc = crc16(pre_crc)
        return pre_crc + struct.pack('<H', crc) + bytes([FRAME_TAIL])

def decode_ack(payload: bytes) -> dict:
    if len(payload) < 2:
        raise ValueError('ACK payload too short')
    seq, msg_type = struct.unpack('<BB', payload[:2])
    return {'ack_seq': seq, 'ack_msg_type': msg_type, 'ack_msg_name': MsgType.name(msg_type)}

def decode_nack(payload: bytes) -> dict:
    if len(payload) < 3:
        raise ValueError('NACK payload too short')
    seq, msg_type, reason = struct.unpack('<BBB', payload[:3])
    return {'nack_seq': seq, 'nack_msg_type': msg_type, 'nack_reason': reason, 'nack_reason_name': NackReason.name(reason)}

class FrameParser:

    def __init__(self):
        self._buf = bytearray()
        self._discard_count = 0

    def feed(self, data: bytes):
        self._buf.extend(data)

    def next_frame(self) -> Optional[Frame]:
        while self._buf:
            hdr_idx = self._find_header()
            if hdr_idx < 0:
                self._discard_count += len(self._buf)
                self._buf.clear()
                return None
            if hdr_idx > 0:
                self._discard_count += hdr_idx
                del self._buf[:hdr_idx]
            if len(self._buf) < 6:
                return None
            version = self._buf[2]
            msg_type = self._buf[3]
            seq = self._buf[4]
            payload_len = self._buf[5]
            if payload_len > MAX_PAYLOAD:
                self._discard_count += 1
                del self._buf[:1]
                continue
            total = 2 + 4 + payload_len + 2 + 1
            if len(self._buf) < total:
                return None
            payload = bytes(self._buf[6:6 + payload_len])
            crc_recv = struct.unpack('<H', self._buf[6 + payload_len:8 + payload_len])[0]
            tail = self._buf[8 + payload_len]
            pre_crc = bytes(self._buf[:6 + payload_len])
            crc_calc = crc16(pre_crc)
            del self._buf[:total]
            if tail != FRAME_TAIL:
                self._discard_count += total
                continue
            if crc_recv != crc_calc:
                self._discard_count += total
                continue
            return Frame(version=version, msg_type=msg_type, seq=seq, payload=payload)
        return None

    def _find_header(self) -> int:
        for i in range(len(self._buf) - 1):
            if self._buf[i] == 85 and self._buf[i + 1] == 170:
                return i
        if self._buf and self._buf[-1] == 85:
            return len(self._buf) - 1
        return -1

    @property
    def discard_count(self) -> int:
        return self._discard_count

    def reset(self):
        self._buf.clear()
        self._discard_count = 0
CHASSIS_TWIST_FORMAT = '<IfffHBB'
CHASSIS_CONTROL_MODE_VELOCITY = 1

def encode_chassis_twist(host_monotonic_ms: int, vx_mps: float, vy_mps: float, wz_rad_s: float, valid_for_ms: int, control_mode: int=CHASSIS_CONTROL_MODE_VELOCITY, flags: int=0) -> bytes:
    values = (float(vx_mps), float(vy_mps), float(wz_rad_s))
    if not all((math.isfinite(value) for value in values)):
        raise ValueError('chassis twist must be finite')
    if not 1 <= int(valid_for_ms) <= 1000:
        raise ValueError('valid_for_ms must be in [1, 1000]')
    if int(control_mode) != CHASSIS_CONTROL_MODE_VELOCITY:
        raise ValueError('unsupported chassis control mode')
    return struct.pack(CHASSIS_TWIST_FORMAT, int(host_monotonic_ms) & 4294967295, *values, int(valid_for_ms), int(control_mode), int(flags) & 255)
CHASSIS_WHEEL_NAMES = ('FL', 'FR', 'RL', 'RR')
CHASSIS_STATUS_PAYLOAD_LEN = 248
CHASSIS_DIAGNOSTICS_PAYLOAD_LEN = 208
IMU_STATUS_PAYLOAD_LEN = 156
CHASSIS_PROFILES = {'safe': 0, 'normal': 1, 'competition_candidate': 2}

def decode_chassis_diagnostics(payload: bytes) -> dict:
    if len(payload) != CHASSIS_DIAGNOSTICS_PAYLOAD_LEN:
        raise ValueError('CHASSIS_DIAGNOSTICS payload must be 208 bytes')
    version, profile, control_mode, reverse_mask, board_tick_ms = struct.unpack_from('<BBBBI', payload, 0)
    command_age_ms, control_period_ms = struct.unpack_from('<HH', payload, 8)
    values = struct.unpack_from('<8f', payload, 12)
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError('invalid chassis profile limits')
    names = ('normal_accel_rad_s2', 'normal_decel_rad_s2', 'reverse_brake_rad_s2',
             'normal_jerk_rad_s3', 'reverse_jerk_rad_s3', 'zero_speed_epsilon_rad_s',
             'reverse_detection_epsilon_rad_s', 'velocity_deadband_rad_s')
    wheels = []
    for index, wheel_name in enumerate(CHASSIS_WHEEL_NAMES):
        offset = 44 + index * 28
        motor_id, reverse_state, limiter_state, supported = struct.unpack_from('<BBBB', payload, offset)
        target, requested, measured, error, measured_torque = struct.unpack_from('<5f', payload, offset + 4)
        feedback_age_ms, fault_code, enabled = struct.unpack_from('<HBB', payload, offset + 24)
        fresh = feedback_age_ms <= 100
        wheels.append({'wheel': wheel_name, 'motor_id': motor_id, 'enabled': bool(enabled),
            'control_mode': 'MIT', 'target_velocity_rad_s': target,
            'requested_velocity_rad_s': requested,
            'velocity_feedback_supported': bool(supported & 1),
            'torque_feedback_supported': bool(supported & 2),
            'measured_velocity_rad_s': measured if fresh and supported & 1 else None,
            'velocity_error_rad_s': error if fresh and supported & 1 else None,
            'measured_torque_nm': measured_torque if fresh and supported & 2 else None,
            'measured_current_a': None, 'bus_voltage_v': None,
            'feedback_age_ms': feedback_age_ms, 'fault_code': fault_code,
            'reverse_state': reverse_state, 'limiter_state': limiter_state,
            'saturation_flags': 1 if limiter_state == 3 else 0})
    event_values = struct.unpack_from('<5I4f4B3I', payload, 156)
    event = None
    if event_values[0]:
        timestamp = lambda value: None if value in (0, 0xFFFFFFFF) else value
        latency = lambda value: None if value == 0xFFFFFFFF else value
        event = {'event_id': event_values[0], 'command_received_ms': event_values[1],
            'first_brake_command_ms': event_values[2], 'zero_cross_ms': timestamp(event_values[3]),
            'reverse_velocity_reached_ms': timestamp(event_values[4]),
            'old_target_velocity_rad_s': event_values[5], 'new_target_velocity_rad_s': event_values[6],
            'measured_velocity_at_command_rad_s': event_values[7],
            'peak_measured_torque_nm': event_values[8], 'motor_id': event_values[9],
            'profile': event_values[10], 'result': event_values[11], 'fault_code': event_values[12],
            'command_to_brake_latency_ms': latency(event_values[13]),
            'brake_to_zero_cross_ms': latency(event_values[14]),
            'zero_cross_to_reverse_response_ms': latency(event_values[15]),
            'peak_current_a': None, 'peak_bus_voltage_v': None}
    return {'version': version, 'profile': profile,
        'profile_name': next((name for name, value in CHASSIS_PROFILES.items() if value == profile), 'unknown'),
        'control_mode': control_mode, 'reverse_mask': reverse_mask, 'board_tick_ms': board_tick_ms,
        'command_age_ms': command_age_ms, 'control_period_ms': control_period_ms,
        'limits': dict(zip(names, values)), 'torque_limit_nm': None, 'current_limit_a': None,
        'wheels': wheels, 'latest_reverse_event': event, 'raw_payload_hex': payload.hex()}

def encode_imu_set_reference(angle_deg: float) -> bytes:
    angle = float(angle_deg)
    if not math.isfinite(angle):
        raise ValueError('IMU yaw reference must be finite')
    return struct.pack('<f', angle)

def decode_imu_status(payload: bytes) -> dict:
    if len(payload) != IMU_STATUS_PAYLOAD_LEN:
        raise ValueError('IMU_STATUS payload must be 156 bytes')
    sequence, timestamp_us, age_us, flags, sample_count, error_count, missed_sample_count, spi_error_count, movement_rejection_count, sample_interval_min_us, sample_interval_max_us, update_execution_max_us = struct.unpack_from('<12I', payload, 0)
    state, calibration_state, source, accel_chip_id, gyro_chip_id = struct.unpack_from('<5B', payload, 48)
    calibration_sample_count = struct.unpack_from('<H', payload, 54)[0]
    accel_mps2 = struct.unpack_from('<3f', payload, 56)
    gyro_rad_s = struct.unpack_from('<3f', payload, 68)
    mag = struct.unpack_from('<3f', payload, 80)
    quaternion_wxyz = struct.unpack_from('<4f', payload, 92)
    roll_rad, pitch_rad, yaw_rad, yaw_continuous_rad, gyro_z_rad_s, temperature_c = struct.unpack_from('<6f', payload, 108)
    gyro_bias_rad_s = struct.unpack_from('<3f', payload, 132)
    calibration_gyro_variance = struct.unpack_from('<3f', payload, 144)
    floats = (*accel_mps2, *gyro_rad_s, *mag, *quaternion_wxyz, roll_rad, pitch_rad, yaw_rad, yaw_continuous_rad, gyro_z_rad_s, temperature_c, *gyro_bias_rad_s, *calibration_gyro_variance)
    if not all((math.isfinite(value) for value in floats)):
        raise ValueError('IMU_STATUS contains NaN or Inf')
    return {'sequence': sequence, 'cboard_timestamp_us': timestamp_us, 'age_us': age_us, 'flags': flags, 'sample_count': sample_count, 'error_count': error_count, 'missed_sample_count': missed_sample_count, 'spi_error_count': spi_error_count, 'movement_rejection_count': movement_rejection_count, 'sample_interval_min_us': sample_interval_min_us, 'sample_interval_max_us': sample_interval_max_us, 'update_execution_max_us': update_execution_max_us, 'state': state, 'calibration_state': calibration_state, 'source_code': source, 'accel_chip_id': accel_chip_id, 'gyro_chip_id': gyro_chip_id, 'calibration_sample_count': calibration_sample_count, 'accel_mps2': accel_mps2, 'gyro_rad_s': gyro_rad_s, 'mag': mag, 'quaternion_wxyz': quaternion_wxyz, 'roll_rad': roll_rad, 'pitch_rad': pitch_rad, 'yaw_rad': yaw_rad, 'yaw_continuous_rad': yaw_continuous_rad, 'gyro_z_rad_s': gyro_z_rad_s, 'temperature_c': temperature_c, 'gyro_bias_rad_s': gyro_bias_rad_s, 'calibration_gyro_variance': calibration_gyro_variance, 'raw_payload_hex': payload.hex()}

def decode_chassis_status(payload: bytes) -> dict:
    if len(payload) != CHASSIS_STATUS_PAYLOAD_LEN:
        raise ValueError('CHASSIS_STATUS payload must be 248 bytes')
    state, online_mask, enabled_mask, fault_mask, board_tick_ms, can_esr = struct.unpack_from('<BBBBII', payload, 0)
    wheels = []
    for index, name in enumerate(CHASSIS_WHEEL_NAMES):
        offset = 12 + index * 52
        esc_id, master_id, flags, error_code, temp_mos, temp_rotor = struct.unpack_from('<HHBBBB', payload, offset)
        position, speed, torque = struct.unpack_from('<fff', payload, offset + 8)
        position_raw, speed_raw, torque_raw = struct.unpack_from('<HHH', payload, offset + 20)
        feedback_sequence, feedback_tick_ms = struct.unpack_from('<II', payload, offset + 28)
        position_unwrapped, feedback_age_ms, rx_count, invalid_count, stale_count, last_raw_can_id = struct.unpack_from('<fHIHHH', payload, offset + 36)
        wheels.append({'wheel': name, 'esc_id': esc_id, 'master_id': master_id, 'enabled': bool(flags & 1), 'online': bool(flags & 2), 'fault': bool(flags & 4), 'error_code': error_code, 'temperature_mos_c': temp_mos, 'temperature_rotor_c': temp_rotor, 'position_rad': position, 'speed_rad_s': speed, 'torque_nm': torque, 'position_raw': position_raw, 'speed_raw': speed_raw, 'torque_raw': torque_raw, 'feedback_sequence': feedback_sequence, 'feedback_tick_ms': feedback_tick_ms, 'position_unwrapped_rad': position_unwrapped, 'feedback_age_ms': feedback_age_ms, 'rx_count': rx_count, 'invalid_count': invalid_count, 'stale_count': stale_count, 'last_raw_can_id': last_raw_can_id})
    control_sequence, control_tick_count, can_tx_failure_count, max_command_skew_us, scheduler_lateness_max_us, runtime_fault, id_mismatch_count = struct.unpack_from('<IIIIIII', payload, 220)
    return {'state': state, 'online_mask': online_mask, 'enabled_mask': enabled_mask, 'fault_mask': fault_mask, 'board_tick_ms': board_tick_ms, 'can_esr': can_esr, 'wheels': wheels, 'control_sequence': control_sequence, 'control_tick_count': control_tick_count, 'can_tx_failure_count': can_tx_failure_count, 'max_command_skew_us': max_command_skew_us, 'scheduler_lateness_max_us': scheduler_lateness_max_us, 'runtime_fault': runtime_fault, 'id_mismatch_count': id_mismatch_count}

@dataclass(frozen=True)
class WheelMotorStatus:
    wheel: str
    esc_id: int
    master_id: int
    online: bool
    enabled: bool
    fault: bool
    fault_code: int
    position_rad: float
    speed_rad_s: float
    torque_nm: float
    temperature_c: float
    feedback_sequence: int
    feedback_age_ms: float
    rotor_temperature_c: float | None = None
    position_unwrapped_rad: float = 0.0
    rx_count: int = 0
    invalid_count: int = 0
    stale_count: int = 0
    last_raw_can_id: int = 0
    source: str = 'hardware'

@dataclass(frozen=True)
class ChassisVelocityEstimate:
    vx_mps: float
    vy_mps: float
    wz_rad_s: float
    speed_mps: float
    motor_rad_s: tuple[float, float, float, float]
    source: str = 'wheel_encoder_80_100'

def _solve_3x3(matrix, vector):
    augmented = [list(row) + [float(value)] for row, value in zip(matrix, vector)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise RuntimeError('80/100 chassis geometry is singular')
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        for index in range(column, 4):
            augmented[column][index] /= divisor
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            for index in range(column, 4):
                augmented[row][index] -= factor * augmented[column][index]
    return tuple(augmented[index][3] for index in range(3))

def estimate_chassis_velocity(motor_rad_s) -> ChassisVelocityEstimate:
    """Estimate body velocity from FL/FR/RL/RR encoder speeds.

    Linear velocity comes from motor feedback, not IMU integration. This keeps
    steady-speed measurement useful even when the optional IMU is not mounted.
    """
    speeds = tuple(float(value) for value in motor_rad_s)
    if len(speeds) != 4:
        raise ValueError('four FL/FR/RL/RR motor speeds are required')
    if not all(math.isfinite(value) for value in speeds):
        raise ValueError('motor speeds must be finite')
    matrix = []
    for x_m, y_m, angle in zip(
            CHASSIS_WHEEL_X_M, CHASSIS_WHEEL_Y_M, CHASSIS_DRIVE_ANGLE_RAD):
        ux, uy = math.cos(angle), math.sin(angle)
        matrix.append((
            ux / CHASSIS_WHEEL_RADIUS_M,
            uy / CHASSIS_WHEEL_RADIUS_M,
            (-y_m * ux + x_m * uy) / CHASSIS_WHEEL_RADIUS_M,
        ))
    normal = [[sum(row[left] * row[right] for row in matrix)
               for right in range(3)] for left in range(3)]
    projected = [sum(row[column] * speed for row, speed in zip(matrix, speeds))
                 for column in range(3)]
    vx_mps, vy_mps, wz_rad_s = _solve_3x3(normal, projected)
    return ChassisVelocityEstimate(
        vx_mps=vx_mps,
        vy_mps=vy_mps,
        wz_rad_s=wz_rad_s,
        speed_mps=math.hypot(vx_mps, vy_mps),
        motor_rad_s=speeds,
    )

@dataclass(frozen=True)
class FourMotorStatus:
    motors: tuple[WheelMotorStatus, WheelMotorStatus, WheelMotorStatus, WheelMotorStatus]
    snapshot_sequence: int
    received_monotonic: float
    online_mask: int
    enabled_mask: int
    fault_mask: int
    stale_mask: int
    can_esr: int
    runtime_state: int
    runtime_fault: int
    raw_payload: bytes
    source: str = 'hardware'

    @property
    def fl(self):
        return self.motors[0]

    @property
    def fr(self):
        return self.motors[1]

    @property
    def rl(self):
        return self.motors[2]

    @property
    def rr(self):
        return self.motors[3]

    @property
    def all_online(self):
        return self.online_mask == 15

    @property
    def all_disabled(self):
        return (
            self.all_online
            and not self.any_stale
            and self.enabled_mask == 0
            and all(not m.enabled and m.fault_code == 0 for m in self.motors)
        )

    @property
    def all_enabled(self):
        return (
            self.all_online
            and not self.any_stale
            and self.enabled_mask == 15
            # DM feedback state 1 means Enabled; state 0 means Disabled.
            and all(m.enabled and not m.fault and m.fault_code == 1 for m in self.motors)
        )

    @property
    def any_fault(self):
        return bool(self.fault_mask or self.runtime_fault)

    @property
    def any_stale(self):
        return bool(self.stale_mask)

    @property
    def velocity_estimate(self):
        return estimate_chassis_velocity(tuple(motor.speed_rad_s for motor in self.motors))

def status_from_protocol(decoded: dict, raw_payload: bytes, received_monotonic: float) -> FourMotorStatus:
    board_tick = decoded['board_tick_ms']
    motors = []
    stale_mask = 0
    for index, item in enumerate(decoded['wheels']):
        age = float(item.get('feedback_age_ms', board_tick - item['feedback_tick_ms'] & 4294967295))
        if not item['online'] or age > 100.0:
            stale_mask |= 1 << index
        motors.append(WheelMotorStatus(wheel=item['wheel'], esc_id=item['esc_id'], master_id=item['master_id'], online=item['online'] and age <= 100.0, enabled=item['enabled'], fault=item['fault'], fault_code=item['error_code'], position_rad=item['position_rad'], speed_rad_s=item['speed_rad_s'], torque_nm=item['torque_nm'], temperature_c=item['temperature_mos_c'], rotor_temperature_c=item['temperature_rotor_c'], feedback_sequence=item['feedback_sequence'], feedback_age_ms=age, position_unwrapped_rad=item.get('position_unwrapped_rad', item['position_rad']), rx_count=item.get('rx_count', item['feedback_sequence']), invalid_count=item.get('invalid_count', 0), stale_count=item.get('stale_count', 0), last_raw_can_id=item.get('last_raw_can_id', item['master_id'])))
    return FourMotorStatus(motors=tuple(motors), snapshot_sequence=decoded['control_sequence'], received_monotonic=received_monotonic, online_mask=decoded['online_mask'], enabled_mask=decoded['enabled_mask'], fault_mask=decoded['fault_mask'], stale_mask=stale_mask, can_esr=decoded['can_esr'], runtime_state=decoded['state'], runtime_fault=decoded['runtime_fault'], raw_payload=raw_payload)

class SerialTransport:
    """Serialized CDC transport with sequence-correlated replies."""

    def __init__(self, port, baudrate=115200, timeout=0.1, exclusive=False):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.exclusive = bool(exclusive)
        self._ser = None
        self._parser = FrameParser()
        self._pending = []
        # A web server may call the driver from different request/heartbeat
        # threads. Serialize complete exchanges instead of permanently tying
        # CDC RX ownership to whichever thread performed the first handshake.
        self._transaction_lock = threading.RLock()
        self._serial_rx_bytes = 0
        self._serial_tx_bytes = 0
        self._complete_frames = 0
        self._partial_reads = 0

    def open(self):
        import serial
        if self.is_open:
            return
        self._ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout, exclusive=self.exclusive)

    def close(self):
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self._pending.clear()
        self._parser.reset()

    @property
    def is_open(self):
        return self._ser is not None and bool(getattr(self._ser, 'is_open', False))

    def _read_once(self, timeout_s):
        available = int(getattr(self._ser, 'in_waiting', 0) or 0)
        requested = available if available else 1
        old_timeout = self._ser.timeout
        self._ser.timeout = max(0.0, min(0.005, timeout_s))
        try:
            chunk = self._ser.read(requested)
        finally:
            self._ser.timeout = old_timeout
        self._serial_rx_bytes += len(chunk)
        if chunk and len(chunk) < requested:
            self._partial_reads += 1
        return chunk

    def _poll_frames(self, timeout_s):
        if not self.is_open:
            self.open()
        deadline = time.monotonic() + max(0.0, timeout_s)
        while True:
            chunk = self._read_once(max(0.0, deadline - time.monotonic()))
            if chunk:
                self._parser.feed(chunk)
                while True:
                    frame = self._parser.next_frame()
                    if frame is None:
                        break
                    self._complete_frames += 1
                    self._pending.append(frame)
                if self._pending or not getattr(self._ser, 'in_waiting', 0):
                    return
            if time.monotonic() >= deadline:
                return
            time.sleep(0.001)

    @staticmethod
    def _matches(frame, request, expected_type):
        if frame.msg_type != expected_type or frame.seq != request.seq:
            return False
        if expected_type != MsgType.ACK:
            return True
        try:
            ack = decode_ack(frame.payload)
        except ValueError:
            return False
        return ack['ack_seq'] == request.seq and ack['ack_msg_type'] == request.msg_type

    def _take_matching(self, request, expected_type):
        for index, frame in enumerate(self._pending):
            if self._matches(frame, request, expected_type):
                return self._pending.pop(index)
            if frame.msg_type in (MsgType.NACK, MsgType.ERROR) and frame.seq == request.seq:
                return self._pending.pop(index)
        return None

    def send_and_wait(self, frame, expected_type, timeout_s=0.5):
        with self._transaction_lock:
            if not self.is_open:
                self.open()
            encoded = frame.encode()
            written = self._ser.write(encoded)
            self._serial_tx_bytes += len(encoded) if written is None else int(written)
            deadline = time.monotonic() + timeout_s
            while True:
                matched = self._take_matching(frame, expected_type)
                if matched is not None:
                    return matched
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._poll_frames(min(0.005, remaining))

    @property
    def stats(self):
        """Return read-only CDC diagnostics used by static acceptance tools."""
        return {
            'serial_rx_bytes': self._serial_rx_bytes,
            'serial_tx_bytes': self._serial_tx_bytes,
            'complete_frames': self._complete_frames,
            'partial_reads': self._partial_reads,
            'malformed_frames': self._parser.discard_count,
            'parser_buffer_length': len(self._parser._buf),
            'pending_frames': len(self._pending),
        }

@dataclass(frozen=True)
class IMUState:
    sequence: int
    cboard_timestamp: int
    received_monotonic: float
    accel_mps2: tuple[float, float, float]
    gyro_rad_s: tuple[float, float, float]
    mag: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    yaw_continuous_rad: float
    gyro_z_rad_s: float
    temperature_c: float
    calibration_state: int
    flags: int
    age_ms: float
    imu_online: bool
    imu_ready: bool
    imu_calibrated: bool
    imu_stale: bool
    imu_fault: bool
    imu_age_ms: float
    healthy: bool
    source: str
    state: int
    sample_count: int
    missed_sample_count: int
    spi_error_count: int
    raw: dict

class DM2325Bits:
    IMU_FLAG_FATAL = 1 << 0 | 1 << 1 | 1 << 3 | 1 << 4 | 1 << 6 | 1 << 10 | 1 << 11 | 1 << 12 | 1 << 13 | 1 << 14 | 1 << 15 | 1 << 16 | 1 << 17
    IMU_STATE_READY = 3
    IMU_STATE_FAULT = 5
    IMU_MAX_AGE_MS = 50.0
    IMU_SOURCE_NAMES = {1: 'GYRO_RELATIVE', 2: 'MAG_FUSED', 3: 'EXTERNAL_CORRECTED'}

    def __init__(self, port='/dev/ttyACM0', baudrate=115200, get_yaw=None, exclusive=False):
        self.port = port
        self.baudrate = baudrate
        self._legacy_get_yaw_override = get_yaw
        self.transport = SerialTransport(port, baudrate, timeout=0.005, exclusive=exclusive)
        self._sequence = 0
        self._closed = True
        self._last_imu_state = None
        self._shutdown_complete = False

    def _next_sequence(self):
        self._sequence = self._sequence + 1 & 255
        return self._sequence

    def _exchange(self, message_type, payload, expected, timeout=0.2):
        request = Frame(msg_type=message_type, seq=self._next_sequence(), payload=payload)
        response = self.transport.send_and_wait(request, expected, timeout)
        if response is None:
            raise RuntimeError(f'{MsgType.name(message_type)} timed out')
        if response.msg_type != expected:
            if response.msg_type == MsgType.NACK:
                raise RuntimeError(f'{MsgType.name(message_type)} rejected: {decode_nack(response.payload)}')
            raise RuntimeError(f'unexpected {MsgType.name(response.msg_type)}')
        return response

    def OpenPort(self):
        self.transport.open()
        self._closed = False
        self._shutdown_complete = False
        return True

    def ClosePort(self):
        self.transport.close()
        self._closed = True
        self._last_imu_state = None

    def HandShake(self):
        self.OpenPort()
        self._exchange(MsgType.PING, b'', MsgType.PONG)
        return True

    def GetYaw(self):
        state = self.GetIMUState()
        if not state.healthy:
            raise RuntimeError(f'C-board IMU unhealthy state={state.state} flags=0x{state.flags:08X} age_ms={state.age_ms:.1f}')
        return float(math.degrees(state.yaw_rad) % 360.0)

    def GetHeadingCandidate(self):
        """Return observable attitude without declaring a robot heading axis.

        Motion code must continue to use GetYaw(), whose health gate rejects an
        unverified installation. This candidate exists for diagnostics and the
        Control Panel while final board orientation is still undecided.
        """
        state = self.GetIMUState()
        data_healthy = bool(
            state.imu_online
            and state.imu_ready
            and state.imu_calibrated
            and not state.imu_stale
            and not state.imu_fault
        )
        if not data_healthy:
            raise RuntimeError(
                f'C-board IMU data unavailable state={state.state} '
                f'flags=0x{state.flags:08X} age_ms={state.age_ms:.1f}'
            )
        return {
            'heading_deg': float(math.degrees(state.yaw_rad) % 360.0),
            'source': 'FIRMWARE_ATTITUDE_CANDIDATE',
            'selected_axis': 'yaw',
            'attitude_candidates_deg': {
                'roll': float(math.degrees(state.roll_rad)),
                'pitch': float(math.degrees(state.pitch_rad)),
                'yaw': float(math.degrees(state.yaw_rad) % 360.0),
            },
            'axis_mapping_verified': False,
            'robot_direction_defined': False,
            'valid_for_motion': False,
            'direction_flags': list(IMU_DIRECTION_FLAGS),
        }

    def GetCachedYaw(self):
        """Return fresh cached hardware yaw without touching the CDC RX owner.

        This is intended for background consumers such as the lidar thread.
        The main control path remains responsible for requesting snapshots.
        """
        state = self._last_imu_state
        if state is None:
            raise RuntimeError('no C-board IMU snapshot has been received')
        effective_age_ms = state.age_ms + (time.monotonic() - state.received_monotonic) * 1000.0
        if not state.healthy or effective_age_ms > self.IMU_MAX_AGE_MS:
            raise RuntimeError(f'cached C-board IMU yaw is stale ({effective_age_ms:.1f} ms)')
        return float(math.degrees(state.yaw_rad) % 360.0)

    def GetYawRad(self):
        state = self.GetIMUState()
        if not state.healthy:
            raise RuntimeError('C-board IMU yaw is not healthy and fresh')
        return float(state.yaw_rad)

    def GetYawContinuous(self):
        state = self.GetIMUState()
        if not state.healthy:
            raise RuntimeError('C-board IMU continuous yaw is not healthy and fresh')
        return float(state.yaw_continuous_rad)

    def GetGyroscopeData(self):
        return self.GetIMUState().gyro_rad_s

    def GetGyroZ(self):
        """Return fresh, calibrated robot-frame yaw rate in rad/s."""
        state = self.GetIMUState()
        if not state.healthy:
            raise RuntimeError('C-board IMU gyro Z is not healthy and fresh')
        return float(state.gyro_z_rad_s)

    def GetAccelerometerData(self):
        return self.GetIMUState().accel_mps2

    def GetMagnetometerData(self):
        return self.GetIMUState().mag

    def GetIMUState(self):
        response = self._exchange(MsgType.GET_IMU_STATUS, b'', MsgType.IMU_STATUS)
        received = time.monotonic()
        raw = decode_imu_status(response.payload)
        age_ms = raw['age_us'] / 1000.0
        imu_online = raw['accel_chip_id'] == 30 and raw['gyro_chip_id'] == 15 and (not raw['flags'] & (1 << 0 | 1 << 1 | 1 << 3))
        imu_ready = raw['state'] == self.IMU_STATE_READY
        imu_calibrated = raw['calibration_state'] == 2
        imu_stale = age_ms > self.IMU_MAX_AGE_MS or bool(raw['flags'] & 1 << 4)
        imu_fault = raw['state'] == self.IMU_STATE_FAULT or bool(raw['flags'] & (self.IMU_FLAG_FATAL & ~(1 << 14)))
        healthy = imu_online and imu_ready and imu_calibrated and (not imu_stale) and (not raw['flags'] & self.IMU_FLAG_FATAL)
        state = IMUState(sequence=raw['sequence'], cboard_timestamp=raw['cboard_timestamp_us'], received_monotonic=received, accel_mps2=tuple(raw['accel_mps2']), gyro_rad_s=tuple(raw['gyro_rad_s']), mag=tuple(raw['mag']), quaternion=tuple(raw['quaternion_wxyz']), roll_rad=raw['roll_rad'], pitch_rad=raw['pitch_rad'], yaw_rad=raw['yaw_rad'], yaw_continuous_rad=raw['yaw_continuous_rad'], gyro_z_rad_s=raw['gyro_z_rad_s'], temperature_c=raw['temperature_c'], calibration_state=raw['calibration_state'], flags=raw['flags'], age_ms=age_ms, imu_online=imu_online, imu_ready=imu_ready, imu_calibrated=imu_calibrated, imu_stale=imu_stale, imu_fault=imu_fault, imu_age_ms=age_ms, healthy=healthy, source=self.IMU_SOURCE_NAMES.get(raw['source_code'], 'UNKNOWN'), state=raw['state'], sample_count=raw['sample_count'], missed_sample_count=raw['missed_sample_count'], spi_error_count=raw['spi_error_count'], raw=raw)
        self._last_imu_state = state
        return state

    def IsIMUHealthy(self):
        try:
            return self.GetIMUState().healthy
        except (RuntimeError, ValueError):
            return False

    def GetIMUAgeMs(self):
        return float(self.GetIMUState().age_ms)

    def ZeroYaw(self):
        self._exchange(MsgType.IMU_ZERO_YAW, b'', MsgType.ACK)
        return True

    def SetYawReference(self, angle_deg):
        payload = encode_imu_set_reference(float(angle_deg))
        self._exchange(MsgType.IMU_SET_REFERENCE, payload, MsgType.ACK)
        return True

    def GetChassisTelemetry(self, timeout=0.2):
        response = self._exchange(MsgType.GET_CHASSIS_STATUS, b'', MsgType.CHASSIS_STATUS, timeout)
        now = time.monotonic()
        return status_from_protocol(decode_chassis_status(response.payload), response.payload, now)

    def GetChassisDiagnostics(self, timeout=0.2):
        response = self._exchange(MsgType.GET_CHASSIS_DIAGNOSTICS, b'',
                                  MsgType.CHASSIS_DIAGNOSTICS, timeout)
        return decode_chassis_diagnostics(response.payload)

    def SetChassisProfile(self, profile):
        if isinstance(profile, str):
            key = profile.strip().lower()
            if key not in CHASSIS_PROFILES:
                raise ValueError(f'unsupported chassis profile: {profile}')
            profile = CHASSIS_PROFILES[key]
        value = int(profile)
        if value not in CHASSIS_PROFILES.values():
            raise ValueError(f'unsupported chassis profile: {profile}')
        self._exchange(MsgType.CHASSIS_SET_PROFILE, bytes((value,)), MsgType.ACK)
        return True

    def RefreshDisabledTelemetry(self):
        """Request fresh status while Disabled without enabling or changing parameters."""
        response = self._exchange(MsgType.CHASSIS_REFRESH_DISABLED, b'', MsgType.CHASSIS_STATUS)
        now = time.monotonic()
        return status_from_protocol(decode_chassis_status(response.payload), response.payload, now)

    def GetWheelFeedback(self):
        return self.GetChassisTelemetry().motors

    def EstimateChassisVelocity(self, status=None):
        status = self.GetChassisTelemetry() if status is None else status
        return status.velocity_estimate

    def GetMotorHealth(self):
        status = self.GetChassisTelemetry()
        return {'all_online': status.all_online, 'all_disabled': status.all_disabled, 'all_enabled': status.all_enabled, 'any_fault': status.any_fault, 'any_stale': status.any_stale, 'motors': status.motors}

    def Arm(self):
        disabled = self.RefreshDisabledTelemetry()
        if disabled.runtime_state != 1 or not disabled.all_disabled:
            raise RuntimeError('ARM requires four fresh Disabled confirmations')
        self._exchange(MsgType.CHASSIS_ARM, b'', MsgType.ACK)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            try:
                # ARMING rejects Twist with NOT_ENABLED. The first accepted
                # zero command is the READY handshake and refreshes the board
                # watchdog without introducing any non-zero motor target.
                self.SetChassisTwist(0.0, 0.0, 0.0)
            except RuntimeError as exc:
                if 'NOT_ENABLED' not in str(exc):
                    raise
                # Probe once per 10 ms C-board control period. Faster NACK
                # traffic can starve the CAN feedback that advances ARMING.
                time.sleep(0.01)
            else:
                # C firmware accepts CHASSIS_TWIST only after all four Enable
                # feedback records have advanced and READY has been entered.
                return True
        status = self.GetChassisTelemetry()
        if status.runtime_state == 6:
            raise RuntimeError(f'chassis ARM fault {status.runtime_fault}')
        raise RuntimeError('chassis ARM timeout')

    def _wait_for_state(self, expected_state, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.GetChassisTelemetry()
            if status.runtime_state == expected_state:
                return status
            if status.runtime_state == 6:
                raise RuntimeError(f'chassis fault {status.runtime_fault}')
            time.sleep(0.01)
        raise RuntimeError(f'chassis state {expected_state} confirmation timeout')

    def SetChassisTwist(self, vx, vy, wz):
        payload = encode_chassis_twist(int(time.monotonic() * 1000.0), float(vx), float(vy), float(wz), 100)
        self._exchange(MsgType.CHASSIS_TWIST, payload, MsgType.ACK, 0.05)

    def SetWheelSpeed(self, fl, fr, rl, rr):
        raise RuntimeError('direct wheel targets are engineering-only and unavailable in production')

    def _is_already_disabled(self):
        try:
            status = self.GetChassisTelemetry()
        except (RuntimeError, ValueError, TypeError, AttributeError):
            return False
        return bool(
            status.runtime_state in (1, 6)
            and status.enabled_mask == 0
            and status.all_disabled
        )

    def stop(self):
        try:
            self.SetChassisTwist(0.0, 0.0, 0.0)
        except RuntimeError as exc:
            if 'NOT_ENABLED' in str(exc) and self._is_already_disabled():
                return True
            raise
        self._exchange(MsgType.CHASSIS_STOP, b'', MsgType.ACK)
        # STOP ACK proves the runtime accepted the zero target while READY or
        # RUNNING. Avoid a STATUS round-trip inside the 100 ms host watchdog.
        return True

    def disable(self):
        try:
            self._exchange(MsgType.CHASSIS_DISABLE, b'', MsgType.ACK)
        except RuntimeError as exc:
            if 'BUSY' in str(exc) and self._is_already_disabled():
                return True
            raise
        status = self._wait_for_state(1)
        if status.enabled_mask != 0 or not status.all_disabled:
            raise RuntimeError('four-motor Disabled confirmation invalid')
        return True

    def emergency_stop(self):
        self._exchange(MsgType.CHASSIS_EMERGENCY_STOP, b'', MsgType.ACK)
        status = self._wait_for_state(6)
        if not status.all_disabled or status.runtime_fault != 8:
            raise RuntimeError('four-motor emergency-stop confirmation invalid')
        return True

    def ClearFault(self):
        """Clear a latched chassis fault only after four fresh Disabled replies."""
        status = self.RefreshDisabledTelemetry()
        if not status.all_disabled:
            raise RuntimeError('fault clear requires four fresh Disabled motors')
        self._exchange(MsgType.CHASSIS_CLEAR_FAULT, b'', MsgType.ACK)
        status = self._wait_for_state(1)
        if not status.all_disabled or status.runtime_fault != 0:
            raise RuntimeError('four-motor fault-clear confirmation invalid')
        return True

    def shutdown(self):
        if self._shutdown_complete:
            self.ClosePort()
            return True
        stop_error = None
        disable_error = None
        disable_confirmed = False
        try:
            try:
                self.stop()
            except Exception as exc:
                stop_error = exc
            try:
                self.disable()
                disable_confirmed = True
            except Exception as exc:
                disable_error = exc
        finally:
            self.ClosePort()
            self._shutdown_complete = disable_confirmed
        if disable_error is not None:
            if stop_error is not None:
                raise disable_error from stop_error
            raise disable_error
        if stop_error is not None:
            raise stop_error
        return True


HOST_FLAG_NONFINITE = 1 << 31
IMU_DIRECTION_FLAGS = (
    'AXIS_MAPPING_UNVERIFIED',
    'ROBOT_DIRECTION_UNDEFINED',
    'DEFERRED_UNTIL_FINAL_ASSEMBLY',
)


@dataclass
class IMUJsonRateTracker:
    last_received: Optional[float] = None
    last_sample_count: Optional[int] = None
    last_timestamp_us: Optional[int] = None

    def update(self, state):
        received = float(state.received_monotonic)
        telemetry_hz = None
        sample_hz = None
        if self.last_received is not None and received > self.last_received:
            telemetry_hz = 1.0 / (received - self.last_received)
        if self.last_sample_count is not None and self.last_timestamp_us is not None:
            sample_delta = (int(state.sample_count) - self.last_sample_count) & 0xFFFFFFFF
            time_delta_us = (
                int(state.cboard_timestamp) - self.last_timestamp_us
            ) & 0xFFFFFFFF
            if time_delta_us:
                sample_hz = sample_delta * 1_000_000.0 / time_delta_us
        self.last_received = received
        self.last_sample_count = int(state.sample_count)
        self.last_timestamp_us = int(state.cboard_timestamp)
        return telemetry_hz, sample_hz


def _imu_finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


def _imu_vector_or_null(values):
    if len(values) != 3:
        return [None, None, None]
    return [_imu_finite_or_none(value) for value in values]


def build_imu_web_record(state, tracker=None):
    telemetry_hz, sample_hz = tracker.update(state) if tracker else (None, None)
    accel = _imu_vector_or_null(state.accel_mps2)
    gyro = _imu_vector_or_null(state.gyro_rad_s)
    scalars = {
        'age_ms': _imu_finite_or_none(state.age_ms),
        'roll_rad': _imu_finite_or_none(state.roll_rad),
        'pitch_rad': _imu_finite_or_none(state.pitch_rad),
        'yaw_rad': _imu_finite_or_none(state.yaw_rad),
        'temperature_c': _imu_finite_or_none(state.temperature_c),
        'received_monotonic': _imu_finite_or_none(state.received_monotonic),
        'telemetry_hz': (
            _imu_finite_or_none(telemetry_hz) if telemetry_hz is not None else None
        ),
        'sample_hz': _imu_finite_or_none(sample_hz) if sample_hz is not None else None,
    }
    finite = all(value is not None for value in accel + gyro) and all(
        scalars[name] is not None
        for name in ('age_ms', 'roll_rad', 'pitch_rad', 'yaw_rad', 'temperature_c')
    )
    flags = int(state.flags)
    if not finite:
        flags |= HOST_FLAG_NONFINITE
    data_healthy = bool(
        finite
        and state.imu_online
        and state.imu_ready
        and state.imu_calibrated
        and not state.imu_stale
        and not state.imu_fault
    )
    return {
        'sequence': int(state.sequence),
        'timestamp_us': int(state.cboard_timestamp),
        'source': 'hardware',
        'healthy': data_healthy,
        'motion_healthy': bool(state.healthy),
        'age_ms': scalars['age_ms'],
        'flags': flags,
        'accel_mps2': accel,
        'gyro_rad_s': gyro,
        'roll_rad': scalars['roll_rad'],
        'pitch_rad': scalars['pitch_rad'],
        'yaw_rad': scalars['yaw_rad'],
        'temperature_c': scalars['temperature_c'],
        'calibration_state': int(state.calibration_state),
        'imu_state': int(state.state),
        'sample_count': int(state.sample_count),
        'missed_sample_count': int(state.missed_sample_count),
        'spi_error_count': int(state.spi_error_count),
        'telemetry_hz': scalars['telemetry_hz'],
        'sample_hz': scalars['sample_hz'],
        'received_monotonic': scalars['received_monotonic'],
        'direction_flags': list(IMU_DIRECTION_FLAGS),
        'axis_mapping_verified': False,
        'robot_direction_defined': False,
    }


def encode_imu_web_record(record):
    import json
    return json.dumps(record, ensure_ascii=False, separators=(',', ':'), allow_nan=False)


def run_imu_json_stream(bits, stdout, stderr, stop_event, interval_s, *, max_frames=None):
    tracker = IMUJsonRateTracker()
    emitted = 0
    bits.OpenPort()
    try:
        while not stop_event.is_set():
            started = time.monotonic()
            try:
                state = bits.GetIMUState()
                stdout.write(encode_imu_web_record(build_imu_web_record(state, tracker)) + '\n')
                stdout.flush()
                emitted += 1
            except (RuntimeError, ValueError, TypeError, OverflowError) as exc:
                print(f'[imu-json] discarded frame: {exc}', file=stderr, flush=True)
            if max_frames is not None and emitted >= max_frames:
                break
            remaining = interval_s - (time.monotonic() - started)
            if remaining > 0.0:
                stop_event.wait(remaining)
    finally:
        bits.ClosePort()
    return emitted


def _imu_json_main():
    import argparse
    import signal
    import sys

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--json-lines', action='store_true')
    parser.add_argument('--interval', type=float, default=0.02)
    args = parser.parse_args()
    if not args.json_lines:
        parser.error('--json-lines is required')
    if not math.isfinite(args.interval) or args.interval <= 0.0:
        parser.error('--interval must be positive and finite')

    stop_event = threading.Event()

    def request_stop(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    bits = DM2325Bits(args.port, exclusive=True)
    try:
        run_imu_json_stream(bits, sys.stdout, sys.stderr, stop_event, args.interval)
    except Exception as exc:
        print(f'[imu-json] fatal: {exc}', file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(_imu_json_main())

# -*- coding: utf-8 -*-
"""
UR 客户端接口协议（仅依赖标准库）。

适用：UR CB3(固件≥3.5) / e-Series 的 Primary(30001)/Secondary(30002)/
Real-time(30003) 客户端接口 —— 每帧格式：
    int32 BE  messageSize   # 整帧长度(含本字段)
    uint8     messageType   # 16 = Robot State
    子包序列: int32 BE packageSize(含本字段) + uint8 packageType + payload

已知 messageType:
    ROBOT_STATE = 16
Robot State 常用子包:
    0 = Robot mode data
    1 = Joint data
    2 = Tool data
    3 = Masterboard data

注意：老固件(<3.5)是固定长度流(无帧头)，此解析器不适用 —— 连上后若检测到
长度不合理会自动提示并转 1108 字节固定模式(只尽力解析 6 关节 q_actual)。
"""
from __future__ import annotations

import socket
import struct

MESSAGE_TYPE_ROBOT_STATE = 16

SUB_ROBOT_MODE = 0
SUB_JOINT_DATA = 1
SUB_TOOL_DATA = 2
SUB_MASTERBOARD = 3

# 每个关节在 Joint data 子包中的字节数:
# q_actual f64, q_target f64, qd_actual f64, I_actual f32, V_actual f32,
# T_motor f32, T_micro f32, jointMode u8  => 24 + 16 + 1 = 41
JOINT_DATA_JOINT_SIZE = 41

# 现代格式帧最小合理长度 / 最大合理长度
# 最小放宽到 5(4字节长度+1字节类型)：版本消息(55B)、心跳(9B)等小帧也要能切，
# 否则会破坏帧流对齐导致解析错位。
FRAME_MIN = 5
FRAME_MAX = 100000

LEGACY_FRAME_SIZE = 1108  # CB3 老固件固定长度(尽力解析)


class URState:
    """从 Robot State 消息里抽出的、我们关心的字段。"""

    __slots__ = (
        "timestamp", "robot_mode", "control_mode", "is_program_running",
        "is_power_on", "is_emergency_stopped", "speed_fraction",
        "q_actual", "qd_actual", "current",
    )

    def __init__(self):
        self.timestamp = 0.0
        self.robot_mode = -1
        self.control_mode = -1
        self.is_program_running = False
        self.is_power_on = False
        self.is_emergency_stopped = False
        self.speed_fraction = 0.0
        self.q_actual: list[float] = []
        self.qd_actual: list[float] = []
        self.current: list[float] = []


def _u32(data: bytes, off: int) -> int:
    return struct.unpack_from(">I", data, off)[0]


def parse_robot_state(payload: bytes) -> URState:
    """解析 Robot State 消息 content(不含 4 字节 size + 1 字节 type)。"""
    st = URState()
    off = 0
    n = len(payload)
    while off + 5 <= n:
        pkg_size = _u32(payload, off)
        pkg_type = payload[off + 4]
        if pkg_size < 5 or off + pkg_size > n:
            break
        body = payload[off + 5: off + pkg_size]
        if pkg_type == SUB_ROBOT_MODE:
            # body: timestamp(u64) + 7×bool(各1字节) + robotMode(u8) +
            #       controlMode(u8) + targetSpeedFraction(f64) + ...
            st.timestamp = struct.unpack_from(">Q", body, 0)[0] * 1e-9
            b = body[8:]
            # b[0..6]: isRealRobotConnected/Enabled/PowerOn/EmergencyStopped/
            #          ProtectiveStopped/ProgramRunning/ProgramPaused
            st.is_power_on = bool(b[2]) if len(b) > 2 else False
            st.is_emergency_stopped = bool(b[3]) if len(b) > 3 else False
            st.is_program_running = bool(b[5]) if len(b) > 5 else False
            if len(b) >= 17:
                st.robot_mode = b[7]
                st.control_mode = b[8]
                st.speed_fraction = struct.unpack_from(">d", b, 9)[0]
        elif pkg_type == SUB_JOINT_DATA:
            # 每关节: q_actual@0 q_target@8 qd_actual@16 (f64×3),
            #          I_actual@24 V_actual@28 T_motor@32 T_micro@36 (f32×4),
            #          jointMode@40 (u8)
            per = JOINT_DATA_JOINT_SIZE
            nj = len(body) // per
            for i in range(nj):
                seg = body[i * per:(i + 1) * per]
                if len(seg) < 41:
                    continue
                st.q_actual.append(struct.unpack_from(">d", seg, 0)[0])
                st.qd_actual.append(struct.unpack_from(">d", seg, 16)[0])
                st.current.append(struct.unpack_from(">f", seg, 24)[0])
        off += pkg_size
    return st


class URStreamClient:
    """连接 UR 客户端接口(TCP)，自动按帧切割并解析 Robot State。"""

    def __init__(self, host: str, port: int = 30001, timeout: float = 2.0):
        self.host = host
        self.port = port
        self.sock: socket.socket | None = None
        self._buf = bytearray()
        self.timeout = timeout
        self.legacy = False

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)
        self._buf = bytearray()

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def _read_more(self) -> bool:
        assert self.sock is not None
        try:
            chunk = self.sock.recv(65536)
        except socket.timeout:
            return False
        if not chunk:
            raise ConnectionError("UR socket closed by peer")
        self._buf += chunk
        return True

    def _extract_frames(self):
        """从缓冲里切出完整帧，返回 (raw_frame_bytes) 列表。"""
        frames = []
        while True:
            if len(self._buf) < 4:
                break
            size = struct.unpack(">I", bytes(self._buf[:4]))[0]
            if FRAME_MIN <= size <= FRAME_MAX:
                if len(self._buf) < size:
                    break  # 等更多数据
                frames.append(bytes(self._buf[:size]))
                del self._buf[:size]
            else:
                # 帧头不合理 → 老固件固定长度模式
                if self._buf and not self.legacy:
                    self.legacy = True
                break
        if self.legacy and not frames:
            # 1108 字节固定帧
            if len(self._buf) >= LEGACY_FRAME_SIZE:
                frames.append(bytes(self._buf[:LEGACY_FRAME_SIZE]))
                del self._buf[:LEGACY_FRAME_SIZE]
        return frames

    def read_state(self, max_attempts: int = 5) -> URState | None:
        """读一帧并解析；老固件模式尽力解 q_actual。"""
        while True:
            frames = self._extract_frames()
            for frame in frames:
                st = self._state_from_frame(frame)
                if st is not None:
                    return st
            if not self._read_more():
                return None

    @staticmethod
    def _state_from_frame(frame: bytes) -> URState | None:
        if len(frame) < 5:
            return None
        if not (FRAME_MIN <= struct.unpack(">I", frame[:4])[0] <= FRAME_MAX):
            # legacy 1108: 前 756/1108 字节无帧头 —— 只尽力抓关节角
            st = URState()
            # 老 CB3(3.1) 1108 字节布局里 q_actual 从偏移 252 起(每关节3double)
            st.q_actual = list(struct.unpack_from(">dddddd", frame, 252)) \
                if len(frame) >= 300 else []
            return st
        mtype = frame[4]
        if mtype != MESSAGE_TYPE_ROBOT_STATE:
            return None
        return parse_robot_state(frame[5:])

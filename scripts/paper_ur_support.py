"""Receive-only UR realtime transport used by the paper calibration scripts.

Field offsets and connection semantics are retained from the jx project's
record_ur_trajectory.py. Only the small transport is vendored here so numerical
and fault tests can run without the robot, ROS, Pinocchio, or a ~/UR10 checkout.
Protocol: https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/realtime-guide.html
"""
import socket
import struct
import threading


FRAME_MIN_SIZE = 540
FRAME_MAX_SIZE = 4096


def decode_frame(frame):
    if frame is None or len(frame) < FRAME_MIN_SIZE:
        raise RuntimeError('incomplete UR realtime frame')
    if struct.unpack_from('>I', frame)[0] != len(frame):
        raise RuntimeError('UR realtime frame length mismatch')
    return tuple(struct.unpack_from('>6d', frame, offset)
                 for offset in (252, 444, 492))


class RealtimeReader:
    def __init__(self, host, port=30013):
        self.host, self.port = host, port
        self.sock = None
        self.buffer = bytearray()
        self.reconnects = 0
        self.last_frame = None
        self.connect()

    def connect(self, timeout_seconds=3):
        self.close()
        self.sock = socket.create_connection((self.host, self.port), timeout=timeout_seconds)
        self.sock.settimeout(.1)
        self.buffer.clear()
        self.last_frame = None
        self.reconnects += 1

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None


class ScriptConnection:
    """Keep the 30002 socket open and continuously consume its state output.

    UR's script manual recommends a persistent connection and reading at least
    79 bytes before closing. Opening a socket, sending, and immediately closing
    without reading can discard the script through an unclean TCP shutdown.
    No motion is retried by this transport.
    """
    def __init__(self, host):
        self.sock = socket.create_connection((host, 30002), timeout=1)
        self.sock.settimeout(.1)
        self.closed = threading.Event()
        self.received = threading.Event()
        self.error = None
        self.received_bytes = 0
        self.thread = threading.Thread(target=self._drain, daemon=True)
        self.thread.start()

    def _drain(self):
        while not self.closed.is_set():
            try:
                chunk = self.sock.recv(65536)
                if not chunk:
                    raise ConnectionError('UR script connection closed')
                self.received_bytes += len(chunk)
                if self.received_bytes >= 79:
                    self.received.set()
            except socket.timeout:
                continue
            except OSError as exc:
                if not self.closed.is_set():
                    self.error = exc
                return

    def send(self, text):
        if self.closed.is_set() or self.error is not None:
            raise ConnectionError('UR script transport is not connected') from self.error
        if not text.endswith('\n'):
            raise ValueError('URScript must end with a newline')
        self.sock.sendall(text.encode('ascii'))

    def close(self):
        if self.closed.is_set():
            return
        # A finite endpoint has settled, or the caller has already sent stop.
        # Finish draining before the orderly close; never resend a program.
        self.received.wait(.2)
        self.closed.set()
        try:
            self.sock.shutdown(socket.SHUT_WR)
        except OSError:
            pass
        self.sock.close()
        self.thread.join(timeout=.3)


def dashboard(cmd, ip, timeout=3):
    """Only query status; callers never use Dashboard motion or unlock commands."""
    with socket.create_connection((ip, 29999), timeout=timeout) as sock:
        sock.recv(4096)
        sock.sendall((cmd + '\n').encode('ascii'))
        return sock.recv(4096).decode('utf-8', 'replace').strip()

import socket
from threading import RLock


class Endpoint(object):
    def __init__(self, ip, port):
        self.ip = ip
        self.port = int(port)


class TCPClient(object):
    def __init__(self, endpoint, timeout_s=1.0, connect_timeout_s=2.0):
        self._endpoint = endpoint
        self._timeout_s = float(timeout_s)
        self._connect_timeout_s = float(connect_timeout_s)
        self._socket = None
        self._lock = RLock()

    @property
    def connected(self):
        return self._socket is not None

    def _open_socket(self):
        sock = socket.create_connection((self._endpoint.ip, self._endpoint.port), timeout=self._connect_timeout_s)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(self._timeout_s)
        return sock

    def connect(self):
        with self._lock:
            if self._socket is None:
                self._socket = self._open_socket()

    def close(self):
        with self._lock:
            sock = self._socket
            self._socket = None

        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass

    def sendall(self, payload):
        last_error = None
        for _ in range(2):
            try:
                self.connect()
                with self._lock:
                    sock = self._socket
                if sock is None:
                    raise socket.error('socket not connected')
                sock.sendall(payload)
                return
            except Exception as exc:
                last_error = exc
                self.close()

        raise last_error

    def recv(self, size):
        self.connect()
        with self._lock:
            sock = self._socket
        if sock is None:
            raise socket.error('socket not connected')

        try:
            return sock.recv(size)
        except socket.timeout:
            raise
        except Exception:
            self.close()
            raise

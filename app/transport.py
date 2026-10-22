"""Byte-stream transports. TCP never reconnects or replays commands automatically."""
import select
import socket
import time
from urllib.parse import urlsplit


class TcpTransport:
    def __init__(self, endpoint, *, timeout=3.0, write_timeout=0.5, **_serial_options):
        url=urlsplit(endpoint)
        if url.scheme!='tcp' or not url.hostname or url.path or url.query or url.fragment or url.username:
            raise ValueError("TCP 地址格式为 tcp://192.168.10.50:5000")
        port=url.port or 5000
        if not 1<=port<=65535:raise ValueError("TCP 端口必须为1～65535")
        self._socket=socket.create_connection((url.hostname,port),timeout=timeout or 3.0)
        self._socket.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        self._socket.setblocking(False)
        self.write_timeout=write_timeout

    @property
    def in_waiting(self):
        if not select.select([self._socket],[],[],0)[0]:return 0
        try:data=self._socket.recv(4096,socket.MSG_PEEK)
        except BlockingIOError:return 0
        if not data:raise ConnectionError("STM32 TCP连接已关闭；请确认停机后手动重连")
        return len(data)

    def read(self,size):
        if size<=0:return b''
        try:data=self._socket.recv(size)
        except BlockingIOError:return b''
        if not data:raise ConnectionError("STM32 TCP连接已关闭")
        return data

    def write(self,data):
        view=memoryview(data);sent=0;deadline=time.monotonic()+self.write_timeout
        while sent<len(view):
            remaining=deadline-time.monotonic()
            if remaining<=0 or not select.select([],[self._socket],[],remaining)[1]:
                raise TimeoutError("TCP发送超时，执行结果未知，未自动重发")
            try:n=self._socket.send(view[sent:])
            except BlockingIOError:continue
            if not n:raise ConnectionError("TCP发送期间连接已关闭")
            sent+=n
        return sent

    def reset_input_buffer(self):
        # Discard prior bytes, with a strict bound so unsolicited traffic cannot hang UI.
        for _ in range(16):
            waiting=self.in_waiting
            if not waiting:return
            self.read(waiting)
        raise ConnectionError("TCP接收流持续异常，未发送新命令")

    def close(self):
        self._socket.close()


def open_transport(endpoint, *, timeout=3.0, write_timeout=0.5):
    if endpoint.startswith('tcp://'):
        return TcpTransport(endpoint,timeout=timeout,write_timeout=write_timeout)
    from serial import Serial
    return Serial(endpoint,115200,bytesize=8,parity='N',stopbits=1,timeout=0,write_timeout=write_timeout)

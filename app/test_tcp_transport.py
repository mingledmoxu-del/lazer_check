import socket
import struct
import threading
import time
import unittest
from app.transport import TcpTransport
from app import MotorController,CommunicationError
from app.protocol import append_crc,CMD_READ_PA,CMD_READ_SPEED


class TcpTests(unittest.TestCase):
    def server(self,handler):
        listener=socket.socket();listener.bind(('127.0.0.1',0));listener.listen(1)
        errors=[]
        def run():
            try:
                conn,_=listener.accept()
                with conn:
                    conn.settimeout(2);handler(conn)
            except BaseException as exc:errors.append(exc)
            finally:listener.close()
        thread=threading.Thread(target=run,daemon=True);thread.start()
        return f'tcp://127.0.0.1:{listener.getsockname()[1]}',thread,errors

    def test_partial_reads_merged_bytes_and_eof(self):
        release=threading.Event()
        def handle(conn):
            self.assertEqual(conn.recv(3),b'abc')
            conn.sendall(b'12');time.sleep(0.02);conn.sendall(b'3456');release.wait(2)
        endpoint,thread,errors=self.server(handle)
        transport=TcpTransport(endpoint)
        try:
            self.assertEqual(transport.write(b'abc'),3)
            data=b'';deadline=time.monotonic()+2
            while len(data)<6 and time.monotonic()<deadline:
                data+=transport.read(min(1,transport.in_waiting));time.sleep(0.001)
            self.assertEqual(data,b'123456')
            release.set();thread.join(2)
            with self.assertRaises(ConnectionError):_=transport.in_waiting
            self.assertFalse(errors)
        finally:release.set();transport.close()

    def test_sdk_round_trip_over_real_tcp_and_no_command_replay(self):
        frames=[]
        def handle(conn):
            for _ in range(2):
                data=b''
                while len(data)<3:data+=conn.recv(3-len(data))
                total=data[2]+3
                while len(data)<total:data+=conn.recv(total-len(data))
                frames.append(data)
                if len(frames)==2:return  # Lost response: SDK must not reconnect/replay.
                payload=bytes((data[3],data[4],0))+data[5:7]+struct.pack('>h',10000)
                response=append_crc(b'\xaa\x55'+bytes((len(payload)+2,))+payload)
                conn.sendall(response[:4]);time.sleep(0.01);conn.sendall(response[4:])
        endpoint,thread,errors=self.server(handle)
        controller=MotorController(endpoint,timeout=0.5).connect()
        try:
            self.assertEqual(controller.read_pa(1,11),10000)
            with self.assertRaises(CommunicationError):controller.read_pa(1,11)
            with self.assertRaises(CommunicationError):controller.read_pa(1,11)
            thread.join(2)
            self.assertEqual(len(frames),2)
            self.assertEqual([f[4] for f in frames],[CMD_READ_PA,CMD_READ_PA])
            self.assertFalse(errors)
        finally:controller.close(stop=False)

    def test_invalid_endpoint(self):
        for endpoint in ('http://localhost:5000','tcp://','tcp://localhost:5000/path','tcp://user@localhost:5000'):
            with self.assertRaises(ValueError):TcpTransport(endpoint)


if __name__=='__main__':unittest.main()

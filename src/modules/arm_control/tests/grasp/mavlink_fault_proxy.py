"""Test-only transparent TCP proxy: selectively drop whole MAVLink frames.

Never edits a payload, CRC, contact measurement, or actuator command. HIL
sensor/time messages keep flowing, so a frozen clock cannot fake timeout proof.
"""
import select
import socket
import threading
import time
from collections import Counter


class FrameStream:
    def __init__(self):
        self.buffer=bytearray()

    def feed(self,data):
        self.buffer.extend(data);frames=[]
        while self.buffer:
            magic=self.buffer[0]
            if magic not in [0xfd,0xfe]: raise RuntimeError('non-MAVLink byte in test bridge')
            header=10 if magic==0xfd else 6
            if len(self.buffer)<header: break
            length=header+self.buffer[1]+2
            if magic==0xfd and self.buffer[2]&1: length+=13
            if len(self.buffer)<length: break
            frame=bytes(self.buffer[:length]);del self.buffer[:length]
            msgid=int.from_bytes(frame[7:10],'little') if magic==0xfd else frame[5]
            frames.append((msgid,frame))
        return frames


class FaultProxy:
    def __init__(self,listen_port=4560,server_port=4562):
        self.server_port=server_port;self.drop_id=None
        self.received=Counter();self.dropped=Counter();self.error=None
        self.last_clock_us=0;self.first_drop_clock_us=None;self.shutdown_note=None
        self.cleaning_up=threading.Event()
        self.stopping=threading.Event();self.connections=[]
        self.listener=socket.socket();self.listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.listener.bind(('127.0.0.1',listen_port));self.listener.listen(1);self.listener.settimeout(.2)
        self.thread=threading.Thread(target=self.run,name='grasp-test-bridge',daemon=True)

    def start(self):
        self.thread.start()

    def run(self):
        try:
            while not self.stopping.is_set():
                try: px4,_=self.listener.accept();break
                except socket.timeout: continue
            else: return
            self.connections.append(px4);deadline=time.monotonic()+15
            while not self.stopping.is_set():
                try: gazebo=socket.create_connection(('127.0.0.1',self.server_port),timeout=.2);break
                except OSError:
                    if time.monotonic()>deadline: raise RuntimeError('Gazebo test bridge connection timeout')
                    self.stopping.wait(.05)
            else: return
            self.connections.append(gazebo)
            for sock in self.connections:
                sock.settimeout(1)
                sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            stream=FrameStream();command_stream=FrameStream()
            while not self.stopping.is_set():
                readable,_,_=select.select([px4,gazebo],[],[],.2)
                for sock in readable:
                    data=sock.recv(65536)
                    if not data: return
                    if sock is px4:
                        outgoing=[]
                        for msgid,frame in command_stream.feed(data):
                            self.received[msgid]+=1
                            if self.drop_id==42000 and msgid==42000:
                                self.dropped[msgid]+=1
                                if self.first_drop_clock_us is None: self.first_drop_clock_us=self.last_clock_us
                            else: outgoing.append(frame)
                        if outgoing: gazebo.sendall(b''.join(outgoing))
                    else:
                        outgoing=[]
                        for msgid,frame in stream.feed(data):
                            self.received[msgid]+=1
                            header=10 if frame[0]==0xfd else 6
                            if msgid in [107,115] and frame[1]>=8:
                                self.last_clock_us=int.from_bytes(frame[header:header+8],'little')
                            if msgid==self.drop_id:
                                self.dropped[msgid]+=1
                                if self.first_drop_clock_us is None:
                                    # 42002 has a source timestamp; 42001 deliberately
                                    # retains its old layout, so anchor it to HIL time.
                                    self.first_drop_clock_us=(int.from_bytes(frame[header:header+8],'little')
                                                              if msgid==42002 else self.last_clock_us)
                            else: outgoing.append(frame)
                        if outgoing: px4.sendall(b''.join(outgoing))
        except (OSError,RuntimeError) as exc:
            if not self.stopping.is_set():
                if self.cleaning_up.is_set(): self.shutdown_note=str(exc)
                else: self.error=str(exc)

    def begin_shutdown(self):
        self.cleaning_up.set()

    def close(self):
        self.stopping.set()
        for sock in [self.listener,*self.connections]:
            try: sock.shutdown(socket.SHUT_RDWR)
            except OSError: pass
            sock.close()
        self.thread.join(timeout=3)

    def report(self):
        return dict(received=dict(self.received),dropped=dict(self.dropped),error=self.error,
                    injected_message_id=next(iter(self.dropped),None),active_filter_message_id=self.drop_id,
                    actuator_commands_modified=False,
                    arm_commands_filtered=self.dropped[42000]>0,
                    first_drop_clock_us=self.first_drop_clock_us,shutdown_note=self.shutdown_note)

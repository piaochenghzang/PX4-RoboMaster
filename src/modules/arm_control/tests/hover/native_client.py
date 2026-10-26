"""Shared local PX4 client/status reader; constructing it sends no commands."""
import re
import os
import socket
import struct
import subprocess as sp
import time


class NativeClient:
    def __init__(self, build, env, *, transport='process', expected_pid=None):
        self.build, self.env = build, env
        if transport not in ('process','unix') or (transport=='unix' and
            (not isinstance(expected_pid,int) or expected_pid<=0)):
            raise ValueError('Unix transport requires the owned PX4 process PID')
        self.transport=transport;self.expected_pid=expected_pid

    def _unix_client(self,module,args):
        # EXACT existing PX4 daemon protocol, not a new control endpoint:
        # platforms/posix/src/px4/common/px4_daemon/{client,server,sock_protocol}.cpp
        # One request per connection; argv joined by spaces + no-TTY byte;
        # response is stdout followed by {0, return_code} and EOF.
        words=[module,*map(str,args)]
        if module not in ('listener','arm_control') or any(
            not w or len(w)>4096 or any(ord(c)<33 or ord(c)>126 for c in w) for w in words):
            raise ValueError('Unsupported local test command/argument')
        deadline=time.monotonic()+4.;reply=bytearray()
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as conn:
            conn.settimeout(4.);conn.connect('/tmp/px4-sock-0')
            pid,uid,_=struct.unpack('3i',conn.getsockopt(socket.SOL_SOCKET,socket.SO_PEERCRED,12))
            if pid!=self.expected_pid or uid!=os.geteuid():
                raise RuntimeError('Native IPC peer is not this test owned PX4 process')
            conn.sendall(' '.join(words).encode('ascii')+b'\x00')
            while True:
                remaining=deadline-time.monotonic()
                if remaining<=0:raise TimeoutError('Native IPC command timeout')
                conn.settimeout(remaining);part=conn.recv(8192)
                if not part:break
                reply.extend(part)
                if len(reply)>1048576:raise RuntimeError('Oversize native IPC reply')
        if len(reply)<2 or reply[-2]!=0:raise RuntimeError('Truncated native IPC reply')
        output=bytes(reply[:-2]).decode('utf-8',errors='replace')
        if reply[-1]:raise RuntimeError(module+' failed: '+output)
        return output

    def client(self, module, *args):
        if self.transport=='unix':return self._unix_client(module,args)
        result = sp.run([str(self.build / 'bin' / ('px4-' + module)), *map(str, args)],
            env=self.env, stdout=sp.PIPE, stderr=sp.STDOUT, text=True, timeout=4)
        if result.returncode:
            raise RuntimeError(module + ' failed: ' + result.stdout)
        return result.stdout

    def status(self, topic):
        out = {}
        for line in self.client('listener', topic, 1).splitlines():
            match = re.match(r'\s+(\w+):\s+(.+)', line)
            if not match:
                continue
            key, value = match.groups(); value = value.strip()
            if value.startswith('['):
                out[key] = [float(x) for x in value[1:value.index(']')].split(',')]
            elif value in ('True', 'False'):
                out[key] = value == 'True'
            else:
                try:
                    out[key] = float(value.split()[0])
                except ValueError:
                    pass
        if 'timestamp' not in out:
            raise RuntimeError('Missing native ' + topic)
        return out

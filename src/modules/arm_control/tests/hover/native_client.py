"""Shared local PX4 client/status reader; constructing it sends no commands."""
import re
import subprocess as sp


class NativeClient:
    def __init__(self, build, env):
        self.build, self.env = build, env

    def client(self, module, *args):
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
                out[key] = [float(x) for x in value.strip('[]').split(',')]
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

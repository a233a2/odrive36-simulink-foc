"""USB CDC transport and protocol for the ODrive FOC control panel."""
import math
import sys

from build import ROOT

MODES = {'力矩': ('torque', 'N·m'), '速度': ('speed', 'rad/s'), '位置': ('position', 'rad')}
MOTION_COMMANDS = frozenset(('set', 'torque', 'speed', 'position', 'current', 'velocity', 'cal'))


def load_pyserial():
    if not getattr(sys, 'frozen', False):
        path = str(ROOT / 'tmp/testdeps')
        if path not in sys.path:
            sys.path.insert(0, path)
    try:
        import serial
        from serial.tools import list_ports
    except ImportError as error:
        raise RuntimeError('需要 pyserial 3.5。运行 python tools/install_test_deps.py。') from error
    return serial, list_ports


def device_ports():
    _, ports = load_pyserial()
    return [port.device for port in ports.comports() if (port.vid, port.pid) == (0xCAFE, 0x4011)]


def encode_command(text):
    line = text.strip()
    if not line or len(line) > 95 or any(not 32 <= ord(ch) <= 126 for ch in line):
        raise ValueError('每条命令最多 95 个可打印 ASCII 字符。')
    return (line + '\n').encode('ascii')


def target_command(mode, axis, value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError('请输入有限的目标值。')
    return f'{MODES[mode][0]} {axis} {number:.9g}'


def response_fields(line):
    return dict(word.split('=', 1) for word in line.split() if '=' in word)


class SerialLink:
    def __init__(self, port_name=None, *, port=None):
        serial, _ = load_pyserial()
        self.port = port if port is not None else serial.Serial(
            port_name, 115200, timeout=0, write_timeout=0.15,
        )
        self.pending = bytearray()
        self.discard_line = False
        self.requested = set()
        self.active = set()

    def send(self, command):
        data = encode_command(command)
        if self.port.write(data) != len(data):
            raise OSError('串口命令未完整发送。')
        words = command.split()
        name = words[0]
        # Acknowledged motion requests alone start the host heartbeat.
        changes_mode = name == 'mode' and len(words) == 3
        if name in ('stop', 'clear'):
            self.requested.clear()
            self.active.clear()
        elif len(words) >= 2 and words[1] in ('0', '1'):
            axis = int(words[1])
            if name in MOTION_COMMANDS:
                self.requested.add(axis)
            elif changes_mode or name == 'idle':
                self.requested.discard(axis)
                self.active.discard(axis)

    def read(self):
        self.pending.extend(self.port.read(1024))
        lines = []
        while b'\n' in self.pending:
            line, _, rest = self.pending.partition(b'\n')
            self.pending = bytearray(rest)
            if not self.discard_line and len(line) <= 1024:
                text = line.decode('ascii', errors='replace').strip()
                self._track_motion(text)
                if text:
                    lines.append(text)
            self.discard_line = False
        if len(self.pending) > 1024:
            self.pending.clear()
            self.discard_line = True
        return lines

    def _track_motion(self, line):
        if not line.startswith('OK '):
            return
        fields = response_fields(line)
        if fields.get('axis') not in ('0', '1'):
            return
        axis = int(fields['axis'])
        response = line.split(' ', 2)[1]
        if response in MOTION_COMMANDS | {'target'} and axis in self.requested:
            self.requested.discard(axis)
            self.active.add(axis)
        elif response == 'feedback' and fields.get('state') == '0':
            self.active.discard(axis)

    def close(self):
        try:
            if self.active or self.requested:
                self.port.write(b'stop\n')
        finally:
            self.port.close()


def check_serial_runtime():
    serial, _ = load_pyserial()
    with serial.serial_for_url('loop://', timeout=0, write_timeout=0.15) as port:
        link = SerialLink(port=port)
        link.send('info')
        if link.read() != ['info']:
            raise RuntimeError('串口运行库回环测试失败。')

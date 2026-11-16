"""Bounded USB commissioning command runner, restricted to this firmware's VID/PID."""
import argparse
from datetime import datetime,timedelta,timezone
from pathlib import Path
import time
from serial_control import MOTION_COMMANDS, device_ports, load_pyserial

def main():
    serial,_=load_pyserial()
    p=argparse.ArgumentParser()
    p.add_argument('--port')
    p.add_argument('--command',action='append',default=[])
    p.add_argument('--watch',type=float,default=0)
    p.add_argument('--period',type=float,default=0.5,help='status sampling period, minimum 0.05 seconds')
    p.add_argument('--sample-command',choices=['status','encoder'],default='status')
    p.add_argument('--log')
    args=p.parse_args()
    if args.watch<0 or args.period<0.05:
        p.error('watch must be nonnegative and period at least 0.05 seconds')
    candidates=device_ports()
    if args.port:
        if args.port not in candidates:
            raise SystemExit('Selected port is not CM firmware (VID CAFE / PID 4011)')
        port_name=args.port
    elif len(candidates)==1:
        port_name=candidates[0]
    else:
        raise SystemExit('Expected one CM firmware port, found '+repr(candidates))
    commands=args.command or ['status','encoder']
    motion_axes={x.split()[1] for x in commands if len(x.split())>=2 and x.split()[0] in MOTION_COMMANDS}
    records=[]
    def record(value):
        stamp=datetime.now(timezone(timedelta(hours=8))).isoformat(timespec='milliseconds')
        line=stamp+' '+value
        print(line,flush=True);records.append(line)
    with serial.Serial(port_name,115200,timeout=0.01,write_timeout=0.2) as port:
        port.reset_input_buffer()
        last_ping=time.monotonic()
        incoming=bytearray()
        def send(cmd):
            port.write((cmd+'\n').encode('ascii'))
            if cmd!='ping': record('> '+cmd)
        def collect(duration):
            nonlocal last_ping
            end=time.monotonic()+duration
            lines=[]
            while time.monotonic()<end:
                if motion_axes and time.monotonic()-last_ping>0.25:
                    send('ping');last_ping=time.monotonic()
                incoming.extend(port.read(4096))
                while b'\n' in incoming:
                    raw,_,rest=incoming.partition(b'\n')
                    incoming[:]=rest
                    line=raw.decode('ascii',errors='replace').strip()
                    if line and line!='pong':
                        record(line);lines.append(line)
            return lines
        record('port='+port_name)
        try:
            for cmd in commands:
                send(cmd);lines=collect(0.25)
                if cmd.strip()!='dfu' and (not lines or any(x.startswith('ERR') for x in lines)):
                    raise RuntimeError('Command failed or received no reply: '+cmd)
            end=time.monotonic()+args.watch
            while time.monotonic()<end:
                send(args.sample_command);lines=collect(min(args.period,max(0.05,end-time.monotonic())))
                expected='fw=' if args.sample_command=='status' else 'M0 ok='
                if not any(x.startswith(expected) for x in lines):
                    raise RuntimeError('Status reply lost; ending motion test')
                if motion_axes and any(x.startswith('fw=') and ' fault=0x0 ' not in x for x in lines):
                    raise RuntimeError('Firmware latched a fault; ending motion test')
        except serial.SerialException:
            if any(cmd.strip()=='dfu' for cmd in commands):
                record('USB disconnected after dfu command; check STM32 BOOTLOADER enumeration')
            else:
                raise
        finally:
            # The caller explicitly starts motion; all such runs end by disabling PWM on those axes.
            for axis in sorted(motion_axes):
                try:
                    send('idle '+axis);lines=collect(0.25)
                    if not any(x.startswith('OK idle') for x in lines):
                        record('Idle not acknowledged; sending stop and relying on heartbeat timeout if USB is unavailable')
                        send('stop');collect(0.25)
                except serial.SerialException as error:
                    record('Could not send idle: '+str(error)+'; firmware heartbeat timeout remains the fallback')
            if args.log:
                path=Path(args.log)
                path.parent.mkdir(parents=True,exist_ok=True)
                path.write_text('\n'.join(records)+'\n',encoding='utf-8')

if __name__=='__main__':
    main()

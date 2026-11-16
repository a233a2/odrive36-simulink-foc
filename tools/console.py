"""USB CDC console. No motor commands are sent automatically; ping keeps an explicitly started command alive."""
import argparse
import queue
import threading
import time

def main():
    try:
        import serial
    except ImportError:
        raise SystemExit('Install pyserial: python -m pip install pyserial')
    parser=argparse.ArgumentParser()
    parser.add_argument('port',help='e.g. COM9 (confirm the NEW firmware port in Device Manager)')
    parser.add_argument('--no-heartbeat',action='store_true',help='for timeout testing')
    args=parser.parse_args()
    commands=queue.Queue()
    from serial.tools import list_ports
    known=[p.device.upper() for p in list_ports.comports() if p.vid==0xcafe and p.pid==0x4011]
    if args.port.upper() not in known:
        raise SystemExit('Port is not CM firmware (VID CAFE / PID 4011); confirm the new firmware COM port')
    done=threading.Event()
    def keyboard():
        while not done.is_set():
            try:
                text=input('> ').strip()
            except (EOFError,KeyboardInterrupt):
                text='quit'
            commands.put(text)
            if text in ['quit','exit']:
                return
    threading.Thread(target=keyboard,daemon=True).start()
    print('Commands: help, encoder, status, prepare, cal 0, position 0 0.1, idle 0, stop, clear, quit')
    print('No motion is started automatically. Closing this console sends stop; lost communication also times out.')
    with serial.Serial(args.port,115200,timeout=0.02,write_timeout=0.2) as port:
        last_ping=0.0
        incoming=bytearray()
        try:
            while True:
                try:
                    cmd=commands.get_nowait()
                    if cmd in ['quit','exit']:
                        break
                    if cmd:
                        port.write((cmd+'\n').encode('ascii'))
                except queue.Empty:
                    pass
                now=time.monotonic()
                if not args.no_heartbeat and now-last_ping>=0.25:
                    port.write(b'ping\n')
                    last_ping=now
                incoming.extend(port.read(1024))
                while b'\n' in incoming:
                    line,_,rest=incoming.partition(b'\n')
                    incoming=bytearray(rest)
                    text=line.decode('ascii',errors='replace').strip()
                    if text and text!='pong':
                        print(text)
        except KeyboardInterrupt:
            pass
        except serial.SerialException as error:
            print('USB disconnected: '+str(error))
        finally:
            done.set()
            try:
                port.write(b'stop\n')
                port.flush()
            except serial.SerialException:
                pass

if __name__=='__main__':
    main()

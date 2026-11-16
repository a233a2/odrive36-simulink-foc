"""USB CDC console. No motor commands are sent automatically; ping keeps an explicitly started command alive."""
import argparse
import queue
import threading
import time
from serial_control import SerialLink, device_ports, load_pyserial

def main():
    try:
        serial,_=load_pyserial()
    except RuntimeError as error:
        raise SystemExit(str(error)) from error
    parser=argparse.ArgumentParser()
    parser.add_argument('port',help='e.g. COM9 (confirm the NEW firmware port in Device Manager)')
    parser.add_argument('--no-heartbeat',action='store_true',help='for timeout testing')
    args=parser.parse_args()
    commands=queue.Queue()
    known=[port.upper() for port in device_ports()]
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
    print('Commands: info, help, prepare, cal 0, torque 0 0.001, speed 0 1, position 0 0.1')
    print('Use feedback 0, mode 0 position, set 0 0.1, idle 0, stop, clear, or quit.')
    print('No motion is started automatically. Closing this console sends stop; lost communication also times out.')
    with serial.Serial(args.port,115200,timeout=0.02,write_timeout=0.2) as port:
        link=SerialLink(port=port)
        last_ping=0.0
        try:
            while True:
                try:
                    cmd=commands.get_nowait()
                    if cmd in ['quit','exit']:
                        break
                    if cmd:
                        try:
                            link.send(cmd)
                        except ValueError as error:
                            print(error)
                except queue.Empty:
                    pass
                now=time.monotonic()
                if not args.no_heartbeat and link.active and now-last_ping>=0.25:
                    link.send('ping')
                    last_ping=now
                for text in link.read():
                    if text!='pong':
                        print(text)
        except KeyboardInterrupt:
            pass
        except OSError as error:
            print('USB disconnected: '+str(error))
        finally:
            done.set()
            try:
                link.close()
            except OSError:
                pass

if __name__=='__main__':
    main()

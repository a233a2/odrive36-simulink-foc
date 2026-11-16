"""Read a stopped axis's diagnostic trace; never starts motion or clears faults."""
import argparse,csv,time
from pathlib import Path
import serial
from serial.tools import list_ports

def main():
    p=argparse.ArgumentParser()
    p.add_argument('axis',type=int,choices=[0,1])
    p.add_argument('output')
    a=p.parse_args()
    ports=[x.device for x in list_ports.comports() if x.vid==0xcafe and x.pid==0x4011]
    if len(ports)!=1: raise SystemExit('Expected one CM firmware port: '+repr(ports))
    rows=[]
    with serial.Serial(ports[0],115200,timeout=.05,write_timeout=.3) as s:
        s.reset_input_buffer()
        for page in range(128):
            s.write(('trace %d %d\n'%(a.axis,page)).encode('ascii'))
            header=False;expected=None;received=0;deadline=time.monotonic()+1
            while time.monotonic()<deadline:
                line=s.readline().decode('ascii').strip()
                if line.startswith('ERR'): raise SystemExit(line)
                if line.startswith('TRACE '):
                    header=True
                    count=int(line.split('count=')[1].split()[0])
                    expected=min(4,max(0,count-page*4))
                elif line.startswith('T '):
                    rows.append(line.split()[1:]);received+=1
                if header and received==expected: break
            else: raise SystemExit('Incomplete trace response on page '+str(page))
            if len(rows)>=count: break
    path=Path(a.output);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['us','position_rad','velocity_rad_s','iq_ref_A','id_A','iq_A','vd_V','vq_V','theta_rad','encoder_age_us']);w.writerows(rows)
    print('Saved %d samples to %s'%(len(rows),path))

if __name__=='__main__': main()

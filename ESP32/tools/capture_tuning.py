#!/usr/bin/env python3
# capture_tuning.py
# Simple Python serial capture utility (uses pyserial).

import argparse
import serial
import sys
import time

parser = argparse.ArgumentParser(description='Capture serial output to file')
parser.add_argument('-p','--port',default='COM10')
parser.add_argument('-b','--baud',type=int,default=921600)
parser.add_argument('-o','--outfile',default=None)
parser.add_argument('-c','--cmd',default=None,help='Optional command to send once after opening the serial port')
parser.add_argument('-s','--stop-string',default='TUNING: capture stopped (auto)',help='If seen in serial output, stop capture and exit')
args = parser.parse_args()

port = args.port
baud = args.baud
outfile = args.outfile
if outfile is None:
    outfile = f'tuning_capture_{time.strftime("%Y%m%d_%H%M%S")}.txt'

print(f'Opening {port} @ {baud}, writing to {outfile}')
try:
    ser = serial.Serial(port, baud, timeout=1)
except Exception as e:
    print('Failed to open serial port:', e)
    sys.exit(2)

# After opening the port, wait a short moment for the device to settle
# and discard any boot messages. Also try to clear DTR/RTS to avoid
# unintentionally resetting the ESP32 when toggling the serial port.
try:
    # Try to disable DTR/RTS lines which on ESP32 can assert a reset
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception:
        # some platforms/drivers may not support toggling; ignore
        pass
    # Give the device time to boot/reply
    time.sleep(0.6)
    # Clear any boot output that may have been emitted on open
    try:
        ser.reset_input_buffer()
    except Exception:
        # fallback: read and discard for a short time
        end = time.time() + 0.1
        while time.time() < end:
            ser.read(1024)
except Exception:
    pass

# If an initial command was provided, send it and wait briefly for device to respond
if args.cmd:
    try:
        cmd = args.cmd
        if not cmd.endswith('\n'):
            cmd = cmd + '\n'
        ser.write(cmd.encode('utf8'))
        ser.flush()
        # give device a moment to process and emit output
        time.sleep(0.3)
        print('Sent initial command:', args.cmd)
    except Exception as e:
        print('Failed to send initial command:', e)

with open(outfile, 'w', encoding='utf8') as f:
    try:
        while True:
            line = ser.readline()
            if not line:
                continue
            try:
                s = line.decode('utf8', errors='replace').rstrip('\r\n')
            except Exception:
                s = str(line)
            print(s)
            f.write(s + '\n')
            f.flush()
            # Stop if the stop-string was observed
            if args.stop_string and args.stop_string in s:
                print('End marker seen, stopping capture')
                break
    except KeyboardInterrupt:
        print('\nCapture stopped by user')
    finally:
        ser.close()

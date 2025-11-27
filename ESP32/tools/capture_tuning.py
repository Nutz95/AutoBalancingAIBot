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

# If an initial command was provided, defer sending it until we see a likely
# interactive prompt / main menu from the device. This avoids missing the
# command when the ESP32 is still booting and the serial task isn't ready.
pending_cmd = None
if args.cmd:
    pending_cmd = args.cmd
    if not pending_cmd.endswith('\n'):
        pending_cmd = pending_cmd + '\n'
    # the actual send will occur once we observe a menu-like line in the serial
    cmd_sent = False

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
            # If we have a pending initial command and we detect the device's
            # interactive menu, send the command once to ensure it's processed.
            if pending_cmd and not cmd_sent:
                menu_triggers = ['== Main Menu ==', 'Main Menu', 'TUNING: capture started', 'IMU tasks started']
                for trig in menu_triggers:
                    if trig in s:
                        try:
                            ser.write(pending_cmd.encode('utf8'))
                            ser.flush()
                            print('Sent initial command (deferred):', args.cmd)
                        except Exception as e:
                            print('Failed to send initial command:', e)
                        cmd_sent = True
                        break
            # Stop if the stop-string was observed
            if args.stop_string and args.stop_string in s:
                print('End marker seen, stopping capture')
                break
    except KeyboardInterrupt:
        print('\nCapture stopped by user')
    finally:
        ser.close()

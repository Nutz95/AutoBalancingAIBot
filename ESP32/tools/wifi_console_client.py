#!/usr/bin/env python3
"""
Simple TCP client to connect to the ESP32 Wi‑Fi console server.
Usage: python wifi_console_client.py <host> [port]

Connects, prints lines received from the ESP, and allows typing commands which
are sent to the ESP terminated with a newline.
"""
import socket
import sys
import threading
import time


if len(sys.argv) < 2:
    print("Usage: wifi_console_client.py <host> [port]")
    sys.exit(1)

HOST = sys.argv[1]
PORT = int(sys.argv[2]) if len(sys.argv) >= 3 else 2333


def make_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Enable TCP keepalive so dead peers are detected by the OS stack.
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    except Exception:
        pass
    return s


class ConsoleClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.reader_thr = None
        self.reader_stop = threading.Event()
        self.connected = threading.Event()
        self.pending_cmd = None

    def connect(self, timeout=5):
        backoff = 1.0
        while True:
            try:
                s = make_socket()
                s.settimeout(timeout)
                s.connect((self.host, self.port))
                s.settimeout(None)
                self.sock = s
                self.connected.set()
                print(f"Connected to {self.host}:{self.port}. Type commands, Ctrl-C to quit.")
                # start reader
                self.reader_stop.clear()
                self.reader_thr = threading.Thread(target=self._reader, daemon=True)
                self.reader_thr.start()
                return True
            except Exception as e:
                print(f"Connect failed: {e}; retrying in {backoff}s...")
                time.sleep(backoff)
                backoff = min(backoff * 2.0, 16.0)

    def close(self):
        try:
            self.reader_stop.set()
            if self.sock:
                try:
                    self.sock.shutdown(socket.SHUT_RDWR)
                except Exception:
                    pass
                self.sock.close()
        finally:
            self.sock = None
            self.connected.clear()

    def _reader(self):
        buf = b''
        try:
            while not self.reader_stop.is_set():
                try:
                    data = self.sock.recv(1024)
                except OSError as e:
                    print('[Reader error]', e)
                    break
                if not data:
                    print('[Disconnected]')
                    break
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    try:
                        print(line.decode('utf-8', errors='ignore'))
                    except Exception:
                        print(repr(line))
        finally:
            # mark disconnected
            self.close()

    def send(self, s):
        if not s.endswith('\n'):
            s = s + '\n'
        b = s.encode('utf-8')
        # try to send; on failure save to pending_cmd and trigger reconnect
        try:
            if not self.sock:
                raise OSError('not connected')
            self.sock.sendall(b)
            return True
        except (BrokenPipeError, OSError) as e:
            print('[Send failed — will attempt reconnect]', e)
            # remember last command to resend after reconnect
            self.pending_cmd = s
            self.close()
            self._attempt_reconnect_and_resend()
            return False

    def _attempt_reconnect_and_resend(self):
        print('[Reconnecting]')
        ok = self.connect()
        if ok and self.pending_cmd:
            try:
                self.sock.sendall(self.pending_cmd.encode('utf-8'))
                print('[Resent pending command]')
                self.pending_cmd = None
            except Exception as e:
                print('[Resend failed]', e)
                # leave pending_cmd set for next reconnect attempt


def main():
    client = ConsoleClient(HOST, PORT)
    client.connect()

    try:
        while True:
            s = sys.stdin.readline()
            if not s:
                break
            # If currently disconnected, attempt to reconnect first
            if not client.connected.is_set():
                print('[Not connected — attempting reconnect before sending]')
                client._attempt_reconnect_and_resend()
            # If there is a pending command from a failed send, merge behavior:
            # we prioritize the new user input; send it and keep pending_cmd as-is
            try:
                client.send(s)
            except KeyboardInterrupt:
                break
    except KeyboardInterrupt:
        pass
    finally:
        client.close()
        print('Closed')


if __name__ == '__main__':
    main()

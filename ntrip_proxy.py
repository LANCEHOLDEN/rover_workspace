#!/usr/bin/env python3
"""
NTRIP multiplexer — combines GEODNET AUTO_WGS84 (observations) and BRDC
(ephemeris) into a single local TCP server for the InertialSense node.

Usage:
  python3 ntrip_proxy.py

Then in the launch file set:
  RTK_server_IP:   '127.0.0.1'
  RTK_server_port: 2102
  RTK_mountpoint:  'COMBINED'
  RTK_username:    ''
  RTK_password:    ''
"""
import socket
import threading
import base64
import time
import datetime

# GEODNET credentials
GEODNET_HOST = 'rtk.geodnet.com'
GEODNET_PORT = 2101
USERNAME     = 'RTKsub_WFUEN'
PASSWORD     = '8513610982'

# Streams to combine: observations + ephemeris
MOUNTPOINTS = ['AUTO_WGS84', 'BRDC']

# Local port the InertialSense node connects to
LOCAL_PORT = 2102

# Approximate rover position (used for VRS GGA sentence)
APPROX_LAT =  34.682
APPROX_LON = -82.861
APPROX_ALT = 183.0


def nmea_checksum(sentence):
    """Calculate NMEA checksum (XOR of all chars between $ and *)."""
    cs = 0
    for c in sentence:
        cs ^= ord(c)
    return f'{cs:02X}'


def make_gga():
    """Build a NMEA GGA sentence with the approximate rover position."""
    now = datetime.datetime.utcnow()
    hhmmss = now.strftime('%H%M%S.00')

    lat = abs(APPROX_LAT)
    lat_deg = int(lat)
    lat_min = (lat - lat_deg) * 60
    lat_hem = 'N' if APPROX_LAT >= 0 else 'S'

    lon = abs(APPROX_LON)
    lon_deg = int(lon)
    lon_min = (lon - lon_deg) * 60
    lon_hem = 'E' if APPROX_LON >= 0 else 'W'

    body = (f'GPGGA,{hhmmss},'
            f'{lat_deg:02d}{lat_min:07.4f},{lat_hem},'
            f'{lon_deg:03d}{lon_min:07.4f},{lon_hem},'
            f'1,14,1.0,{APPROX_ALT:.1f},M,0.0,M,,')
    return f'${body}*{nmea_checksum(body)}\r\n'


def ntrip_connect(mountpoint, retry_delay=5):
    """Connect to NTRIP caster using NTRIP 2.0 with GGA in headers. Retries on failure."""
    credentials = base64.b64encode(f'{USERNAME}:{PASSWORD}'.encode()).decode()
    while True:
        try:
            gga = make_gga().strip()
            request = (
                f'GET /{mountpoint} HTTP/1.1\r\n'
                f'Host: {GEODNET_HOST}:{GEODNET_PORT}\r\n'
                f'Ntrip-Version: Ntrip/2.0\r\n'
                f'User-Agent: NTRIP PythonProxy/1.0\r\n'
                f'Authorization: Basic {credentials}\r\n'
                f'Ntrip-GGA: {gga}\r\n'
                f'Connection: keep-alive\r\n'
                f'\r\n'
            )
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(15)
            sock.connect((GEODNET_HOST, GEODNET_PORT))
            sock.sendall(request.encode())
            # Read response headers
            response = b''
            while b'\r\n\r\n' not in response:
                chunk = sock.recv(1024)
                if not chunk:
                    raise ConnectionError('Empty response')
                response += chunk
            if b'200' not in response and b'ICY' not in response:
                raise ConnectionError(f'Bad response: {response[:200]}')
            sock.settimeout(None)
            print(f'[proxy] Connected to GEODNET/{mountpoint} — response: {response[:100]}')
            return sock
        except Exception as e:
            print(f'[proxy] Failed to connect to {mountpoint}: {e} — retrying in {retry_delay}s')
            time.sleep(retry_delay)
            time.sleep(retry_delay)


def gga_sender(ntrip_sock, stop_event, interval=10):
    """Periodically send GGA to keep VRS stream alive."""
    while not stop_event.is_set():
        time.sleep(interval)
        if stop_event.is_set():
            break
        try:
            ntrip_sock.sendall(make_gga().encode())
        except Exception:
            break


def forward_stream(mountpoint, client_sock, lock, stop_event):
    """Connect to NTRIP mountpoint and forward data to client. Reconnects on drop."""
    while not stop_event.is_set():
        ntrip_sock = None
        try:
            ntrip_sock = ntrip_connect(mountpoint)
            # Keep VRS alive by sending GGA every 10s
            gga_thread = threading.Thread(
                target=gga_sender, args=(ntrip_sock, stop_event), daemon=True)
            gga_thread.start()
            total_bytes = 0
            while not stop_event.is_set():
                data = ntrip_sock.recv(4096)
                if not data:
                    print(f'[proxy] {mountpoint} stream closed after {total_bytes} bytes, reconnecting...')
                    break
                total_bytes += len(data)
                with lock:
                    try:
                        client_sock.sendall(data)
                    except Exception:
                        stop_event.set()
                        return
        except Exception as e:
            if not stop_event.is_set():
                print(f'[proxy] {mountpoint} error: {e}, reconnecting...')
            time.sleep(2)
        finally:
            if ntrip_sock:
                try:
                    ntrip_sock.close()
                except Exception:
                    pass


def handle_client(client_sock, addr):
    """Handle incoming InertialSense connection."""
    print(f'[proxy] InertialSense connected from {addr}')
    try:
        # Read and discard the NTRIP request from InertialSense
        request = b''
        client_sock.settimeout(3)
        try:
            while b'\r\n\r\n' not in request:
                chunk = client_sock.recv(1024)
                if not chunk:
                    break
                request += chunk
        except socket.timeout:
            pass
        client_sock.settimeout(None)

        # Send ICY 200 OK to satisfy the NTRIP handshake
        client_sock.sendall(b'ICY 200 OK\r\n\r\n')

        lock = threading.Lock()
        stop_event = threading.Event()
        threads = []

        for mountpoint in MOUNTPOINTS:
            t = threading.Thread(
                target=forward_stream,
                args=(mountpoint, client_sock, lock, stop_event),
                daemon=True
            )
            t.start()
            threads.append(t)

        for t in threads:
            t.join()

    except Exception as e:
        print(f'[proxy] Client handler error: {e}')
    finally:
        client_sock.close()
        print(f'[proxy] InertialSense disconnected from {addr}')


def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', LOCAL_PORT))
    server.listen(1)
    print(f'[proxy] NTRIP multiplexer listening on localhost:{LOCAL_PORT}')
    print(f'[proxy] Combining streams: {MOUNTPOINTS}')

    while True:
        client_sock, addr = server.accept()
        t = threading.Thread(target=handle_client, args=(client_sock, addr), daemon=True)
        t.start()


if __name__ == '__main__':
    main()

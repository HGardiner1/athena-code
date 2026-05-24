#!/usr/bin/env python3
import argparse
import math
import time
import socket
import struct as st

CAN_CMD_ID       = 0x110
CAN_RESP_ID      = 0x110
CMD_READ_DIODE   = 0x20
RESPONSE_DELAY_S = 0.001

CAN_FRAME_FMT  = "=IH2x8s"
CAN_FRAME_SIZE = st.calcsize(CAN_FRAME_FMT)

def open_can_socket(interface):
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((interface,))
    sock.settimeout(30.0)
    print(f"[SIM] Opened CAN socket on {interface}")
    return sock

def send_frame(sock, can_id, data):
    dlc    = len(data)
    padded = data.ljust(8, b'\x00')
    frame  = st.pack(CAN_FRAME_FMT, can_id, dlc, padded)
    sock.send(frame)

def recv_frame(sock):
    try:
        raw = sock.recv(CAN_FRAME_SIZE)
        can_id, dlc, data = st.unpack(CAN_FRAME_FMT, raw)
        can_id &= socket.CAN_EFF_MASK
        return can_id, dlc, data[:dlc]
    except TimeoutError:
        return None

class IntensityGenerator:
    def __init__(self, pattern, fixed_value=128):
        self.pattern     = pattern
        self.fixed_value = fixed_value
        self.step        = 0

    def next(self):
        val = 0
        if self.pattern == "fixed":
            val = self.fixed_value
        elif self.pattern == "ramp":
            val = self.step % 256
        elif self.pattern == "sine":
            val = int(127.5 + 117.5 * math.sin(2 * math.pi * self.step / 100))
        elif self.pattern == "noise":
            val = (self.step * 37 + 13) % 256
        elif self.pattern == "gaussian":
            center = 100
            width  = 20
            pos    = self.step % 200
            val    = int(240 * math.exp(-0.5 * ((pos - center) / width) ** 2) + 10)
        self.step += 1
        return min(255, max(0, val))

def build_response_frame(port_id, intensity):
    return bytes([(CMD_READ_DIODE + port_id) & 0xFF, intensity & 0xFF])

def run_simulator(interface, port_id, pattern, fixed_value, delay):
    global RESPONSE_DELAY_S
    RESPONSE_DELAY_S = delay

    sock      = open_can_socket(interface)
    generator = IntensityGenerator(pattern, fixed_value)

    print(f"[SIM] Sensor Diode Simulator")
    print(f"[SIM] Listening on CAN ID 0x{CAN_CMD_ID:03X}")
    print(f"[SIM] Responding on CAN ID 0x{CAN_RESP_ID:03X}")
    print(f"[SIM] Port ID: {port_id}")
    print(f"[SIM] Pattern: {pattern}" + (f" (value={fixed_value})" if pattern == "fixed" else ""))
    print(f"[SIM] Response delay: {RESPONSE_DELAY_S * 1000:.1f} ms")
    print(f"[SIM] Press Ctrl+C to stop\n")

    request_count = 0

    try:
        while True:
            result = recv_frame(sock)
            if result is None:
                print("[SIM] Timeout — still listening...")
                continue

            can_id, dlc, data = result

            if can_id != CAN_CMD_ID:
                continue
            if dlc < 2:
                print(f"[SIM] Short frame on 0x{can_id:03X}, ignoring")
                continue

            cmd_byte = data[0]
            validate = data[1]

            if validate == 0:
                print(f"[SIM] Unvalidated request (cmd=0x{cmd_byte:02X}), ignoring")
                continue

            received_port = cmd_byte & 0x0F
            cmd_base      = cmd_byte & 0xF0

            if cmd_base != CMD_READ_DIODE:
                print(f"[SIM] Unknown command 0x{cmd_byte:02X}, ignoring")
                continue

            if received_port != port_id:
                print(f"[SIM] Request for port {received_port}, we are port {port_id}, ignoring")
                continue

            intensity = generator.next()
            time.sleep(RESPONSE_DELAY_S)

            send_frame(sock, CAN_RESP_ID, build_response_frame(port_id, intensity))

            request_count += 1
            print(f"[SIM] Request #{request_count} → port={port_id}, intensity={intensity} (0x{intensity:02X})")

    except KeyboardInterrupt:
        print(f"\n[SIM] Stopped — served {request_count} requests")
    finally:
        sock.close()
        print("[SIM] CAN socket closed")

def main():
    parser = argparse.ArgumentParser(description="Sensor Diode CAN Bus Simulator")
    parser.add_argument("--interface", "-i", default="can0")
    parser.add_argument("--port", "-p", type=int, default=0)
    parser.add_argument("--pattern", choices=["fixed", "ramp", "sine", "noise", "gaussian"], default="sine")
    parser.add_argument("--value", "-v", type=int, default=128)
    parser.add_argument("--delay", "-d", type=float, default=RESPONSE_DELAY_S)

    args = parser.parse_args()

    run_simulator(
        interface   = args.interface,
        port_id     = args.port,
        pattern     = args.pattern,
        fixed_value = args.value,
        delay       = args.delay
    )

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
CCD CAN Bus Simulator
=====================
Simulates the PCB-side CAN responses for the CCD hardware interface.

Protocol:
  Listen on CAN ID 0x100 for:
    - 0x20 (CMD_REQUEST_BINARY): triggers 1 ACK + 66 binary data frames
    - 0x30 (CMD_REQUEST_BYTE):   triggers 1 ACK + 608 byte data frames

  ACK frame:       CAN ID 0x101, data[0]=cmd_byte, data[1]=1 (success)
  Binary frames:   CAN ID 0x102, data[0]=frame_id, data[1..7]=56 bits of pixel data
  Byte frames:     CAN ID 0x103, data[0]=frame_id_low, data[1]=frame_id_high,
                                 data[2..7]=6 pixel intensity bytes

Usage:
  # Real CAN interface
  python3 ccd_can_simulator.py --interface can0

  # Virtual CAN (for testing without hardware)
  sudo modprobe vcan
  sudo ip link add dev can0 type vcan
  sudo ip link set up can0
  python3 ccd_can_simulator.py --interface can0

  # Specify mode explicitly
  python3 ccd_can_simulator.py --interface can0 --mode byte
  python3 ccd_can_simulator.py --interface can0 --mode binary
"""

import argparse
import struct
import time
import socket
import struct as st
import sys

# ── CAN constants ────────────────────────────────────────────────────────────
CAN_CMD_ID        = 0x100   # HWI sends requests here
CAN_ACK_ID        = 0x101   # We send ACK here
CAN_BINARY_ID     = 0x102   # We send binary data frames here
CAN_BYTE_ID       = 0x103   # We send byte data frames here

CMD_REQUEST_BINARY = 0x20
CMD_REQUEST_BYTE   = 0x30

# CCD sensor parameters
NUM_PIXELS         = 3648
BINARY_FRAMES      = 66     # ceil(3648 / 56)
BYTE_FRAMES        = 608    # ceil(3648 / 6)
PIXELS_PER_BINARY_FRAME = 56
PIXELS_PER_BYTE_FRAME   = 6

# Timing
INTER_FRAME_DELAY_S = 0.001  # 1ms between frames — adjust to match real PCB


# ── CAN socket helpers ───────────────────────────────────────────────────────

# Raw CAN frame format: <IH2x8s> = id(4) + dlc(2) + pad(2) + data(8)
CAN_FRAME_FMT = "=IH2x8s"
CAN_FRAME_SIZE = st.calcsize(CAN_FRAME_FMT)

def open_can_socket(interface: str) -> socket.socket:
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((interface,))
    sock.settimeout(30.0)
    print(f"[SIM] Opened CAN socket on {interface}")
    return sock

def send_frame(sock: socket.socket, can_id: int, data: bytes) -> None:
    dlc = len(data)
    padded = data.ljust(8, b'\x00')
    frame = st.pack(CAN_FRAME_FMT, can_id, dlc, padded)
    sock.send(frame)

def recv_frame(sock: socket.socket):
    """Returns (can_id, dlc, data) or None on timeout."""
    try:
        raw = sock.recv(CAN_FRAME_SIZE)
        can_id, dlc, data = st.unpack(CAN_FRAME_FMT, raw)
        can_id &= socket.CAN_EFF_MASK  # strip flags
        return can_id, dlc, data[:dlc]
    except TimeoutError:
        return None


# ── Pixel data generators ────────────────────────────────────────────────────

def generate_binary_pixels(num_pixels: int = NUM_PIXELS):
    """
    Generates a simple test pattern: alternating 0/1 bits.
    Replace with any pattern you want to test (e.g. all bright, all dark,
    Gaussian peak, ramp, etc.)
    """
    return [(i % 2) for i in range(num_pixels)]

def generate_byte_pixels(num_pixels: int = NUM_PIXELS):
    """
    Generates a synthetic Raman-like spectrum:
    - Gaussian peak around pixel 1800 (roughly mid-range)
    - Background noise floor
    Replace with a flat, ramp, or real reference spectrum as needed.
    """
    import math
    pixels = []
    peak_center = num_pixels // 2
    peak_width  = num_pixels // 10
    for i in range(num_pixels):
        gaussian = 200 * math.exp(-0.5 * ((i - peak_center) / peak_width) ** 2)
        noise    = (i * 7 + 13) % 20   # deterministic noise
        value    = int(gaussian + noise + 10)
        pixels.append(min(255, max(0, value)))
    return pixels


# ── Frame builders ───────────────────────────────────────────────────────────

def build_ack_frame(cmd_byte: int, success: bool = True):
    return bytes([cmd_byte, 1 if success else 0])

def build_binary_data_frame(frame_id: int, pixels: list, start_pixel: int):
    """
    Packs 56 binary pixel values into 7 bytes (8 pixels per byte), little-endian.
    frame_id fits in 1 byte (0-65, since 66 frames total).
    """
    data = bytearray(8)
    data[0] = frame_id & 0xFF
    for byte_idx in range(7):
        packed = 0
        for bit in range(8):
            pixel_idx = start_pixel + byte_idx * 8 + bit
            if pixel_idx < len(pixels) and pixels[pixel_idx]:
                packed |= (1 << bit)
        data[1 + byte_idx] = packed
    return bytes(data)

def build_byte_data_frame(frame_id: int, pixels: list, start_pixel: int):
    """
    Packs 6 byte-intensity pixel values.
    frame_id is uint16 split across data[0] (low) and data[1] (high).
    """
    data = bytearray(8)
    data[0] = frame_id & 0xFF
    data[1] = (frame_id >> 8) & 0xFF
    for i in range(6):
        pixel_idx = start_pixel + i
        data[2 + i] = pixels[pixel_idx] if pixel_idx < len(pixels) else 0
    return bytes(data)


# ── Acquisition simulators ───────────────────────────────────────────────────

def simulate_binary_acquisition(sock: socket.socket, port_id: int):
    print(f"[SIM] Binary acquisition requested (port {port_id})")

    # 1. Send ACK
    ack = build_ack_frame(CMD_REQUEST_BINARY, success=True)
    send_frame(sock, CAN_ACK_ID, ack)
    print(f"[SIM] Sent ACK on 0x{CAN_ACK_ID:03X}")
    time.sleep(INTER_FRAME_DELAY_S)

    # 2. Generate pixel data
    pixels = generate_binary_pixels()

    # 3. Send 66 data frames on 0x102
    for frame_id in range(BINARY_FRAMES):
        start_pixel = frame_id * PIXELS_PER_BINARY_FRAME
        frame_data  = build_binary_data_frame(frame_id, pixels, start_pixel)
        send_frame(sock, CAN_BINARY_ID, frame_data)

        if frame_id % 10 == 0 or frame_id == BINARY_FRAMES - 1:
            print(f"[SIM]   Binary frame {frame_id + 1}/{BINARY_FRAMES} "
                  f"(pixels {start_pixel}–{start_pixel + PIXELS_PER_BINARY_FRAME - 1})")

        time.sleep(INTER_FRAME_DELAY_S)

    print(f"[SIM] Binary acquisition complete — {BINARY_FRAMES} frames sent")

def simulate_byte_acquisition(sock: socket.socket, port_id: int):
    print(f"[SIM] 8-bit acquisition requested (port {port_id})")

    # 1. Send ACK
    ack = build_ack_frame(CMD_REQUEST_BYTE, success=True)
    send_frame(sock, CAN_ACK_ID, ack)
    print(f"[SIM] Sent ACK on 0x{CAN_ACK_ID:03X}")
    time.sleep(INTER_FRAME_DELAY_S)

    # 2. Generate pixel data
    pixels = generate_byte_pixels()

    # 3. Send 608 data frames on 0x103
    for frame_id in range(BYTE_FRAMES):
        start_pixel = frame_id * PIXELS_PER_BYTE_FRAME
        frame_data  = build_byte_data_frame(frame_id, pixels, start_pixel)
        send_frame(sock, CAN_BYTE_ID, frame_data)

        if frame_id % 50 == 0 or frame_id == BYTE_FRAMES - 1:
            print(f"[SIM]   Byte frame {frame_id + 1}/{BYTE_FRAMES} "
                  f"(pixels {start_pixel}–{start_pixel + PIXELS_PER_BYTE_FRAME - 1})")

        time.sleep(INTER_FRAME_DELAY_S)

    print(f"[SIM] 8-bit acquisition complete — {BYTE_FRAMES} frames sent")



def run_simulator(interface: str, force_mode: str = None, delay: float = INTER_FRAME_DELAY_S):
    global INTER_FRAME_DELAY_S
    INTER_FRAME_DELAY_S = delay
    sock = open_can_socket(interface)
    print(f"[SIM] Listening for CCD requests on CAN ID 0x{CAN_CMD_ID:03X}...")
    print(f"[SIM] Inter-frame delay: {INTER_FRAME_DELAY_S * 1000:.1f} ms")
    print(f"[SIM] Press Ctrl+C to stop\n")

    try:
        while True:
            result = recv_frame(sock)
            if result is None:
                print("[SIM] Timeout waiting for request — still listening...")
                continue

            can_id, dlc, data = result

            if can_id != CAN_CMD_ID:
                continue  # ignore frames not addressed to CCD

            if dlc < 2:
                print(f"[SIM] Received short frame on 0x{can_id:03X}, ignoring")
                continue

            cmd_byte = data[0]
            validate  = data[1]

            if validate == 0:
                print(f"[SIM] Received unvalidated request (cmd=0x{cmd_byte:02X}), ignoring")
                continue

            # Determine port_id from lower nibble
            port_id  = cmd_byte & 0x0F
            cmd_base = cmd_byte & 0xF0

            if force_mode == "binary" or cmd_base == CMD_REQUEST_BINARY:
                simulate_binary_acquisition(sock, port_id)
            elif force_mode == "byte" or cmd_base == CMD_REQUEST_BYTE:
                simulate_byte_acquisition(sock, port_id)
            else:
                print(f"[SIM] Unknown command byte 0x{cmd_byte:02X}, ignoring")

    except KeyboardInterrupt:
        print("\n[SIM] Stopped by user")
    finally:
        sock.close()
        print("[SIM] CAN socket closed")


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="CCD CAN Bus Simulator")
    parser.add_argument(
        "--interface", "-i",
        default="can0",
        help="CAN interface name (default: can0)")
    parser.add_argument(
        "--mode", "-m",
        choices=["binary", "byte"],
        default=None,
        help="Force acquisition mode regardless of command byte (default: auto-detect)")
    parser.add_argument(
        "--delay", "-d",
        type=float,
        default=INTER_FRAME_DELAY_S,
        help=f"Inter-frame delay in seconds (default: {INTER_FRAME_DELAY_S})")

    args = parser.parse_args()

    run_simulator(args.interface, force_mode=args.mode)

if __name__ == "__main__":
    main()
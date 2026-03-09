"""
test_can_connection.py

Simple CAN connectivity test between the RPi5 (MCP2515 via SPI → SocketCAN) and the STM32.

One-time RPi5 setup
-------------------
1. Enable the MCP2515 overlay in /boot/firmware/config.txt:

       dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25
       #                                 ^^^^^^^^           ^^
       #   Match your MCP2515 crystal (8 or 16 MHz)    GPIO pin wired to INT

   For SPI1 (CE1) use  dtoverlay=mcp2515-can1,oscillator=8000000,interrupt=25

2. Reboot, then bring the interface up (repeat after every reboot, or add to /etc/network/interfaces):

       sudo ip link set can0 up type can bitrate 500000
       #                                         ^^^^^^  must match STM CAN config

   Verify with:  ip -d link show can0

Usage
-----
    python test_can_connection.py [--channel can0] [--bitrate 500000] [--mcu 1]
"""

import argparse
import time
import sys
import cantools
import can
from pathlib import Path

# ── DBC path (relative to this file) ────────────────────────────────────────
DBC_PATH = Path(__file__).parent.parent / "app" / "common" / "omni_robot.dbc"

# ── Defaults ─────────────────────────────────────────────────────────────────
DEFAULT_CHANNEL  = "can0"
DEFAULT_BITRATE  = 500_000   # match STM CAN peripheral config
DEFAULT_MCU      = 1           # which MCU joint to ping (1-4)
RX_TIMEOUT_S     = 2.0         # seconds to wait for a status reply


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="RPi5 ↔ STM32 CAN connectivity test")
    p.add_argument("--channel", default=DEFAULT_CHANNEL,
                   help=f"SocketCAN interface (default: {DEFAULT_CHANNEL})")
    p.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE,
                   help=f"CAN bitrate in bps (default: {DEFAULT_BITRATE})")
    p.add_argument("--mcu", type=int, choices=[1, 2, 3, 4], default=DEFAULT_MCU,
                   help="MCU joint index to test (1-4, default: 1)")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    # ── Load DBC ─────────────────────────────────────────────────────────────
    print(f"[INFO] Loading DBC: {DBC_PATH}")
    db = cantools.database.load_file(str(DBC_PATH))

    cmd_name    = f"RPi_Command_{args.mcu}"   # e.g. RPi_Command_1
    status_name = f"MCU_Status_{args.mcu}"    # e.g. MCU_Status_1

    cmd_msg    = db.get_message_by_name(cmd_name)
    status_msg = db.get_message_by_name(status_name)

    print(f"[INFO] TX  → {cmd_name}   (ID=0x{cmd_msg.frame_id:03X})")
    print(f"[INFO] RX  ← {status_name} (ID=0x{status_msg.frame_id:03X})")

    # ── Open CAN bus (MCP2515 exposes as SocketCAN on RPi) ───────────────────
    try:
        bus = can.Bus(
            interface="socketcan",
            channel=args.channel,
            bitrate=args.bitrate,
        )
    except OSError as exc:
        print(f"[ERROR] Could not open {args.channel}: {exc}")
        print("        1. Check /boot/firmware/config.txt has:")
        print("             dtoverlay=mcp2515-can0,oscillator=<8000000|16000000>,interrupt=<gpio>")
        print("        2. Bring the interface up:")
        print(f"             sudo ip link set {args.channel} up type can bitrate {args.bitrate}")
        return 1

    print(f"[INFO] CAN bus opened on {args.channel} @ {args.bitrate} bps\n")

    # ── Test 1: Send 10 RPi_Commands, angle 0→90° in 10° steps ─────────────
    print("=== Test 1: TX – send 10 RPi_Commands (0° to 90°, 10° steps) ===")
    tx_ok = True
    for i in range(10):
        angle = float(i * 10)
        payload = cmd_msg.encode({
            "TargetAngle_deg": angle,
            "MoveDuration_ms": 500,
            "TorqueEnable"   : 1,
            "StopCmd"        : 0,
        })
        tx_frame = can.Message(
            arbitration_id=cmd_msg.frame_id,
            data=payload,
            is_extended_id=False,
        )
        try:
            bus.send(tx_frame)
            print(f"  [{i+1:2d}/10] TargetAngle={angle:5.1f}°  data={payload.hex(' ').upper()}")
        except can.CanError as exc:
            print(f"  [{i+1:2d}/10] [FAIL] TX error: {exc}")
            tx_ok = False
            break
        if i < 9:
            time.sleep(0.1)

    if not tx_ok:
        bus.shutdown()
        return 1
    print("  [PASS] All 10 frames sent\n")

    # ── Test 2: Receive a status frame ───────────────────────────────────────
    print(f"=== Test 2: RX – wait for {status_name} (timeout {RX_TIMEOUT_S}s) ===")
    deadline = time.monotonic() + RX_TIMEOUT_S
    rx_frame = None

    while time.monotonic() < deadline:
        frame = bus.recv(timeout=deadline - time.monotonic())
        if frame is None:
            break
        if frame.arbitration_id == status_msg.frame_id:
            rx_frame = frame
            break

    if rx_frame is None:
        print(f"  [FAIL] No {status_name} received within {RX_TIMEOUT_S}s")
        print("         Check STM firmware is running and CAN termination is present.")
        bus.shutdown()
        return 1

    try:
        decoded = db.decode_message(rx_frame.arbitration_id, rx_frame.data)
    except Exception as exc:
        print(f"  [FAIL] Decode error: {exc}")
        bus.shutdown()
        return 1

    print(f"  Received : {rx_frame}")
    print(f"  Decoded  :")
    for signal, value in decoded.items():
        print(f"    {signal:<20s} = {value}")
    print("  [PASS] RX succeeded\n")

    # ── Summary ──────────────────────────────────────────────────────────────
    bus.shutdown()
    print("=== All tests passed – CAN link is operational ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())

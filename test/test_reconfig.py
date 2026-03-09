"""
test_reconfig.py

Interactive reconfiguration test for the omni-robot arm.

Tests that the MCU correctly activates / deactivates servo slots when an
RPi_Reconfig (0x300) CAN frame is sent with a new servo-ID list.

Verification
------------
MCU_Status_1..6 frames are monitored for a configurable window after each
reconfig.  A slot is considered ACTIVE if at least one MCU_Status frame
for that slot is received within the window, and ABSENT otherwise.

Reconfig semantics
------------------
RPi_Reconfig carries six fields (ServoId_1 .. ServoId_6).
  • A non-zero value in slot N tells the MCU to probe servo bus ID N.
  • A value of 0 means "this slot should be absent".
After the reconfig the MCU re-scans the bus and only publishes MCU_Status
for slots whose physical servo responded to the ping.

One-time RPi5 setup
-------------------
    dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25
    sudo ip link set can0 up type can bitrate 500000

Usage examples
--------------
    # All 6 motors connected; remove motor 3, then add it back:
    python test_reconfig.py --present-ids 1,2,3,4,5,6 --remove-id 3

    # Only motors 1 and 2 available:
    python test_reconfig.py --present-ids 1,2 --remove-id 1
"""

import argparse
import sys
import time
from pathlib import Path

import can
import cantools

# ── DBC ─────────────────────────────────────────────────────────────────────
DBC_PATH = Path(__file__).parent.parent / "app" / "common" / "omni_robot.dbc"

# ── Defaults ─────────────────────────────────────────────────────────────────
DEFAULT_CHANNEL     = "can0"
DEFAULT_BITRATE     = 500_000
DEFAULT_PRESENT_IDS = "1,2,3,4,5,6"   # servo bus IDs physically on the bus
DEFAULT_REMOVE_ID   = 1                # which ID to unplug during the test
RECONFIG_SETTLE_S   = 2.0              # seconds to wait after sending RPi_Reconfig
RX_WINDOW_S         = 3.0             # seconds to collect MCU_Status frames


# ── Helpers ──────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="MCU reconfiguration test")
    p.add_argument("--channel", default=DEFAULT_CHANNEL,
                   help=f"SocketCAN interface (default: {DEFAULT_CHANNEL})")
    p.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE,
                   help=f"CAN bitrate in bps (default: {DEFAULT_BITRATE})")
    p.add_argument("--present-ids", default=DEFAULT_PRESENT_IDS,
                   help="Comma-separated servo bus IDs that are physically "
                        f"connected (default: {DEFAULT_PRESENT_IDS})")
    p.add_argument("--remove-id", type=int, default=DEFAULT_REMOVE_ID,
                   help=f"Servo bus ID to unplug during the partial-reconfig "
                        f"step (default: {DEFAULT_REMOVE_ID})")
    p.add_argument("--rx-window", type=float, default=RX_WINDOW_S,
                   help=f"Seconds to collect MCU_Status frames (default: {RX_WINDOW_S})")
    p.add_argument("--settle", type=float, default=RECONFIG_SETTLE_S,
                   help=f"Seconds to wait for the MCU to finish rescanning "
                        f"after reconfig (default: {RECONFIG_SETTLE_S})")
    return p.parse_args()


def build_reconfig_payload(db: cantools.db.Database,
                           active_ids: list[int]) -> bytes:
    """
    Encode an RPi_Reconfig frame where ServoId_N = N if N is in active_ids,
    else 0.
    """
    msg = db.get_message_by_name("RPi_Reconfig")
    signals = {}
    for slot in range(1, 7):
        signals[f"ServoId_{slot}"] = slot if slot in active_ids else 0
    return msg.encode(signals)


def send_reconfig(bus: can.Bus, db: cantools.db.Database,
                  active_ids: list[int], label: str) -> None:
    msg      = db.get_message_by_name("RPi_Reconfig")
    payload  = build_reconfig_payload(db, active_ids)
    frame    = can.Message(arbitration_id=msg.frame_id,
                           data=payload,
                           is_extended_id=False)
    bus.send(frame)
    print(f"  [TX] RPi_Reconfig {label}: active IDs = {sorted(active_ids)}"
          f"  data = {payload.hex(' ').upper()}")


def collect_active_slots(bus: can.Bus, db: cantools.db.Database,
                         window_s: float) -> set[int]:
    """
    Listen on the bus for `window_s` seconds.
    Returns the set of MCU slot indices (1-based) from which at least one
    MCU_Status frame was received.
    """
    # Build a lookup: CAN ID → slot number
    status_id_to_slot: dict[int, int] = {}
    for slot in range(1, 7):
        msg = db.get_message_by_name(f"MCU_Status_{slot}")
        status_id_to_slot[msg.frame_id] = slot

    active: set[int] = set()
    deadline = time.monotonic() + window_s

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        frame = bus.recv(timeout=max(remaining, 0))
        if frame is None:
            break
        slot = status_id_to_slot.get(frame.arbitration_id)
        if slot is not None:
            active.add(slot)

    return active


def prompt_continue(message: str) -> None:
    print(f"\n{'─' * 60}")
    print(f"  ACTION REQUIRED: {message}")
    input("  Press Enter when ready...")
    print('─' * 60)


def check(label: str, expected: set[int], actual: set[int]) -> bool:
    ok = expected == actual
    status = "[PASS]" if ok else "[FAIL]"
    print(f"\n  {status} {label}")
    print(f"    Expected active slots : {sorted(expected)}")
    print(f"    Observed active slots : {sorted(actual)}")
    if not ok:
        missing  = expected - actual
        extra    = actual - expected
        if missing:
            print(f"    Missing (no MCU_Status): slots {sorted(missing)}")
        if extra:
            print(f"    Unexpected (spurious MCU_Status): slots {sorted(extra)}")
    return ok


# ── Main ─────────────────────────────────────────────────────────────────────

def main() -> int:
    args = parse_args()

    # Parse --present-ids
    try:
        present_ids = [int(x.strip()) for x in args.present_ids.split(",") if x.strip()]
    except ValueError:
        print("[ERROR] --present-ids must be comma-separated integers, e.g. 1,2,3")
        return 1

    if args.remove_id not in present_ids:
        print(f"[ERROR] --remove-id {args.remove_id} is not in --present-ids {present_ids}")
        return 1

    partial_ids = [i for i in present_ids if i != args.remove_id]

    print(f"[INFO] Loading DBC: {DBC_PATH}")
    db = cantools.database.load_file(str(DBC_PATH))

    # Open the CAN bus
    try:
        bus = can.Bus(interface="socketcan",
                      channel=args.channel,
                      bitrate=args.bitrate)
    except OSError as exc:
        print(f"[ERROR] Could not open {args.channel}: {exc}")
        return 1

    print(f"[INFO] CAN bus opened on {args.channel} @ {args.bitrate} bps")
    print(f"[INFO] Physically present servo IDs : {sorted(present_ids)}")
    print(f"[INFO] Motor to be removed          : servo ID {args.remove_id}")
    print(f"[INFO] RX collection window         : {args.rx_window}s")
    print(f"[INFO] Reconfig settle time         : {args.settle}s\n")

    results: list[bool] = []

    # ── Step 1: Full reconfig ────────────────────────────────────────────────
    print("=" * 60)
    print("STEP 1 – Full reconfig: all physically present motors")
    print("=" * 60)
    prompt_continue(
        f"Make sure ALL motors ({sorted(present_ids)}) are connected, "
        "then press Enter to send the full RPi_Reconfig."
    )

    send_reconfig(bus, db, present_ids, "(full)")
    print(f"  [INFO] Waiting {args.settle}s for MCU to finish re-scanning...")
    time.sleep(args.settle)

    print(f"  [INFO] Collecting MCU_Status frames for {args.rx_window}s...")
    observed = collect_active_slots(bus, db, args.rx_window)
    expected = set(present_ids)
    results.append(check("Full reconfig – all present motors should be active",
                         expected, observed))

    # ── Step 2: Partial reconfig (remove one motor) ──────────────────────────
    print("\n" + "=" * 60)
    print(f"STEP 2 – Partial reconfig: remove motor {args.remove_id}")
    print("=" * 60)
    prompt_continue(
        f"Physically DISCONNECT motor with servo bus ID {args.remove_id}, "
        "then press Enter to send the partial RPi_Reconfig."
    )

    send_reconfig(bus, db, partial_ids, f"(without ID {args.remove_id})")
    print(f"  [INFO] Waiting {args.settle}s for MCU to finish re-scanning...")
    time.sleep(args.settle)

    print(f"  [INFO] Collecting MCU_Status frames for {args.rx_window}s...")
    observed = collect_active_slots(bus, db, args.rx_window)
    expected = set(partial_ids)
    results.append(check(
        f"Partial reconfig – motor {args.remove_id} should be absent, "
        f"remaining {sorted(partial_ids)} should be active",
        expected, observed
    ))

    # ── Step 3: Re-add the removed motor ────────────────────────────────────
    print("\n" + "=" * 60)
    print(f"STEP 3 – Re-add motor {args.remove_id}")
    print("=" * 60)
    prompt_continue(
        f"Physically RECONNECT motor with servo bus ID {args.remove_id}, "
        "then press Enter to send the full RPi_Reconfig again."
    )

    send_reconfig(bus, db, present_ids, "(full, re-add)")
    print(f"  [INFO] Waiting {args.settle}s for MCU to finish re-scanning...")
    time.sleep(args.settle)

    print(f"  [INFO] Collecting MCU_Status frames for {args.rx_window}s...")
    observed = collect_active_slots(bus, db, args.rx_window)
    expected = set(present_ids)
    results.append(check(
        f"Re-add motor {args.remove_id} – all {sorted(present_ids)} should be active again",
        expected, observed
    ))

    # ── Summary ──────────────────────────────────────────────────────────────
    bus.shutdown()
    print("\n" + "=" * 60)
    passed = sum(results)
    total  = len(results)
    print(f"RESULT: {passed}/{total} tests passed")
    if passed == total:
        print("  All reconfiguration tests PASSED.")
    else:
        print("  One or more tests FAILED. See details above.")
    print("=" * 60)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())

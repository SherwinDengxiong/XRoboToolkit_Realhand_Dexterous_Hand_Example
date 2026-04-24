"""Hardware teleop for a single RealHand L6 via the new realhand SDK.

Example:
    # Bring up the CAN interface first, then:
    python scripts/hardware/teleop_realhand_l6_hardware.py --hand-type right \
        --interface-name can0
"""

import threading
import time

import tyro

from xrobotoolkit_teleop.common.xr_client import XrClient
from xrobotoolkit_teleop.hardware.realhand_l6_controller import RealHandL6Controller


def main(
    hand_type: str = "right",
    interface_name: str = "can0",
    control_hz: float = 60.0,
    reset: bool = False,
) -> None:
    """Drive a single RealHand L6 from XR hand tracking.

    Args:
        hand_type: ``"left"`` or ``"right"``.
        interface_name: SocketCAN interface attached to the hand (e.g. ``can0``).
        control_hz: Target loop rate for retarget + command threads.
        reset: If true, open the hand and exit without entering the control loop.
    """
    xr_client = XrClient()
    controller = RealHandL6Controller(
        xr_client,
        hand_type=hand_type,
        interface_name=interface_name,
        control_hz=control_hz,
    )

    if reset:
        print("Reset flag detected. Opening hand and exiting.")
        controller.reset()
        controller.close()
        return

    stop_event = threading.Event()
    retarget = threading.Thread(
        target=controller.run_retarget_thread, args=(stop_event,), daemon=True
    )
    control = threading.Thread(
        target=controller.run_control_thread, args=(stop_event,), daemon=True
    )

    retarget.start()
    control.start()

    try:
        while not stop_event.is_set():
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("KeyboardInterrupt. Shutting down...")
        stop_event.set()

    retarget.join()
    control.join()
    controller.close()
    print("RealHand L6 controller stopped.")


if __name__ == "__main__":
    tyro.cli(main)

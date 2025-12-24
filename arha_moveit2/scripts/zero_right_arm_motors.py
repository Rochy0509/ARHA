#!/usr/bin/env python3

import sys
from time import sleep

import myactuator_rmd_py as rmd


def main():
    try:
        driver = rmd.CanDriver("can0")
    except Exception as e:
        print(f"Failed to initialize CAN driver: {e}")
        return 1

    motors = {i: rmd.ActuatorInterface(driver, i) for i in range(1, 7)}

    def set_and_verify_zero(tolerance=0.1, max_attempts=3):
        for attempt in range(1, max_attempts + 1):
            print(f"\n=== Zeroing attempt {attempt} ===")
            ok = True

            for motor_id, motor in motors.items():
                try:
                    pos = motor.getMultiTurnAngle()
                    print(f"[set] Motor {motor_id} angle {pos}")
                    motor.setCurrentPositionAsEncoderZero()
                    sleep(0.1)
                except Exception as e:
                    print(f"Motor {motor_id} zero error: {e}")
                    ok = False

            print("Resetting motors...")
            for motor_id, motor in motors.items():
                try:
                    motor.reset()
                except Exception as e:
                    print(f"Motor {motor_id} reset error: {e}")
                    ok = False

            sleep(5.0)  # allow reset

            for motor_id, motor in motors.items():
                try:
                    pos = motor.getMultiTurnAngle()
                    print(f"[verify] Motor {motor_id} angle {pos}")
                    if abs(pos) > tolerance:
                        ok = False
                        print(f"Motor {motor_id} exceeds tolerance ±{tolerance}")
                except Exception as e:
                    print(f"Motor {motor_id} read error: {e}")
                    ok = False

            if ok:
                return True
            print("Retrying zeroing...")
        return False

    success = set_and_verify_zero()
    print("\nZeroing result:", "SUCCESS" if success else "FAILED")
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())

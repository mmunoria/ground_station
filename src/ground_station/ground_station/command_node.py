"""
    ros2 run ground_station command_node
    ros2 run ground_station command_node --ros-args \
        -p drone_domains:="[1,2]" -p drone_namespaces:="['drone1','drone2']"

    For the graphical interface (menu commands + per-drone setpoint/path
    dispatch), run `command_gui` instead -- it accepts the same parameters.
"""

import time

from .drone_targets import load_targets


def _broadcast(targets, text):
    for target in targets:
        target.publish_command(text)
    print(f"sent {text!r} to {len(targets)} target(s)")


def _run_menu(targets):
    print("=== Drone Command Node ===")
    for target in targets:
        print(f"  -> {target.describe()}")
    print()
    while True:
        print("  0) arm")
        print("  1) fly")
        print("  2) hover")
        print("  3) straight")
        print("  4) land")
        print("  5) custom command")
        print("  q) quit")
        try:
            choice = input("Select: ").strip().lower()
        except EOFError:
            break

        if choice in ("q", "quit", "exit"):
            break
        elif choice in ("0", "arm"):
                _broadcast(targets, "arm")

        elif choice in ("1", "fly"):
                _broadcast(targets, "fly")

        elif choice in ("2", "hover"):
            _broadcast(targets, "hover")

        elif choice in ("3", "straight"):
                _broadcast(targets, "straight")

        elif choice in ("4", "land"):
            _broadcast(targets, "land")

        elif choice in ("5", "custom"):
            try:
                text = input("Enter custom command string: ").strip()
            except EOFError:
                break
            if text:
                _broadcast(targets, text)
            else:
                print("(empty command, not sent)")
        else:
            print(f"Unrecognized option: {choice!r}")
        print()


def main(args=None):
    targets = load_targets(args)
    print(f"Waiting for {len(targets)} domain participant(s) to settle...")
    time.sleep(1.5)
    try:
        _run_menu(targets)
    except KeyboardInterrupt:
        pass
    finally:
        for target in targets:
            target.close()


if __name__ == "__main__":
    main()

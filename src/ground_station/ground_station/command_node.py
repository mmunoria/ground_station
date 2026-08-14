"""
    ros2 run ground_station command_node
    ros2 run ground_station command_node --ros-args \
        -p drone_domains:="[1,2]" -p drone_namespaces:="['drone1','drone2']"
"""

import time
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from std_msgs.msg import String


class _DomainTarget:

    def __init__(self, index, domain_id, namespace):
        self.domain_id = domain_id
        self.namespace = namespace
        self.topic = f"/{namespace}/command" if namespace else "/command"

        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain_id)
        suffix = domain_id if domain_id is not None else "default"
        self.node = rclpy.create_node(
            f"command_node_{index}_d{suffix}", context=self.context)
        self.pub = self.node.create_publisher(String, self.topic, 10)

    def describe(self):
        domain = self.domain_id if self.domain_id is not None else "default"
        return f"{self.topic} (domain {domain})"

    def publish(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)


def _load_targets(args):
    rclpy.init(args=args)
    cfg_node = rclpy.create_node("command_node_config")
    cfg_node.declare_parameter("drone_domains", Parameter.Type.INTEGER_ARRAY)
    cfg_node.declare_parameter("drone_namespaces", [""])
    domains_default = Parameter("drone_domains", Parameter.Type.INTEGER_ARRAY, [])
    domains = list(cfg_node.get_parameter_or("drone_domains", domains_default).value)
    namespaces = list(cfg_node.get_parameter("drone_namespaces").value)
    cfg_node.destroy_node()
    rclpy.shutdown()

    if not domains:
        domains = [None] * len(namespaces or [""])
        namespaces = namespaces or [""]
    elif len(namespaces) != len(domains):
        if namespaces in ([], [""]):
            namespaces = [""] * len(domains)
        else:
            raise ValueError(
                "drone_domains and drone_namespaces must be the same length "
                f"(got {len(domains)} domains, {len(namespaces)} namespaces)")

    return [
        _DomainTarget(i, d, ns)
        for i, (d, ns) in enumerate(zip(domains, namespaces))
    ]


def _broadcast(targets, text):
    for target in targets:
        target.publish(text)
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
        print("  3) land")
        print("  4) straight line")
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
        elif choice in ("3", "land"):
            _broadcast(targets, "land")
        elif choice in ("4", "straight"):
            _broadcast(targets, "straight")
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
    targets = _load_targets(args)
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


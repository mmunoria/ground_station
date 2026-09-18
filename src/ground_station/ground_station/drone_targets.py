"""
Shared multi-domain DDS plumbing for the ground_station command/setpoint tools.

Both the terminal menu (command_node.py) and the GUI (command_gui.py) build their
list of drones by calling `load_targets()`, which reads the `drone_domains` /
`drone_namespaces` parameters exactly as before and returns one `DomainTarget`
per (domain, namespace) pair. Each `DomainTarget` owns its own rclpy `Context`
bound to a single ROS_DOMAIN_ID -- that "one Context per domain" pattern is
unchanged -- and now publishes six things into that domain:

  * std_msgs/String        on  /{namespace}/command    (arm/hover/land/custom)
  * geometry_msgs/PoseStamped on /{namespace}/setpoint (single setpoint)
  * nav_msgs/Path          on  /{namespace}/path        (multi-waypoint path)
  * std_msgs/String        on  /{namespace}/mode        (manual/position/offboard)
  * std_msgs/String        on  /{namespace}/flight_type (hover/traverse)
  * std_msgs/Float64       on  /{namespace}/delay_per_setpoint (seconds)

Each `DomainTarget` also *subscribes* to that domain's /rosout
(rcl_interfaces/msg/Log). Every ROS 2 node publishes its `get_logger()` calls
there by default -- offboard/mocap/estimator nodes included -- so this one
subscription per domain, not per node, is enough to see every log message
(including errors) any node on that drone emits, with no change needed on
the drone side. Entries are buffered in `DomainTarget.log_entries` and handed
to the caller via `drain_new_logs()` -- see command_gui.py's log panel for
the consumer -- rather than acted on inline, so a slow UI can't stall the
executor.

All poses use frame_id "map" to match the OptiTrack mocap global frame.
"""

import collections
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import Log
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import Float64, String

FRAME_ID = "map"

LOG_LEVEL_NAMES = {
    Log.DEBUG: "DEBUG",
    Log.INFO: "INFO",
    Log.WARN: "WARN",
    Log.ERROR: "ERROR",
    Log.FATAL: "FATAL",
}


def yaw_to_quaternion(yaw_rad):
    """Yaw-only rotation (roll = pitch = 0) as an (x, y, z, w) quaternion tuple."""
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def make_pose_stamped(node, x, y, z, yaw_rad=0.0, frame_id=FRAME_ID):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    qx, qy, qz, qw = yaw_to_quaternion(yaw_rad)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class DomainTarget:
    """One ROS_DOMAIN_ID / namespace pair, with its own Context, node and executor."""

    def __init__(self, index, domain_id, namespace):
        self.index = index
        self.domain_id = domain_id
        self.namespace = namespace
        self.command_topic = f"/{namespace}/command" if namespace else "/command"
        self.setpoint_topic = f"/{namespace}/setpoint" if namespace else "/setpoint"
        self.path_topic = f"/{namespace}/path" if namespace else "/path"
        self.mode_topic = f"/{namespace}/mode" if namespace else "/mode"
        self.flight_type_topic = (
            f"/{namespace}/flight_type" if namespace else "/flight_type")
        self.delay_per_setpoint_topic = (
            f"/{namespace}/delay_per_setpoint" if namespace else "/delay_per_setpoint")

        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain_id)
        suffix = domain_id if domain_id is not None else "default"
        self.node = rclpy.create_node(
            f"command_node_{index}_d{suffix}", context=self.context)

        self.command_pub = self.node.create_publisher(String, self.command_topic, 10)
        self.pose_pub = self.node.create_publisher(PoseStamped, self.setpoint_topic, 10)
        self.path_pub = self.node.create_publisher(Path, self.path_topic, 10)
        self.mode_pub = self.node.create_publisher(String, self.mode_topic, 10)
        self.flight_type_pub = self.node.create_publisher(
            String, self.flight_type_topic, 10)
        self.delay_per_setpoint_pub = self.node.create_publisher(
            Float64, self.delay_per_setpoint_topic, 10)

        # /rosout -- see module docstring. One subscription per domain picks up
        # get_logger() calls from every node running in that domain (offboard,
        # mocap, estimator, ...), so error/warning monitoring needs no changes
        # on the drone side. Buffered rather than handled inline; drain with
        # drain_new_logs(). depth=50 gives the buffer headroom for a burst of
        # log lines between two polls, since rosout is BEST-effort-adjacent in
        # practice (RELIABLE/TRANSIENT_LOCAL upstream, but a busy node can log
        # faster than a slow consumer drains).
        self.log_entries = collections.deque(maxlen=2000)
        self.rosout_sub = self.node.create_subscription(
            Log, "/rosout", self._on_rosout, 50)

        # Not strictly required for pure publishing, but keeps the node's context
        # spinning (discovery, parameter services, future subscriptions) without
        # ever blocking the caller -- see spin_once().
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

    def describe(self):
        domain = self.domain_id if self.domain_id is not None else "default"
        return f"{self.command_topic} (domain {domain})"

    def label(self):
        """Short human-readable label for GUI dropdowns."""
        domain = self.domain_id if self.domain_id is not None else "default"
        ns = self.namespace or "(no namespace)"
        return f"domain {domain} — {ns}"

    def publish_command(self, text):
        msg = String()
        msg.data = text
        self.command_pub.publish(msg)

    # Old name, kept so any external code/tests calling target.publish(...) still work.
    publish = publish_command

    def publish_setpoint(self, x, y, z, yaw_rad=0.0):
        pose = make_pose_stamped(self.node, x, y, z, yaw_rad)
        self.pose_pub.publish(pose)
        return pose

    def publish_mode(self, text):
        """Vehicle mode command ("manual" / "position" / "offboard") on its own topic."""
        msg = String()
        msg.data = text
        self.mode_pub.publish(msg)

    def publish_flight_type(self, text):
        """Flight type ("hover" / "traverse") on its own topic."""
        msg = String()
        msg.data = text
        self.flight_type_pub.publish(msg)

    def publish_delay_per_setpoint(self, seconds):
        """Delay to hold at each setpoint before advancing, in seconds, on its own topic."""
        msg = Float64()
        msg.data = float(seconds)
        self.delay_per_setpoint_pub.publish(msg)

    def publish_path(self, waypoints):
        """waypoints: iterable of (x, y, z, yaw_rad) tuples, sent in order."""
        path = Path()
        path.header.frame_id = FRAME_ID
        path.header.stamp = self.node.get_clock().now().to_msg()
        path.poses = [
            make_pose_stamped(self.node, x, y, z, yaw_rad)
            for (x, y, z, yaw_rad) in waypoints
        ]
        self.path_pub.publish(path)
        return path

    def _on_rosout(self, msg):
        """Buffer one /rosout entry; see drain_new_logs() for consuming them."""
        stamp_sec = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.log_entries.append({
            "domain_id": self.domain_id,
            "namespace": self.namespace,
            "target_label": self.label(),
            "node": msg.name,
            "level": msg.level,
            "level_name": LOG_LEVEL_NAMES.get(msg.level, str(msg.level)),
            "text": msg.msg,
            "stamp_sec": stamp_sec,
        })

    def drain_new_logs(self):
        """Pop and return every /rosout entry received since the last call."""
        entries = list(self.log_entries)
        self.log_entries.clear()
        return entries

    def spin_once(self, timeout_sec=0.0):
        """Non-blocking pump for this domain's executor; safe to poll from a GUI loop."""
        self.executor.spin_once(timeout_sec=timeout_sec)

    def close(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)


def load_targets(args=None):
    """Parse drone_domains / drone_namespaces and build one DomainTarget per entry."""
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
        DomainTarget(i, d, ns)
        for i, (d, ns) in enumerate(zip(domains, namespaces))
    ]

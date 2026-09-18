"""
Graphical front-end for the ground_station command node.

    ros2 run ground_station command_gui
    ros2 run ground_station command_gui --ros-args \
        -p drone_domains:="[1,2,3,4,5,6,7]" \
        -p drone_namespaces:="['drone1','drone2','drone3','drone4','drone5','drone6','drone7']"

Everything the terminal menu (command_node.py) offers -- arm/fly/hover/straight
/land plus arbitrary custom command strings, broadcast to every configured
domain -- is available here as buttons/fields. Alongside those, a "Mode" row
of buttons (manual/position/offboard) broadcasts on each domain's /mode topic,
also not tied to any file. In addition, a per-drone panel lets you pick one
domain by ID and send it a single geometry_msgs/PoseStamped setpoint or build
up a nav_msgs/Path of waypoints and send that.

A "Flight formation" panel loads one of the JSON files described in
../../../flight_formations/README.md (schema_version/run_number/flight_type/
delay_per_setpoint/drones[]/meta_data) and dispatches every drone in it in
one click: each drones[] entry's drone_id is matched to a configured
DomainTarget (see _target_for_drone_id) and sent a single setpoint for its
first setpoints_m entry (flight_type "hover", to hold position) plus a
"speed <mps>" command when traversing (flight_type "traverse"). Either way,
every setpoints_m entry -- one or many -- is also mirrored onto /path in
order, so /path is never left empty (or truncated) after a formation
dispatch, and the file's flight_type and delay_per_setpoint (when present)
are published as-is on each dispatched drone's /flight_type and
/delay_per_setpoint topics -- there is no separate UI control for either,
the formation file is the only source for them.

An "Error / warning log" panel gives live, per-drone monitoring of every
node's log output. Every ROS 2 node publishes its get_logger() calls to its
domain's /rosout by default (rcl_interfaces/msg/Log) -- offboard/mocap/
estimator nodes included -- so each DomainTarget's single /rosout
subscription (see drone_targets.py) already sees every log line, including
errors, that any node on that drone emits, with no per-topic wiring and no
changes needed on the drone side. Rows are tagged with the originating
domain/namespace and node name, and are colored by severity; the "Min
level" dropdown filters what gets added to the table (default: Warning+).

Threading model
----------------
Rather than spinning each domain's node on a background thread (which would
need cross-thread locking to touch Qt widgets safely), this GUI runs entirely
on the Qt main thread and drives each DomainTarget's executor with a QTimer
that calls `spin_once(timeout_sec=0)` -- a non-blocking pump. That keeps the
UI responsive (the timer callback never blocks) while still letting every
domain's rclpy context do its normal background work (discovery, parameter
services, etc.), and avoids any need for locks/queues between ROS and Qt.
"""

import json
import os
import sys
import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
from rcl_interfaces.msg import Log

from .drone_targets import load_targets

SPIN_TIMER_MS = 50
WAYPOINT_HEADERS = ["#", "x (m)", "y (m)", "z (m)", "yaw (deg)"]
FORMATION_HEADERS = ["Drone", "Target", "Setpoints (x, y, z) m"]
LOG_HEADERS = ["Time", "Drone", "Node", "Level", "Message"]
LOG_ROW_LIMIT = 1000  # oldest rows drop once the table hits this, newest at the bottom

# Dropdown options for the log panel's "Min level" filter -- entries below the
# selected level are dropped as they're drained, not just hidden, so lowering
# the filter later won't bring back anything already discarded.
LOG_LEVEL_OPTIONS = [
    ("Warning+ (default)", Log.WARN),
    ("Error+", Log.ERROR),
    ("All (incl. info/debug)", Log.DEBUG),
]

LOG_LEVEL_COLORS = {
    "WARN": QColor(255, 243, 176),
    "ERROR": QColor(255, 205, 205),
    "FATAL": QColor(255, 170, 170),
}

# Repo layout is .../ground_station/src/ground_station/ground_station/command_gui.py
# and .../ground_station/flight_formations/ -- this only resolves when running from
# a source checkout (not an installed package), so it's a starting point for the
# file dialog, not a hard dependency.
FORMATIONS_DIR = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "flight_formations")
)

PRESET_COMMANDS = [
    ("arm", "arm"),
    ("fly", "fly"),
    ("hover", "hover"),
    ("straight", "straight"),
    ("land", "land"),
]

# Vehicle mode buttons -- not tied to any flight formation file, just direct
# broadcasts on each domain's /mode topic.
PRESET_MODES = [
    ("manual", "manual"),
    ("position", "position"),
    ("offboard", "offboard"),
]


class MainWindow(QMainWindow):

    def __init__(self, targets):
        super().__init__()
        self.targets = targets
        self._waypoints = []  # list of (x, y, z, yaw_rad)
        self._formation = None  # parsed flight formation JSON, or None if none loaded

        self.setWindowTitle("Ground Station Command GUI")
        self._build_ui()

        # Non-blocking pump for every domain's executor -- see module docstring.
        self._spin_timer = QTimer(self)
        self._spin_timer.timeout.connect(self._poll_spin)
        self._spin_timer.start(SPIN_TIMER_MS)

    # ------------------------------------------------------------------ UI

    def _build_ui(self):
        central = QWidget()
        root = QVBoxLayout(central)

        root.addWidget(self._build_targets_box())
        root.addWidget(self._build_command_box())
        root.addWidget(self._build_setpoint_box())
        root.addWidget(self._build_formation_box())
        root.addWidget(self._build_error_log_box())

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        root.addWidget(QLabel("Log"))
        root.addWidget(self.log)

        self.setCentralWidget(central)
        self.resize(760, 980)

    def _build_targets_box(self):
        box = QGroupBox("Configured domains")
        layout = QVBoxLayout(box)
        if not self.targets:
            layout.addWidget(QLabel("(no domains configured)"))
        for target in self.targets:
            layout.addWidget(QLabel(f"  -> {target.describe()}"))
        return box

    def _build_command_box(self):
        box = QGroupBox("Broadcast command (all domains)")
        layout = QVBoxLayout(box)

        presets_row = QHBoxLayout()
        for label, text in PRESET_COMMANDS:
            btn = QPushButton(label)
            btn.clicked.connect(lambda _checked=False, t=text: self._broadcast(t))
            presets_row.addWidget(btn)
        layout.addLayout(presets_row)

        custom_row = QHBoxLayout()
        self.custom_command_edit = QLineEdit()
        self.custom_command_edit.setPlaceholderText("custom command string")
        self.custom_command_edit.returnPressed.connect(self._send_custom_command)
        send_custom_btn = QPushButton("Send custom")
        send_custom_btn.clicked.connect(self._send_custom_command)
        custom_row.addWidget(self.custom_command_edit)
        custom_row.addWidget(send_custom_btn)
        layout.addLayout(custom_row)

        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Mode:"))
        for label, text in PRESET_MODES:
            btn = QPushButton(label)
            btn.clicked.connect(lambda _checked=False, t=text: self._broadcast_mode(t))
            mode_row.addWidget(btn)
        layout.addLayout(mode_row)

        return box

    def _build_setpoint_box(self):
        box = QGroupBox("Per-drone setpoint dispatch")
        layout = QVBoxLayout(box)

        drone_row = QHBoxLayout()
        drone_row.addWidget(QLabel("Drone:"))
        self.drone_combo = QComboBox()
        for target in self.targets:
            self.drone_combo.addItem(target.label(), target)
        drone_row.addWidget(self.drone_combo, stretch=1)
        layout.addLayout(drone_row)

        fields_row = QHBoxLayout()
        self.x_spin = self._make_pose_spinbox(fields_row, "x (m)")
        self.y_spin = self._make_pose_spinbox(fields_row, "y (m)")
        self.z_spin = self._make_pose_spinbox(fields_row, "z (m)")
        self.yaw_spin = QDoubleSpinBox()
        self.yaw_spin.setRange(-180.0, 180.0)
        self.yaw_spin.setDecimals(1)
        self.yaw_spin.setSingleStep(5.0)
        yaw_col = QVBoxLayout()
        yaw_col.addWidget(QLabel("yaw (deg)"))
        yaw_col.addWidget(self.yaw_spin)
        fields_row.addLayout(yaw_col)
        layout.addLayout(fields_row)

        single_row = QHBoxLayout()
        send_single_btn = QPushButton("Send single setpoint")
        send_single_btn.clicked.connect(self._send_single_setpoint)
        single_row.addWidget(send_single_btn)
        single_row.addStretch(1)
        layout.addLayout(single_row)

        waypoint_btn_row = QHBoxLayout()
        add_btn = QPushButton("Add waypoint")
        add_btn.clicked.connect(self._add_waypoint)
        remove_btn = QPushButton("Remove selected")
        remove_btn.clicked.connect(self._remove_waypoint)
        up_btn = QPushButton("Move up")
        up_btn.clicked.connect(lambda: self._move_waypoint(-1))
        down_btn = QPushButton("Move down")
        down_btn.clicked.connect(lambda: self._move_waypoint(1))
        clear_btn = QPushButton("Clear path")
        clear_btn.clicked.connect(self._clear_path)
        for btn in (add_btn, remove_btn, up_btn, down_btn, clear_btn):
            waypoint_btn_row.addWidget(btn)
        layout.addLayout(waypoint_btn_row)

        self.waypoint_table = QTableWidget(0, len(WAYPOINT_HEADERS))
        self.waypoint_table.setHorizontalHeaderLabels(WAYPOINT_HEADERS)
        self.waypoint_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.waypoint_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.waypoint_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.waypoint_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        layout.addWidget(self.waypoint_table)

        send_path_btn = QPushButton("Send path (all waypoints, in order)")
        send_path_btn.clicked.connect(self._send_path)
        layout.addWidget(send_path_btn)

        return box

    def _build_formation_box(self):
        box = QGroupBox("Flight formation")
        layout = QVBoxLayout(box)

        load_row = QHBoxLayout()
        load_btn = QPushButton("Load formation file...")
        load_btn.clicked.connect(self._load_formation_file)
        load_row.addWidget(load_btn)
        self.formation_summary_label = QLabel("(no formation loaded)")
        load_row.addWidget(self.formation_summary_label, stretch=1)
        layout.addLayout(load_row)

        self.formation_table = QTableWidget(0, len(FORMATION_HEADERS))
        self.formation_table.setHorizontalHeaderLabels(FORMATION_HEADERS)
        self.formation_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.formation_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.formation_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        layout.addWidget(self.formation_table)

        send_row = QHBoxLayout()
        send_formation_btn = QPushButton("Send formation (all drones)")
        send_formation_btn.clicked.connect(self._send_formation)
        send_row.addWidget(send_formation_btn)
        send_row.addStretch(1)
        layout.addLayout(send_row)

        return box

    def _build_error_log_box(self):
        box = QGroupBox("Error / warning log (all domains)")
        layout = QVBoxLayout(box)

        filter_row = QHBoxLayout()
        filter_row.addWidget(QLabel("Min level:"))
        self.log_level_combo = QComboBox()
        for label, level in LOG_LEVEL_OPTIONS:
            self.log_level_combo.addItem(label, level)
        filter_row.addWidget(self.log_level_combo)
        filter_row.addStretch(1)
        clear_log_btn = QPushButton("Clear")
        clear_log_btn.clicked.connect(self._clear_error_log)
        filter_row.addWidget(clear_log_btn)
        layout.addLayout(filter_row)

        self.error_log_table = QTableWidget(0, len(LOG_HEADERS))
        self.error_log_table.setHorizontalHeaderLabels(LOG_HEADERS)
        header = self.error_log_table.horizontalHeader()
        for col in range(len(LOG_HEADERS) - 1):
            header.setSectionResizeMode(col, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(len(LOG_HEADERS) - 1, QHeaderView.Stretch)
        self.error_log_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.error_log_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        layout.addWidget(self.error_log_table)

        return box

    @staticmethod
    def _make_pose_spinbox(row_layout, label):
        spin = QDoubleSpinBox()
        spin.setRange(-1000.0, 1000.0)
        spin.setDecimals(3)
        spin.setSingleStep(0.1)
        col = QVBoxLayout()
        col.addWidget(QLabel(label))
        col.addWidget(spin)
        row_layout.addLayout(col)
        return spin

    # --------------------------------------------------------------- logic

    def _log(self, text):
        self.log.appendPlainText(text)

    def _broadcast(self, text):
        for target in self.targets:
            target.publish_command(text)
        self._log(f"sent {text!r} to {len(self.targets)} target(s)")

    def _send_custom_command(self):
        text = self.custom_command_edit.text().strip()
        if not text:
            self._log("(empty command, not sent)")
            return
        self._broadcast(text)
        self.custom_command_edit.clear()

    def _broadcast_mode(self, text):
        for target in self.targets:
            target.publish_mode(text)
        self._log(f"mode {text!r} -> {len(self.targets)} target(s)")

    def _selected_target(self):
        if not self.targets:
            QMessageBox.warning(self, "No domains", "No domains are configured.")
            return None
        return self.drone_combo.currentData()

    def _current_pose_fields(self):
        x = self.x_spin.value()
        y = self.y_spin.value()
        z = self.z_spin.value()
        yaw_rad = self.yaw_spin.value() * 3.141592653589793 / 180.0
        return x, y, z, yaw_rad

    def _send_single_setpoint(self):
        target = self._selected_target()
        if target is None:
            return
        x, y, z, yaw_rad = self._current_pose_fields()
        target.publish_setpoint(x, y, z, yaw_rad)
        self._log(
            f"setpoint -> {target.label()} {target.setpoint_topic}: "
            f"x={x:.3f} y={y:.3f} z={z:.3f} yaw={self.yaw_spin.value():.1f}deg"
        )

    def _add_waypoint(self):
        self._waypoints.append(self._current_pose_fields())
        self._refresh_waypoint_table()

    def _remove_waypoint(self):
        row = self.waypoint_table.currentRow()
        if row < 0:
            return
        del self._waypoints[row]
        self._refresh_waypoint_table()

    def _move_waypoint(self, delta):
        row = self.waypoint_table.currentRow()
        target_row = row + delta
        if row < 0 or not (0 <= target_row < len(self._waypoints)):
            return
        self._waypoints[row], self._waypoints[target_row] = (
            self._waypoints[target_row],
            self._waypoints[row],
        )
        self._refresh_waypoint_table()
        self.waypoint_table.selectRow(target_row)

    def _clear_path(self):
        self._waypoints.clear()
        self._refresh_waypoint_table()

    def _refresh_waypoint_table(self):
        self.waypoint_table.setRowCount(len(self._waypoints))
        for row, (x, y, z, yaw_rad) in enumerate(self._waypoints):
            yaw_deg = yaw_rad * 180.0 / 3.141592653589793
            values = [str(row + 1), f"{x:.3f}", f"{y:.3f}", f"{z:.3f}", f"{yaw_deg:.1f}"]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)
                self.waypoint_table.setItem(row, col, item)

    def _send_path(self):
        target = self._selected_target()
        if target is None:
            return
        if not self._waypoints:
            QMessageBox.warning(self, "Empty path", "Add at least one waypoint first.")
            return
        target.publish_path(self._waypoints)
        self._log(
            f"path -> {target.label()} {target.path_topic}: "
            f"{len(self._waypoints)} waypoint(s)"
        )

    # -------------------------------------------------------- flight formation

    def _target_for_drone_id(self, drone_id):
        """Match a flight-formation file's drone_id (1-5) to a configured DomainTarget.

        Tried in order: ROS_DOMAIN_ID equal to drone_id (matches config/drones.yaml's
        index-aligned drone_domains), namespace "px4_<id>" or "drone<id>" (matches
        drones.yaml / drone_profiles naming), then falling back to position in the
        targets list (drone_id N -> targets[N-1]) for a bare/default configuration.
        """
        for target in self.targets:
            if target.domain_id == drone_id:
                return target
        for target in self.targets:
            if target.namespace in (f"px4_{drone_id}", f"drone{drone_id}"):
                return target
        if 1 <= drone_id <= len(self.targets):
            return self.targets[drone_id - 1]
        return None

    def _load_formation_file(self):
        start_dir = FORMATIONS_DIR if os.path.isdir(FORMATIONS_DIR) else ""
        path, _ = QFileDialog.getOpenFileName(
            self, "Load flight formation file", start_dir, "Flight formation (*.json)")
        if not path:
            return
        try:
            with open(path) as f:
                formation = json.load(f)
            self._validate_formation(formation)
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, "Invalid formation file", f"{path}:\n{exc}")
            return

        self._formation = formation
        self._refresh_formation_table()
        speed = formation.get("flight_speed_mps")
        speed_text = f" @ {speed:g} m/s" if formation["flight_type"] == "traverse" else ""
        delay = formation.get("delay_per_setpoint")
        delay_text = f", {delay:g}s/setpoint" if delay is not None else ""
        self.formation_summary_label.setText(
            f"run {formation.get('run_number')}: {formation['flight_type']}{speed_text}, "
            f"{formation['num_drones']} drone(s), {formation.get('coordinate_system')} frame"
            f"{delay_text} ({os.path.basename(path)})"
        )
        self._log(f"formation: loaded {path}")

    @staticmethod
    def _validate_formation(formation):
        """Minimal shape check -- see flight_formations/README.md for the full spec."""
        flight_type = formation.get("flight_type")
        if flight_type not in ("hover", "traverse"):
            raise ValueError('flight_type must be "hover" or "traverse"')
        delay = formation.get("delay_per_setpoint")
        if delay is not None and not isinstance(delay, (int, float)):
            raise ValueError("delay_per_setpoint must be a number or null")
        drones = formation.get("drones")
        if not isinstance(drones, list) or not drones:
            raise ValueError("drones must be a non-empty array")
        if formation.get("num_drones") != len(drones):
            raise ValueError(
                f"num_drones ({formation.get('num_drones')}) does not match "
                f"len(drones) ({len(drones)})")
        for drone in drones:
            for key in ("drone_id", "setpoints_m"):
                if key not in drone:
                    raise ValueError(f"drone entry missing {key!r}: {drone}")
            setpoints = drone["setpoints_m"]
            if not isinstance(setpoints, list) or not setpoints:
                raise ValueError(
                    f"drone {drone.get('drone_id')} setpoints_m must be a non-empty array")
            if flight_type == "traverse" and len(setpoints) < 2:
                raise ValueError(
                    f"drone {drone.get('drone_id')} setpoints_m needs at least 2 points "
                    f"for a traverse run")
            for point in setpoints:
                if any(point.get(axis) is None for axis in ("x", "y", "z")):
                    raise ValueError(
                        f"drone {drone.get('drone_id')} setpoints_m has a point with a null axis")

    @staticmethod
    def _format_setpoints(setpoints):
        return " -> ".join(f"({p['x']:.3f}, {p['y']:.3f}, {p['z']:.3f})" for p in setpoints)

    def _refresh_formation_table(self):
        drones = self._formation["drones"] if self._formation else []
        self.formation_table.setRowCount(len(drones))
        for row, drone in enumerate(drones):
            drone_id = drone["drone_id"]
            target = self._target_for_drone_id(drone_id)
            target_text = target.label() if target else "(unmatched)"
            values = [
                str(drone_id),
                target_text,
                self._format_setpoints(drone["setpoints_m"]),
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)
                self.formation_table.setItem(row, col, item)

    def _send_formation(self):
        if self._formation is None:
            QMessageBox.warning(self, "No formation loaded", "Load a flight formation file first.")
            return
        formation = self._formation
        flight_type = formation["flight_type"]
        speed = formation.get("flight_speed_mps")
        delay = formation.get("delay_per_setpoint")
        sent = 0
        for drone in formation["drones"]:
            drone_id = drone["drone_id"]
            target = self._target_for_drone_id(drone_id)
            if target is None:
                self._log(f"formation: no configured domain for drone_id {drone_id}, skipping")
                continue
            setpoints = drone["setpoints_m"]
            # Every setpoints_m entry -- one or many -- is always mirrored onto
            # /path in order, so /path is never left empty or truncated to a
            # single pose regardless of flight_type.
            waypoints = [(p["x"], p["y"], p["z"], 0.0) for p in setpoints]
            target.publish_path(waypoints)
            # flight_type and delay_per_setpoint come straight from the loaded
            # formation file (see flight_formations/README.md) -- there is no
            # separate UI control for either.
            target.publish_flight_type(flight_type)
            if delay is not None:
                target.publish_delay_per_setpoint(delay)
            if flight_type == "hover":
                point = setpoints[0]
                target.publish_setpoint(point["x"], point["y"], point["z"])
                self._log(
                    f"formation hover -> {target.label()}: "
                    f"x={point['x']:.3f} y={point['y']:.3f} z={point['z']:.3f}"
                )
            else:  # traverse
                if speed is not None:
                    target.publish_command(f"speed {speed}")
                self._log(
                    f"formation traverse -> {target.label()}: "
                    f"{self._format_setpoints(setpoints)} @ {speed} m/s"
                )
            sent += 1
        self._log(
            f"formation run {formation.get('run_number')}: dispatched "
            f"{sent}/{len(formation['drones'])} drone(s)"
        )

    # -------------------------------------------------------------- error log

    def _clear_error_log(self):
        self.error_log_table.setRowCount(0)

    def _consume_target_logs(self, target):
        min_level = self.log_level_combo.currentData()
        for entry in target.drain_new_logs():
            if entry["level"] >= min_level:
                self._append_log_row(entry)

    def _append_log_row(self, entry):
        table = self.error_log_table
        if table.rowCount() >= LOG_ROW_LIMIT:
            table.removeRow(0)
        row = table.rowCount()
        table.insertRow(row)

        stamp = entry["stamp_sec"]
        time_text = time.strftime("%H:%M:%S", time.localtime(stamp)) if stamp else ""
        values = [
            time_text,
            entry["target_label"],
            entry["node"],
            entry["level_name"],
            entry["text"],
        ]
        color = LOG_LEVEL_COLORS.get(entry["level_name"])
        for col, value in enumerate(values):
            item = QTableWidgetItem(value)
            if color is not None:
                item.setBackground(color)
            table.setItem(row, col, item)
        table.scrollToBottom()

    # ------------------------------------------------------------- ROS pump

    def _poll_spin(self):
        for target in self.targets:
            target.spin_once(timeout_sec=0.0)
            self._consume_target_logs(target)

    def closeEvent(self, event):
        self._spin_timer.stop()
        for target in self.targets:
            target.close()
        super().closeEvent(event)


def main(args=None):
    targets = load_targets(args)
    app = QApplication(sys.argv[:1])
    window = MainWindow(targets)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

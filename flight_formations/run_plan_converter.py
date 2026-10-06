#!/usr/bin/env python3
"""HALO run plan (CSV export) -> flight formation JSON converter.

Standalone tool -- does not import or depend on the ground station package.
Reads the "Run Plan" CSV exported from 260828-HALO-Wind_Tunnel_Run_Plan.xlsx
and writes one flight formation file per run, in the format described in
README.md (same folder).

Usage:
    python3 run_plan_converter.py                 # open the GUI
    python3 run_plan_converter.py --run 27        # convert one run, no GUI
    python3 run_plan_converter.py --run 7 --run 9 # several runs
    python3 run_plan_converter.py --all           # convert every run

The mapping from the plan's location/altitude codes to tunnel coordinates is
explained in CONVERTER_NOTES.txt. It is defined once in DEFAULT_GRID below and
can be overridden from the GUI's "Grid" fields.
"""

import argparse
import csv
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CSV = os.path.join(HERE, "260828-HALO-Wind_Tunnel_Run_Plan.xlsx - Run Plan.csv")
DEFAULT_OUT_DIR = os.path.join(HERE, "generated")

SCHEMA_VERSION = "2.0"
MAX_DRONES = 5
# Seconds to hold at each setpoints_m entry (top-level delay_per_setpoint).
DEFAULT_DELAY_PER_SETPOINT = 1

# The plan's "UAV 1".."UAV 5" columns -> the drone's ROS_DOMAIN_ID, which is
# what the output's drone_id holds (the ground station matches drone_id to a
# domain first).
UAV_TO_DOMAIN_ID = {1: 3, 2: 4, 3: 5, 4: 6, 5: 7}

# All values in millimetres, in the frame of HALO-MTU-Flow-Sampling-Grid.pdf
# (X along the flow, Y across it). The output is converted to the "tunnel"
# frame from README.md: metres, origin at the turntable centre.
DEFAULT_GRID = {
    # Location code "(i j)": i picks the X column, j picks the Y row.
    # These are the 3x3 pressure-tap / taped-grid lines from the PDF.
    "x_by_index_mm": [3600, 4600, 5600],
    "y_by_index_mm": [-1000, 0, 1000],
    # Altitude code 1/2/3 -> z (PDF: z = 1428, 1928, 2428 mm).
    "z_by_index_mm": [1428, 1928, 2428],
    # Turntable centre in the PDF frame -- becomes the tunnel-frame origin.
    "origin_x_mm": 5600,
    "origin_y_mm": 0,
    # Altitude used for Y sweeps whose altitude column holds the sweep
    # itself (run 9): the plate centre, per the PDF.
    "y_sweep_default_z_mm": 1928,
}

# CSV column indexes (row 4 of the sheet is the header).
COL_RUN, COL_NAME, COL_SEQ, COL_FLOW, COL_PLATE, COL_ATTR, COL_DUR = range(7)
COL_TONE, COL_SOURCE, COL_HT, COL_TSPEED, COL_NUM_UAV = 7, 8, 9, 10, 11
COL_UAV1 = 12          # UAV n: active / location / altitude at 12 + 3*(n-1)
COL_NOTES = 27
HEADER_ROWS = 4

LOCATION_RE = re.compile(r"^\(\s*(\d+)\s+(\d+)\s*\)$")


def _clean(cell):
    """Strip whitespace and the stray double quotes the xlsx export leaves."""
    return (cell or "").strip().strip('"').strip()


def _number(cell):
    cell = _clean(cell)
    if not cell:
        return None
    value = float(cell)
    return int(value) if value.is_integer() else value


def _mm_to_m(value):
    return round(value / 1000.0, 4)


# ------------------------------------------------------------------ parsing

def read_run_plan(csv_path):
    """Return {run_number: {"header": row, "rows": [rows...]}} in file order.

    A run starts at a row with a value in the "Run #" column and continues
    over the following rows with that column blank (one row per test
    condition). Run-number rows with no data at all (the empty 21..50 block
    at the bottom of the sheet) are skipped.
    """
    with open(csv_path, newline="", encoding="utf-8-sig") as f:
        rows = [r + [""] * (COL_NOTES + 1 - len(r)) for r in csv.reader(f)]

    runs = {}
    current = None
    for row in rows[HEADER_ROWS:]:
        if not any(_clean(c) for c in row):
            continue
        run_cell = _clean(row[COL_RUN])
        if run_cell:
            if not any(_clean(c) for c in row[COL_RUN + 1:]):
                current = None  # placeholder row with only a run number
                continue
            run_number = int(float(run_cell))
            if run_number in runs:
                raise ValueError(f"run {run_number} appears twice in {csv_path}")
            current = {"header": row, "rows": [row]}
            runs[run_number] = current
        elif current is not None:
            current["rows"].append(row)
    return runs


def _uav_cells(row, n):
    base = COL_UAV1 + 3 * (n - 1)
    return _clean(row[base]), _clean(row[base + 1]), _clean(row[base + 2])


def _parse_location(code):
    match = LOCATION_RE.match(code)
    if not match:
        return None
    return int(match.group(1)), int(match.group(2))


def _grid_lookup(values, index, what):
    if not 1 <= index <= len(values):
        raise ValueError(f"{what} index {index} outside 1..{len(values)}")
    return values[index - 1]


def _sweep_path(location, altitude, grid, warnings, drone_id):
    """Ordered list of (x_mm, y_mm, z_mm) for one drone's sweep pass.

    location / altitude are the cleaned CSV strings for one condition row.
    Returns (points, sweep_label) where sweep_label is None for a hover.
    """
    xs, ys, zs = grid["x_by_index_mm"], grid["y_by_index_mm"], grid["z_by_index_mm"]
    loc = _parse_location(location)

    # Y sweep encoded in the location column, fixed altitude ("+Y-Sweep-2").
    y_loc = re.match(r"^([+-])Y-Sweep(?:-(\d+))?$", location, re.I)
    if y_loc:
        sign = y_loc.group(1)
        z = _grid_lookup(zs, int(altitude), "altitude") if altitude.isdigit() else \
            grid["y_sweep_default_z_mm"]
        # The run names (e.g. 1Max-Ty-Pt1-...) say the sweep starts at Pt1 =
        # (1 1); it runs along that X column across the full grid width.
        x = xs[0]
        ends = (ys[0], ys[-1]) if sign == "+" else (ys[-1], ys[0])
        return [(x, ends[0], z), (x, ends[1], z)], f"{sign}Y-Sweep"

    if loc is None:
        raise ValueError(f"drone {drone_id}: unrecognised location {location!r}")
    x = _grid_lookup(xs, loc[0], "location x")
    y = _grid_lookup(ys, loc[1], "location y")

    if altitude.isdigit():
        return [(x, y, _grid_lookup(zs, int(altitude), "altitude"))], None

    z_sweep = re.match(r"^([+-])Z-Sweep$", altitude, re.I)
    if z_sweep:
        sign = z_sweep.group(1)
        lo, hi = min(zs), max(zs)
        ends = (lo, hi) if sign == "+" else (hi, lo)
        return [(x, y, ends[0]), (x, y, ends[1])], f"{sign}Z-Sweep"

    y_sweep = re.match(r"^([+-])Y-Sweep$", altitude, re.I)
    if y_sweep:
        sign = y_sweep.group(1)
        z = grid["y_sweep_default_z_mm"]
        ends = (min(ys), max(ys)) if sign == "+" else (max(ys), min(ys))
        warnings.add(
            f"drone {drone_id}: Y sweep has no altitude in the plan; "
            f"using z = {z} mm (plate centre)")
        return [(x, ends[0], z), (x, ends[1], z)], f"{sign}Y-Sweep"

    raise ValueError(f"drone {drone_id}: unrecognised altitude {altitude!r}")


def _to_tunnel(point_mm, grid):
    x, y, z = point_mm
    return {
        "x": _mm_to_m(x - grid["origin_x_mm"]),
        "y": _mm_to_m(y - grid["origin_y_mm"]),
        "z": _mm_to_m(z),
    }


def convert_run(run_number, run, grid=None):
    """Build the flight formation dict for one run. Returns (formation, warnings)."""
    grid = grid or DEFAULT_GRID
    header, rows = run["header"], run["rows"]
    warnings = set()

    active_ids = [n for n in range(1, MAX_DRONES + 1)
                  if _uav_cells(header, n)[0].lower() == "yes"]
    if not active_ids:
        raise ValueError(f"run {run_number}: no active UAVs")

    declared = _number(header[COL_NUM_UAV])
    if declared is not None and declared != len(active_ids):
        warnings.add(
            f"'Number of UAVs' says {declared} but {len(active_ids)} UAV(s) are marked active")

    # Per drone: one entry per condition row, in plan order, so the setpoint
    # list lines up with meta_data.test_conditions. A hover row adds its one
    # point (repeated rows repeat the point); a sweep row adds its start and
    # end, dropping the start when it equals the previous row's end.
    drones = []
    any_sweep = False
    per_row_sweeps = [[] for _ in rows]
    for uav in active_ids:
        drone_id = UAV_TO_DOMAIN_ID[uav]
        path, hover_points = [], set()
        for i, row in enumerate(rows):
            active, location, altitude = _uav_cells(row, uav)
            if active.lower() != "yes":
                continue
            points, label = _sweep_path(location, altitude, grid, warnings, drone_id)
            per_row_sweeps[i].append(label)
            if label is None:
                hover_points.add(points[0])
                path.extend(points)
            else:
                any_sweep = True
                path.extend(points[1:] if path and path[-1] == points[0] else points)
        _, location, altitude = _uav_cells(header, uav)
        drones.append({
            "drone_id": drone_id,
            "setpoints_m": [_to_tunnel(p, grid) for p in path],
            "location_code": location,
            "altitude_code": int(altitude) if altitude.isdigit() else altitude,
        })
        if len(hover_points) > 1:
            warnings.add(f"drone {drone_id} (UAV {uav}): hover position changes between condition rows")

    hover_or_traverse = _clean(header[COL_HT]).upper() or None
    flight_type = "traverse" if any_sweep else "hover"
    if hover_or_traverse and hover_or_traverse[0] != flight_type[0].upper():
        warnings.add(
            f"plan column says '{hover_or_traverse}' but the UAV paths are a "
            f"{flight_type}; using '{flight_type}'")

    speed = _number(header[COL_TSPEED])
    if flight_type == "traverse" and speed is None:
        warnings.add("traverse run has no traverse speed in the plan; flight_speed_mps left null")

    name = _clean(header[COL_NAME]) or None
    if name is None:
        warnings.add("run has no name in the plan")

    test_conditions = []
    for row, sweeps in zip(rows, per_row_sweeps):
        flow = _number(row[COL_FLOW])
        cond = {
            # The plan's "Flow Speed (k/hr)" column, as written.
            "flow_speed_kph": float(flow) if flow is not None else None,
            "sound_source_tone": _clean(row[COL_TONE]) or None,
            "active_point_source_id": _number(row[COL_SOURCE]),
        }
        labels = sorted({s for s in sweeps if s})
        if labels:
            cond["sweep"] = labels[0] if len(labels) == 1 else labels
        test_conditions.append(cond)

    notes_cells = [_clean(r[COL_NOTES]) for r in rows] + \
        [_clean(r[COL_NAME]) for r in rows[1:]]  # free text left in the name column
    notes = "; ".join(n for n in notes_cells if n) or None

    plate = _clean(header[COL_PLATE]).lower() == "yes"
    plate_text = "installed" if plate else "removed"
    flows = sorted({c["flow_speed_kph"] for c in test_conditions
                    if c["flow_speed_kph"] is not None})
    if len(flows) > 1:
        flow_conditions = f"Flow speed swept with turbulence plate {plate_text}"
    elif flows:
        flow_conditions = f"Flow speed {flows[0]:g} k/hr with turbulence plate {plate_text}"
    else:
        flow_conditions = f"Turbulence plate {plate_text}"

    duration = _number(header[COL_DUR])
    sequence = _number(header[COL_SEQ])

    formation = {
        "schema_version": SCHEMA_VERSION,
        "run_number": run_number,
        "coordinate_system": "tunnel",
        "flight_type": flight_type,
        "delay_per_setpoint": DEFAULT_DELAY_PER_SETPOINT,
        "num_drones": len(drones),
        "flight_speed_mps": speed if flight_type == "traverse" else None,
        "drones": drones,
        "meta_data": {
            "run_number": run_number,
            "num_drones": len(drones),
            "run_name": name,
            "halo_sequence": sequence,
            "hover_or_traverse": hover_or_traverse,
            "turbulence_plate_installed": plate,
            "halo_test_attributes": _clean(header[COL_ATTR]) or None,
            "duration_s": float(duration) if duration is not None else None,
            "flow_speed_mps": None,
            "flow_conditions": flow_conditions,
            "tunnel_test_sequence": str(float(sequence)) if sequence is not None else None,
            "test_conditions": test_conditions,
            "notes": notes,
        },
    }
    return formation, sorted(warnings)


def output_path(out_dir, run_number):
    return os.path.join(out_dir, f"R{run_number:03d}.json")


def write_formation(formation, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    path = output_path(out_dir, formation["run_number"])
    with open(path, "w") as f:
        json.dump(formation, f, indent=2)
        f.write("\n")
    return path


def convert_many(runs, run_numbers, out_dir, grid=None):
    """Convert and write the given runs. Returns a list of (run, path|None, messages)."""
    results = []
    for n in run_numbers:
        try:
            formation, warnings = convert_run(n, runs[n], grid)
            results.append((n, write_formation(formation, out_dir), warnings))
        except (ValueError, KeyError) as exc:
            results.append((n, None, [f"ERROR: {exc}"]))
    return results


def run_summary(run_number, run):
    header = run["header"]
    active = [n for n in range(1, MAX_DRONES + 1) if _uav_cells(header, n)[0].lower() == "yes"]
    return {
        "run": run_number,
        "name": _clean(header[COL_NAME]) or "(unnamed)",
        "type": _clean(header[COL_HT]) or "?",
        "drones": len(active),
        "plate": "TP" if _clean(header[COL_PLATE]).lower() == "yes" else "-",
        "rows": len(run["rows"]),
    }


# ---------------------------------------------------------------------- GUI

def launch_gui(csv_path, out_dir):
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk

    root = tk.Tk()
    root.title("HALO Run Plan -> Flight Formation")
    root.geometry("1100x680")

    state = {"runs": {}}
    csv_var = tk.StringVar(value=csv_path)
    out_var = tk.StringVar(value=out_dir)
    grid_vars = {}

    # --- file pickers
    files = ttk.Frame(root, padding=8)
    files.pack(fill="x")
    for r, (label, var, pick) in enumerate([
        ("Run plan CSV", csv_var, lambda: _pick_csv()),
        ("Output folder", out_var, lambda: _pick_out()),
    ]):
        ttk.Label(files, text=label).grid(row=r, column=0, sticky="w", padx=(0, 6))
        ttk.Entry(files, textvariable=var).grid(row=r, column=1, sticky="ew", pady=2)
        ttk.Button(files, text="Browse...", command=pick).grid(row=r, column=2, padx=4)
    files.columnconfigure(1, weight=1)

    # --- grid settings
    grid_box = ttk.LabelFrame(root, text="Grid (mm, PDF frame) -- see CONVERTER_NOTES.txt",
                              padding=8)
    grid_box.pack(fill="x", padx=8)
    grid_fields = [
        ("x_by_index_mm", "Location 1st index -> X"),
        ("y_by_index_mm", "Location 2nd index -> Y"),
        ("z_by_index_mm", "Altitude 1/2/3 -> Z"),
        ("origin_x_mm", "Turntable X"),
        ("origin_y_mm", "Turntable Y"),
        ("y_sweep_default_z_mm", "Y-sweep Z"),
    ]
    for i, (key, label) in enumerate(grid_fields):
        value = DEFAULT_GRID[key]
        text = ", ".join(str(v) for v in value) if isinstance(value, list) else str(value)
        grid_vars[key] = tk.StringVar(value=text)
        ttk.Label(grid_box, text=label).grid(row=i // 3, column=(i % 3) * 2, sticky="e", padx=4)
        ttk.Entry(grid_box, textvariable=grid_vars[key], width=20).grid(
            row=i // 3, column=(i % 3) * 2 + 1, sticky="w", pady=2)

    def current_grid():
        grid = {}
        for key, var in grid_vars.items():
            parts = [p for p in re.split(r"[,\s]+", var.get().strip()) if p]
            nums = [float(p) for p in parts]
            if isinstance(DEFAULT_GRID[key], list):
                if len(nums) < 1:
                    raise ValueError(f"{key} needs at least one value")
                grid[key] = nums
            else:
                if len(nums) != 1:
                    raise ValueError(f"{key} needs exactly one value")
                grid[key] = nums[0]
        return grid

    # --- run list + preview
    body = ttk.PanedWindow(root, orient="horizontal")
    body.pack(fill="both", expand=True, padx=8, pady=8)

    left = ttk.Frame(body)
    columns = ("run", "name", "type", "drones", "plate", "rows")
    tree = ttk.Treeview(left, columns=columns, show="headings", selectmode="extended")
    for col, width in zip(columns, (50, 200, 50, 60, 50, 50)):
        tree.heading(col, text=col.capitalize())
        tree.column(col, width=width, anchor="w" if col == "name" else "center")
    scroll = ttk.Scrollbar(left, orient="vertical", command=tree.yview)
    tree.configure(yscrollcommand=scroll.set)
    tree.pack(side="left", fill="both", expand=True)
    scroll.pack(side="left", fill="y")
    body.add(left, weight=1)

    right = ttk.Frame(body)
    preview = tk.Text(right, wrap="none", font=("TkFixedFont", 9))
    pscroll = ttk.Scrollbar(right, orient="vertical", command=preview.yview)
    preview.configure(yscrollcommand=pscroll.set)
    preview.pack(side="left", fill="both", expand=True)
    pscroll.pack(side="left", fill="y")
    body.add(right, weight=2)

    # --- actions
    actions = ttk.Frame(root, padding=(8, 0, 8, 8))
    actions.pack(fill="x")
    status = tk.StringVar(value="")
    ttk.Label(actions, text="Run #").pack(side="left")
    run_entry = ttk.Entry(actions, width=8)
    run_entry.pack(side="left", padx=4)
    ttk.Button(actions, text="Convert run #", command=lambda: convert_entry()).pack(side="left")
    ttk.Button(actions, text="Convert selected",
               command=lambda: convert_selected()).pack(side="left", padx=6)
    ttk.Button(actions, text="Convert ALL", command=lambda: convert_all()).pack(side="left")
    ttk.Button(actions, text="Reload CSV", command=lambda: load_csv()).pack(side="left", padx=6)
    ttk.Label(actions, textvariable=status).pack(side="left", padx=10)

    def _pick_csv():
        path = filedialog.askopenfilename(
            initialdir=os.path.dirname(csv_var.get()) or HERE,
            filetypes=[("CSV", "*.csv"), ("All files", "*")])
        if path:
            csv_var.set(path)
            load_csv()

    def _pick_out():
        path = filedialog.askdirectory(initialdir=out_var.get() or HERE)
        if path:
            out_var.set(path)

    def load_csv():
        tree.delete(*tree.get_children())
        try:
            state["runs"] = read_run_plan(csv_var.get())
        except (OSError, ValueError) as exc:
            state["runs"] = {}
            messagebox.showerror("Could not read run plan", str(exc))
            return
        for n, run in state["runs"].items():
            s = run_summary(n, run)
            tree.insert("", "end", iid=str(n), values=[s[c] for c in columns])
        status.set(f"{len(state['runs'])} runs loaded")

    def show_preview(_event=None):
        sel = tree.selection()
        preview.delete("1.0", "end")
        if not sel:
            return
        n = int(sel[0])
        run_entry.delete(0, "end")
        run_entry.insert(0, str(n))
        try:
            formation, warnings = convert_run(n, state["runs"][n], current_grid())
            text = json.dumps(formation, indent=2)
            if warnings:
                text = "WARNINGS:\n  " + "\n  ".join(warnings) + "\n\n" + text
        except ValueError as exc:
            text = f"ERROR: {exc}"
        preview.insert("1.0", text)

    def do_convert(numbers):
        if not numbers:
            messagebox.showinfo("Nothing to convert", "Select or enter at least one run.")
            return
        try:
            grid = current_grid()
        except ValueError as exc:
            messagebox.showerror("Bad grid setting", str(exc))
            return
        results = convert_many(state["runs"], numbers, out_var.get(), grid)
        ok = [r for r in results if r[1]]
        lines = [f"Wrote {len(ok)}/{len(results)} file(s) to {out_var.get()}", ""]
        for n, path, msgs in results:
            lines.append(f"Run {n}: " + (os.path.basename(path) if path else "FAILED"))
            lines.extend(f"    {m}" for m in msgs)
        status.set(lines[0])
        preview.delete("1.0", "end")
        preview.insert("1.0", "\n".join(lines))

    def convert_entry():
        text = run_entry.get().strip()
        if not text.isdigit() or int(text) not in state["runs"]:
            messagebox.showerror("Unknown run", f"Run {text!r} is not in the run plan.")
            return
        do_convert([int(text)])

    def convert_selected():
        do_convert([int(i) for i in tree.selection()])

    def convert_all():
        if messagebox.askyesno("Convert all",
                               f"Convert all {len(state['runs'])} runs into {out_var.get()}?\n"
                               "Existing files with the same names are overwritten."):
            do_convert(list(state["runs"]))

    tree.bind("<<TreeviewSelect>>", show_preview)
    load_csv()
    root.mainloop()


# ---------------------------------------------------------------------- CLI

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--csv", default=DEFAULT_CSV, help="run plan CSV")
    parser.add_argument("--out", default=DEFAULT_OUT_DIR, help="output folder")
    parser.add_argument("--run", type=int, action="append", help="run number (repeatable)")
    parser.add_argument("--all", action="store_true", help="convert every run")
    args = parser.parse_args()

    if not args.run and not args.all:
        launch_gui(args.csv, args.out)
        return 0

    runs = read_run_plan(args.csv)
    numbers = list(runs) if args.all else args.run
    missing = [n for n in numbers if n not in runs]
    if missing:
        print(f"Run(s) not in the plan: {missing}", file=sys.stderr)
        return 1
    failed = 0
    for n, path, msgs in convert_many(runs, numbers, args.out):
        print(f"run {n}: {path or 'FAILED'}")
        for m in msgs:
            print(f"    {m}")
        failed += path is None
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

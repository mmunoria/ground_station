# Flight Formation File Format

This folder defines the JSON format used to hand a drone formation flight plan
to the ground station. It follows the professors' spec (drone position,
flight type, per-run metadata) but stores it as structured JSON instead of
free-form text so it can be validated and parsed programmatically.

## Files

- `flight_formation_template.json` — blank template with every field
  present and set to `null`/a default, meant to be copied and filled in for
  a new run.
- `flight_formation_example_run5.json` — the "Run 5" example from the
  professors' instructions, filled in, to show the format in use.

## Field reference

| Field | Type | Notes |
|---|---|---|
| `schema_version` | string | Bump if the format changes later, so old files stay readable. |
| `run_number` | int | Primary identifier for the flight, matched to the test plan/runlog. |
| `coordinate_system` | string | Which frame the positions below are given in. Defaults to `"tunnel"` (see below), but is a labeled field rather than hard-coded so a different frame can be used later without changing the file format. |
| `flight_type` | `"hover"` \| `"traverse"` | Hover holds position at `setpoints_m[0]`; traverse flies through `setpoints_m` in order at `flight_speed_mps`. |
| `num_drones` | int | 1–5, number of drones participating in this run. |
| `flight_speed_mps` | number \| null | Only used when `flight_type` is `"traverse"`. Ignored for `"hover"` (kept in the file, per the professors' example, rather than omitted — see below). |
| `delay_per_setpoint` | number \| null | Seconds to hold at each `setpoints_m` entry before advancing to the next one. Optional — omit or leave `null` for no delay. |
| `drones` | array | One entry per drone, length must equal `num_drones`. |
| `drones[].drone_id` | int | 1–5. |
| `drones[].setpoints_m` | array of `{x, y, z}` | Ordered list of positions in meters, in `coordinate_system`, flown in sequence. For `"hover"`, exactly one entry. For `"traverse"`, at least two (first is the start, last is the end; extra entries are intermediate waypoints). |
| `meta_data` | object | Logging/test-plan info, meant to eventually be auto-populated from the runlog (HALO sequences). |
| `meta_data.run_number` | int | Cross-check copy of the top-level `run_number` (see below). |
| `meta_data.num_drones` | int | Cross-check copy of the top-level `num_drones`. |
| `meta_data.flow_speed_mps` | number \| null | Tunnel flow speed for this run. |
| `meta_data.flow_conditions` | string \| null | Free-text description of flow conditions beyond just speed, if needed. |
| `meta_data.tunnel_test_sequence` | string \| null | Test sequence identifier from the HALO runlog. |

### The "tunnel" coordinate system

`coordinate_system: "tunnel"` means: distances in meters, measured from the
center of the turntable, with

- **+x** pointing downstream, in line with the flow
- **+y** pointing toward the wall away from the control room
- **+z** pointing up

This is the frame definition itself and doesn't need to be repeated inside
every flight file — it only needs to be declared by name (`"tunnel"`), which
is what the `coordinate_system` field is for. If a different reference frame
is ever needed (e.g. a drone-relative or world/lab frame), it gets a new name
here and its own definition added to this README, without changing the file
structure.

## Design choices

**Why JSON.** Machine-parseable, human-readable, no extra dependencies to
read/write it, and it matches what the professors asked for. It also
supports nesting, which the flat "Drone 1 start position = (...)"-style
example doesn't need to be flattened for.

**Why `drones` is an array of objects, not flat keys like
`drone_1_start_position`.** The spec says a run can have 1 to 5 drones. An
array scales naturally to any of those counts and lets the ground station
loop over `drones` instead of needing to know in advance how many
`drone_N_...` keys to look for. Each drone's setpoints stay together in one
object instead of being split across separately-indexed groups.

**Why `setpoints_m` is a single ordered array instead of separate
`start_position_m`/`end_position_m` fields.** A plain start/end pair can't
express an intermediate waypoint without changing the file's shape. A list
handles hover (one entry) and traverse (two or more entries, flown in order)
with the same field and the same parsing code — no conditional schema, and
room to add waypoints to a traverse run later without another format change.

**Why there's no explicit duration field.** Per the instructions: hover
holds until externally prompted to move (no fixed duration to record), and
traverse duration is derived by the flight controller from `flight_speed_mps`
and the distance between consecutive `setpoints_m` entries, so it doesn't
need to be duplicated as a separate field in the file.

**Why `run_number` and `num_drones` appear both at the top level and again
inside `meta_data`.** The top-level values are what the flight controller
actually reads to fly the mission. The `meta_data` copies are the
"as-logged" values, meant to eventually be pulled automatically from the
HALO runlog/test plan. Keeping both allows a later validation step to flag a
mismatch (e.g. the file's flight plan says 5 drones but the runlog says 4)
instead of silently trusting one source.

**Why `coordinate_system` is a string field instead of assuming tunnel
coordinates everywhere.** The instructions specifically call out that "we
need to be able to define which coordinate system this information is
provided [in]," even though tunnel coordinates will be the common case since
they're known ahead of time. A named field keeps that flexibility without
adding complexity to the common case (it just defaults to `"tunnel"`).

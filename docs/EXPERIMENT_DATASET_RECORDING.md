# Local dual-stream experiment recording

`scripts/record_experiment_dataset.py` records ROS raw force at its publisher
rate and UR realtime frames (default 30013), without tare, robot commands,
automatic Git operations or uploading. The optional wrapped command is an
explicit user-selected experiment and MAY move the robot: its own safety
checks and on-site supervision remain required.

```bash
cd ~/UR10
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash

# Record only, no experiment launched; Ctrl+C saves an interrupted dataset.
PYTHONNOUSERSITE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
python3 scripts/record_experiment_dataset.py --name manual-test --duration 300

# Long-running watcher: idle 30 s pauses raw recording; movement resumes.
PYTHONNOUSERSITE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
python3 scripts/record_experiment_dataset.py --name motion-watch --watch

# Static relaxation/drift tests need continuous raw recording.
PYTHONNOUSERSITE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
python3 scripts/record_experiment_dataset.py --name static-drift --continuous --duration 300

# Example: choose to record AND run an independently approved experiment.
# --run MUST be last; all subsequent arguments belong to the experiment.
PYTHONNOUSERSITE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
python3 scripts/record_experiment_dataset.py --name gravity-repeat \
--run /usr/bin/python3 /home/jx/.dsh-analysis/repeat_local_gravity_20261021.py \
--execute --site-confirmed --repeats 3
```

Default output: `outputs/experiment_datasets/<timestamp>-<name>/`:

- `force.csv.gz`: receipt wall/monotonic times, ROS source time/frame_id, six
  unmodified force/torque values in N and Nm.
- `ur.csv.gz`: receipt wall/monotonic times, controller uptime, joints (rad),
  active TCP pose (m, rotvec rad), TCP velocity fields.
- `ur-frames.bin.gz`: optional (`--raw-frames`) original binary realtime
  frames. Disabled by default to avoid duplicating parsed state.
- `force-50hz.csv.gz`: component averages per 20 ms receipt-time bin; source
  timestamp is the last source timestamp in the bin.
- `ur-50hz.csv.gz`: last state per 20 ms receipt-time bin. Lightweight preview,
  NOT anti-aliased or hardware-synchronized simulation resampling.
- `idle-force-summary.csv.gz`: idle windows up to 30 s, component running
  means/std/counts (population standard deviation), constant-memory calculation.
- `markers.csv`: experiment start/exit and optional collection-session markers.
- `metadata.json`: counts, command argv, cwd, errors, connections and status.
- `experiment.log`: stdout/stderr of the optional experiment command.

Wrapped execution starts only after both streams deliver >=20 frames and
their latest receive ages are <250 ms. Missing readiness for 15 seconds
prevents experiment launch. A stale stream for >0.5 seconds after readiness
is lost interrupts the wrapped process group using SIGINT. This is a data
integrity mechanism, NOT a certified robot safety stop; the experiment must
handle SIGINT safely and a human must retain E-stop access. A process that
does not exit within 10 seconds is flagged as possibly still running.

Standalone --duration defaults to 60 seconds; --watch listens until Ctrl+C.
Wrapped runs record until child
exit, rather than stopping at --duration. Ctrl+C forwards SIGINT to the child.
A recording failure is retained for diagnosis and must NOT be used as a
complete paired dataset. Check metadata.status, both counts, time continuity,
experiment_exit_code and logs: recorded data does not imply experiment success.

Raw saving starts enabled, pauses after --idle-seconds (default 30), and
resumes on measured motion. Detection uses linear/angular TCP speed >=0.1
mm/s / 0.001 rad/s OR accumulated joint/position changes >=0.0005 rad /
0.1 mm. These thresholds exceed typical state jitter but may miss smaller
or extremely slow motions: use --continuous when that detail matters.
During pauses both streams remain monitored and --pre-seconds (default 2,
maximum 10) is buffered with hard queue bounds. Idle force summaries remain.
Motion pause/resume markers explicitly distinguish intentional gaps from
loss. Do not interpolate through those gaps as if raw data were recorded.
Existing datasets are not deleted or rewritten. New full-rate streams are
losslessly compressed as they are written; analysis should read in chunks.

Two streams are NOT hardware synchronized. Controller uptime, ROS source
time and receipt clocks are different clocks; preserve them for alignment,
delay analysis and resampling. Do not pair CSV rows by equal row number.
Record calibration/TF, tool installation, payload and cable-route details
alongside each experiment when needed for simulation. Additional realtime
clients may consume controller resources; validate recording before motion.

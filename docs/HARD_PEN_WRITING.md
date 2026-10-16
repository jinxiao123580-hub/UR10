# Hard-pen TCP and writing prototype — 2026-10-16

## Status and evidence

- Named profile: `config/hard_pen_tcp.yaml`. Flange-to-tip XYZ is
  `[0.113898, -1.661301, 329.069491]` mm. Position only; no controller TCP write.
- Original samples and validation: `outputs/tcp_calibration/ballpoint-pen-20261016.json`.
  Nine fitting poses and three held-out poses; fitting RMS 0.347 mm, held-out
  maximum 0.991 mm. These are fixed-point consistency errors, not external
  absolute positioning accuracy.
- Paper registration: `outputs/paper_writing/a4-imported-20261016.json`.
  Three manually supplied corner XYs; their Zs do not define the writing plane.
  Unordered-corner inference chooses the long edge toward base +X (or +Y
  when vertical). Unmarked paper has a 180-degree ambiguity: check an air run.
- Actual contact measurement:
  `outputs/paper_writing/writing-20261016-143507-030047-contact.json`.
  Contact at base Z 26.745 mm followed by 2 mm retraction was recorded.
- Writing attempt: `outputs/paper_writing/writing-20261016-143507-030047.json`.
  Motion reached the third stroke and stopped at the 5 N overload guard.
  This is NOT proof of successful visible handwriting or complete delivery.
- Later attempts and failures are retained in `outputs/paper_writing/`;
  `writing-20261016-144931-263967.json` reports
  `completed_fixed_height_motion`, with its sibling contact JSON reporting
  successful contact/retraction. That establishes a completed motion sequence,
  not yet visual handwriting-quality acceptance or validation of later changes.
  12-point paper calibration history is in `outputs/paper_calibration/`.
  A single contact height does not calibrate paper tilt. Check each saved
  session's status and validation fields rather than assuming success.

## Run from a clone

Use the existing ROS Humble, camera/force environment and installed robot
model dependencies described by the repository setup documentation. Python
needs system NumPy/SciPy; execution additionally needs rclpy, geometry_msgs
and Pinocchio/robotpkg plus the existing UR10 model files. No hardware
dependencies are installed automatically. These scripts no longer require
`~/.dsh-analysis`; keep all sibling scripts and repository model assets.

```bash
cd /path/to/UR10
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
bash scripts/start_dashboard.sh

# Hardware-free tests
PYTHONNOUSERSITE=1 python3 scripts/test_write_ballpoint.py
PYTHONNOUSERSITE=1 python3 scripts/test_paper_plane_auto.py

# New sheet: teach any three distinct corners, use a NEW registration name
PYTHONNOUSERSITE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
python3 scripts/write_ballpoint.py --teach \
--registration outputs/paper_writing/a4-new.json

# Offline plan only (no robot connection/motion)
PYTHONNOUSERSITE=1 python3 scripts/write_ballpoint.py \
--registration outputs/paper_writing/a4-new.json --paper-z-mm 26.968

# Supervised air check, 5 mm above the assumed plane
PYTHONNOUSERSITE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
python3 scripts/write_ballpoint.py --registration outputs/paper_writing/a4-new.json \
--paper-z-mm 26.968 --air --execute

# Supervised writing: re-probe at first stroke, then use measured contact Z
PYTHONNOUSERSITE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
python3 scripts/write_ballpoint.py --registration outputs/paper_writing/a4-new.json \
--paper-z-mm 26.968 --execute
```

Archived registrations are tied to the physical paper, TCP calibration hash,
active controller TCP and recorded pen orientation. A clone on another PC
does not establish that today's physical scene matches an archived scene.
Re-teach if paper/tool mounting changes. Start with the pen raised 3–100 mm,
ensure no other motion program is running, supervise and keep E-stop available.

## Protection and remaining work

Horizontal setpoint 30 mm/s; finite steps reduce average speed. Vertical
contact motion remains slow. Raw baseline-relative force-vector delta >=5 N
stops motion; contact probe uses a 1 N trigger and <=6 mm descent.
Horizontal writing at >=2 N first stops and attempts upward-only 0.1 mm
unloading, <=1 mm total. No downward pressure chasing or tare under contact.
This is conservative unloading, NOT validated constant normal-force control.
Loss of state during contact stops execution; idle/air reconnection is bounded.
Joint-limit and model checks remain enabled. Unmodeled pen, cables and table
still require on-site checking. Latest speed/unloading changes have offline
coverage but are not certified by a complete successful hardware writing run.

Next acceptance: verify fresh-state reliability, paper tilt, visible strokes,
normal force across the sheet, and full start-to-finish run at the new speed.

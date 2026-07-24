# Viam Hackathon — Pick the Red Block

Vision-guided pick of a red block with a uFactory xArm6 + gripper and an Intel
RealSense D435I, using Viam's off-the-shelf services (color detector →
detections-to-segments 3D segmenter → motion service) and returning to a saved
home configuration.

## Setup

```bash
python -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
```

Create a `.env` (never committed) with the machine credentials:

```
VIAM_ADDRESS=<machine-address>.viam.cloud
VIAM_API_KEY=<api-key>
VIAM_API_KEY_ID=<api-key-id>
```

## Scripts

| File | Purpose |
|------|---------|
| `connect.py` | Robot connection helper. `disable_sessions=True` so a brief client-side disconnect doesn't trip the session heartbeat that auto-stops the arm mid-move. |
| `save_home.py` | Record the arm's current joint configuration to `home.json`. |
| `go_home.py` | Clear any xArm error/e-stop and move safely to the saved home config. |
| `pick.py` | Full pick: home → perceive block (3D segmenter, multi-sample + outlier filter) → approach → slow descent → grab → lift → home. `--dry` runs perception only (no motion). |
| `safety.py` | Shared safety helpers: `clear_error`, `set_speed`, `recover_and_home`. |
| `app_read.py` / `app_logs.py` / `push_config.py` | Viam App-API tooling used to read the machine config, pull logs, and push config changes. |

## Machine setup this relies on

- `arm-1` (`viam:ufactory:xArm6`) with `speed_degs_per_sec` capped for safety.
- A gripper (`gripper-1`) — note its frame origin is at the gripper **base**;
  fingertips are ~40mm below, so `pick.py` commands the frame 40mm high
  (`FINGER_OFFSET_MM`).
- `realsense-346222073240` (`viam:camera:realsense`, `align_color_depth: true`)
  mounted on `arm-1` and registered in the frame system.
- `color-detector` (`rdk:builtin:color_detector`, red `#FF0000`) and
  `object-segmenter` (`viam:vision:detections-to-segments`,
  `mean_k=50, sigma=1.5`) for stable 3D block localization.

## Safety measures

1. **Velocity** — arm capped to 20°/s (40°/s² accel); extra-slow 8°/s final descent.
2. **Accurate position** — block centroid sampled repeatedly and outlier-filtered;
   hard fingertip **z-floor** so the gripper can never drive into the table.
3. **E-stop recovery** — on start, clear errors and return to home first.

> Note: running from a laptop over the Viam cloud proxy is unreliable on
> congested networks; for production run this on the machine's own PC (local
> connection) or as a Viam module.

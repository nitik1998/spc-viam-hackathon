# Orchestrator scripts (xianhai + claude)

Command layer + calibration for robot17 pick/sort. Copy `.env.example` to `.env`
and fill in an API key from the machine's CONNECT tab. `pip install viam-sdk`.

## The demo command layer — robot.py
    python robot.py scan            # list blocks (color + world mm), NO motion
    python robot.py pick red        # home -> detect -> overhead refine -> calibrated grasp -> clamp -> lift
    python robot.py place left      # slots: left / middle / right
    python robot.py home | drop | status

Safety: workspace bounds (X ±350, Y 250-600, TCP z >= 40 so fingertips can't hit
the table), straight-line descent (LinearConstraint), collision-aware error
handling, edge-of-frame detection with an overhead re-detect pass.

## Calibration — calibrate.py  (DO THIS FIRST, once)
    python calibrate.py clear      # clear xArm overcurrent/error state
    python calibrate.py stage      # home -> detect -> park fingers at straddle height 6cm beside block
    # slide the block so it's centered between the fingertips, then:
    python calibrate.py measure    # NO motion; writes calibration.json (dx/dy aim + grasp TCP height)

robot.py auto-loads calibration.json. Without it, picks fly uncalibrated (~3-4cm off).

## Known facts (hard-won)
- Motion targets are the gripper TCP; fingertips extend ~55mm BELOW it. Grasp at TCP z~=60, never block height.
- RealSense min depth ~28cm: no close-range re-detection; detect from home / overhead only.
- Never add the RealSense's UVC nodes (video0/2/4...) as webcam components — blocks the realsense module.
- Multi-color: one color_detector + detections-to-segments pair per color; add names to the dicts in robot.py.

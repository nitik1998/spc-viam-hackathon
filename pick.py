"""Pick the red block safely, with verbose progress logging.

Safety: (#1) capped velocity + slow smooth descent, (#2) multi-sample
outlier-filtered block position, (#3) clear e-stop + home first on start.

Logging: object position (each sample + world), gripper/arm pose and holding
state at every stage, and every motion step start/finish. All logs flush
immediately; moves are timeout-guarded and errors are printed (never silent).

Usage: python pick.py        (full pick)
       python pick.py --dry   (perception only, no arm motion)
"""
import asyncio
import json
import os
import statistics
import sys
import time
import traceback

from connect import connect
from safety import set_speed
from viam.components.arm import Arm
from viam.components.camera import Camera
from viam.components.gripper import Gripper
from viam.services.vision import Vision
from viam.services.motion import Motion
from viam.proto.component.arm import JointPositions
from viam.proto.common import Pose, PoseInFrame

DRY_RUN = "--dry" in sys.argv

MARKER_GRIPPERS = {"pick-marker", "place-marker", "table"}
FINGER_OFFSET_MM = 40.0  # gripper FRAME is at the gripper base; fingertips are 40mm below it.
                         # Command the frame this much higher so the fingertips reach the target.
APPROACH_MM = 100.0      # fingertip height above the block for the pre-grasp approach
GRASP_DROP_MM = 20.0     # descend fingertips this far below the block-top centroid to grip the body
Z_FLOOR_MM = -40.0       # hard safety floor on FINGERTIP world z (block ~40mm tall, table ~ -45)
CRUISE_SPEED = 20.0
DESCENT_SPEED = 8.0
SAMPLES = 5              # target number of GOOD samples to collect
MIN_VALID = 3            # minimum good samples required to proceed
TOL_MM = 20.0            # reject samples farther than this from the median
MAX_PERCEPTION_ATTEMPTS = 30  # keep trying through reconnect blips
RETRY_WAIT_S = 1.5       # wait out a client reconnect after a dropped call
MOVE_TIMEOUT = 90.0

_t0 = time.time()


def log(*a):
    print(f"[{time.time() - _t0:6.1f}s]", *a, flush=True)


async def guarded(coro, what):
    """Await a motion coroutine with a timeout; log and re-raise on failure."""
    log(f"  -> {what} ...")
    try:
        r = await asyncio.wait_for(coro, MOVE_TIMEOUT)
        log(f"  <- {what} done (result={r})")
        return r
    except asyncio.TimeoutError:
        log(f"  !! {what} TIMED OUT after {MOVE_TIMEOUT}s")
        raise
    except Exception as e:
        log(f"  !! {what} FAILED: {type(e).__name__}: {str(e)[:150]}")
        raise


async def status(m, motion, arm, gripper, grip_n, tag):
    """Log arm end pose, gripper world pose, and holding state."""
    try:
        p = await arm.get_end_position()
        log(f"[{tag}] arm end pose: x={p.x:.1f} y={p.y:.1f} z={p.z:.1f}")
    except Exception as e:
        log(f"[{tag}] arm pose err: {str(e)[:80]}")
    try:
        gp = (await motion.get_pose(grip_n, "world")).pose
        log(f"[{tag}] gripper world pose: x={gp.x:.1f} y={gp.y:.1f} z={gp.z:.1f}")
    except Exception as e:
        log(f"[{tag}] gripper pose err: {str(e)[:80]}")
    try:
        h = await gripper.is_holding_something()
        log(f"[{tag}] gripper holding: {getattr(h, 'is_holding_something', h)}")
    except Exception as e:
        log(f"[{tag}] gripper holding err: {str(e)[:80]}")


async def discover(m):
    by = lambda st: [rn.name for rn in m.resource_names if rn.subtype == st]
    arms, cams, grips, visions = by("arm"), by("camera"), by("gripper"), by("vision")
    if not arms:
        raise RuntimeError("no arm on the machine")
    arm = arms[0]
    grip = next((g for g in grips if g == "gripper-1"),
                next((g for g in grips if g not in MARKER_GRIPPERS), None))
    if not grip:
        raise RuntimeError("no non-marker gripper found")
    cam = None
    for c in cams:
        try:
            if (await Camera.from_robot(m, c).get_properties()).supports_pcd:
                cam = c
                break
        except Exception:
            continue
    if not cam:
        cam = next((c for c in cams if "realsense" in c.lower()), None)
    if not cam:
        raise RuntimeError(f"no depth camera among {cams}")
    seg = None
    for v in sorted(visions, key=lambda n: 0 if "segment" in n.lower() else 1):
        try:
            if (await Vision.from_robot(m, v).get_properties()).object_point_clouds_supported:
                seg = v
                break
        except Exception:
            continue
    if not seg:
        raise RuntimeError(f"no 3D-segmenter vision service among {visions}")
    return arm, grip, cam, seg


def choose_target(objs):
    pairs = [(o, g) for o in objs for g in o.geometries.geometries]
    if not pairs:
        return None, None
    named = [(o, g) for (o, g) in pairs
             if any(k in (g.label or "").lower() for k in ("red", "block"))]
    if named:
        return named[0]
    vol = lambda g: (g.box.dims_mm.x * g.box.dims_mm.y * g.box.dims_mm.z
                     if g.HasField("box") else 0.0)
    return max(pairs, key=lambda og: vol(og[1]))


async def locate_block(m, seg_n, cam_n):
    log(f"perceiving block from '{seg_n}' (need {MIN_VALID}+ good of {SAMPLES}) ...")
    seg = Vision.from_robot(m, seg_n)
    pts = []
    attempt = 0
    while len(pts) < SAMPLES and attempt < MAX_PERCEPTION_ATTEMPTS:
        attempt += 1
        try:
            objs = await seg.get_object_point_clouds(cam_n)
            obj, geom = choose_target(objs)
            if obj is None:
                log(f"  attempt {attempt}: no object")
                await asyncio.sleep(0.3)
                continue
            src = obj.geometries.reference_frame or cam_n
            c = geom.center
            w = (await m.transform_pose(
                PoseInFrame(reference_frame=src, pose=c), "world")).pose
        except Exception as e:
            log(f"  attempt {attempt}: err {str(e)[:50]} (retry after reconnect)")
            await asyncio.sleep(RETRY_WAIT_S)
            continue
        log(f"  good {len(pts) + 1}: camera=({c.x:.1f},{c.y:.1f},{c.z:.1f}) "
            f"world=({w.x:.1f},{w.y:.1f},{w.z:.1f})")
        pts.append((w.x, w.y, w.z))
        await asyncio.sleep(0.2)
    if len(pts) < MIN_VALID:
        raise RuntimeError(f"only {len(pts)} valid detections (<{MIN_VALID}) after {attempt} attempts")
    med = [statistics.median([p[i] for p in pts]) for i in range(3)]
    kept = [p for p in pts if max(abs(p[i] - med[i]) for i in range(3)) <= TOL_MM]
    log(f"kept {len(kept)}/{len(pts)} samples within {TOL_MM}mm of median")
    if len(kept) < MIN_VALID:
        raise RuntimeError(f"detections too scattered (>{TOL_MM}mm)")
    med = [statistics.median([p[i] for p in kept]) for i in range(3)]
    log(f"BLOCK POSITION (world): x={med[0]:.1f} y={med[1]:.1f} z={med[2]:.1f}")
    return med[0], med[1], med[2]


async def run(m):
    if not os.path.exists("home.json"):
        raise RuntimeError("home.json not found -- run save_home.py first")
    home = json.load(open("home.json"))

    log("discovering resources ...")
    arm_n, grip_n, cam_n, seg_n = await discover(m)
    log(f"arm={arm_n} gripper={grip_n} camera={cam_n} segmenter={seg_n}")
    arm = Arm.from_robot(m, arm_n)
    gripper = Gripper.from_robot(m, grip_n)
    motion = Motion.from_robot(m, "builtin")

    await guarded(motion.get_pose(cam_n, "world"), "confirm camera in frame system")

    if not DRY_RUN:
        log("SAFETY: clearing errors + homing first (#3)")
        await guarded(arm.do_command({"clear_error": True}), "clear_error")
        await set_speed(arm, CRUISE_SPEED)
        await status(m, motion, arm, gripper, grip_n, "start")
        await guarded(arm.move_to_joint_positions(JointPositions(values=home["joints_deg"])),
                      "home before pick")
        await status(m, motion, arm, gripper, grip_n, "home")

    wx, wy, wz = await locate_block(m, seg_n, cam_n)

    def down(fingertip_z):
        # command the gripper FRAME 40mm above the desired FINGERTIP height
        return PoseInFrame(reference_frame="world",
                           pose=Pose(x=wx, y=wy, z=fingertip_z + FINGER_OFFSET_MM,
                                     o_x=0, o_y=0, o_z=-1, theta=0))
    approach_tip = wz + APPROACH_MM
    grasp_tip = max(wz - GRASP_DROP_MM, Z_FLOOR_MM)  # never drive fingertips into the table
    approach, grasp = down(approach_tip), down(grasp_tip)
    log(f"block top(tip)={wz:.1f}  approach tip={approach_tip:.1f}  grasp tip={grasp_tip:.1f}  "
        f"(frame = tip+{FINGER_OFFSET_MM}mm, floor {Z_FLOOR_MM})")

    if DRY_RUN:
        log("[dry-run] perception OK; skipping motion.")
        return

    await set_speed(arm, CRUISE_SPEED)
    await guarded(gripper.open(), "open gripper")
    await guarded(motion.move(component_name=grip_n, destination=approach), "move to approach")
    await status(m, motion, arm, gripper, grip_n, "approach")

    log(f"SAFETY: slow descent at {DESCENT_SPEED} deg/s (#1)")
    await set_speed(arm, DESCENT_SPEED)
    await guarded(motion.move(component_name=grip_n, destination=grasp), "descend to grasp")
    await status(m, motion, arm, gripper, grip_n, "grasp")

    got = await guarded(gripper.grab(), "grab")
    await status(m, motion, arm, gripper, grip_n, "grabbed")

    await set_speed(arm, CRUISE_SPEED)
    await guarded(motion.move(component_name=grip_n, destination=approach), "lift")
    await status(m, motion, arm, gripper, grip_n, "lifted")

    await guarded(arm.move_to_joint_positions(JointPositions(values=home["joints_deg"])),
                  "return home")
    await status(m, motion, arm, gripper, grip_n, "home")
    log(f"DONE. grabbed={got}")


async def main() -> None:
    log(f"=== pick.py {'(DRY RUN)' if DRY_RUN else '(FULL PICK)'} ===")
    m = await connect()
    try:
        await run(m)
    except Exception as e:
        log(f"ABORTED: {type(e).__name__}: {e}")
        traceback.print_exc()
        try:
            await Arm.from_robot(m, "arm-1").stop()
            log("arm stopped.")
        except Exception as se:
            log(f"stop failed: {str(se)[:80]}")
    finally:
        await m.close()
        log("connection closed.")


if __name__ == "__main__":
    asyncio.run(main())

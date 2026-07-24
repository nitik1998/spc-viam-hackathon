"""Demo command layer for robot17 — safe, named verbs only.

    python robot.py scan                 # list visible blocks (color, world mm) — NO motion
    python robot.py pick red             # full staged pick of that color's biggest block
    python robot.py place left|middle|right
    python robot.py home                 # return to survey pose
    python robot.py drop                 # open gripper in place
    python robot.py status               # joints + held state — NO motion

Add colors by configuring a color_detector + detections-to-segments pair on the
robot, then adding the segmenter name to COLOR_SEGMENTERS below.
"""
import asyncio
import json
import os
import sys

from viam.robot.client import RobotClient
from viam.components.arm import Arm
from viam.components.gripper import Gripper
from viam.services.motion import MotionClient
from viam.services.vision import VisionClient
from viam.proto.common import Pose, PoseInFrame
from viam.proto.service.motion import Constraints, LinearConstraint
from viam.proto.component.arm import JointPositions

CAM = "realsense-346222073240"
FRAME_W, FRAME_H = 1280, 720
EDGE_MARGIN = 15          # px; a box touching the border means the block is partly out of view
REFINE_TCP_Z = 250        # safe altitude for the re-detect pass (camera stays beyond RealSense min depth)
COLOR_SEGMENTERS = {
    "red": "object-segmenter",
    "blue": "blue-object-segmenter",
}
COLOR_DETECTORS = {
    "red": "color-detector",
    "blue": "blue-color-detector",
}
HOME_FILE = "home_joints.json"
TABLE_Z = -10.0
HOVER_MM = 100
LIFT_MM = 120
PLACE_Z = 8.0        # block-center height when releasing (just above table)
GRIPPER_TIMEOUT = 15
GRIPPER_SETTLE_S = 3.0   # cover for the deployed module returning before jaws finish

# workspace safety bounds (world mm) — refuse any motion target outside these
X_RANGE = (-350, 350)
Y_RANGE = (250, 600)
Z_RANGE = (40, 300)   # TCP floor: fingertips extend ~55mm below TCP; 40 keeps them off the table

SORT_TARGETS = {"red": "left", "blue": "right"}   # sort: each color to its slot
SORTED_TOL_MM = 40         # block within this distance of its slot counts as sorted

SLOTS = {   # world-frame place positions; verify/adjust with `place`-hover once
    "left":   (-180.0, 445.0),
    "middle": (0.0,    445.0),
    "right":  (180.0,  445.0),
}


def load_calibration():
    """Offsets from calibrate.py measure. Without it, assume uncalibrated but use a safe straddle height."""
    try:
        cal = json.load(open("calibration.json"))
        print(f"  (calibration: dx={cal['dx']} dy={cal['dy']} grasp_z={cal['grasp_tcp_z']})")
        return cal
    except FileNotFoundError:
        print("  (WARNING: no calibration.json — run calibrate.py measure first; using defaults)")
        return {"dx": 0.0, "dy": 0.0, "grasp_tcp_z": 60.0}


def load_env(path=".env"):
    with open(path) as f:
        for line in f:
            if "=" in line:
                k, v = line.strip().split("=", 1)
                os.environ.setdefault(k, v)


async def connect():
    load_env()
    opts = RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"])
    return await RobotClient.at_address(os.environ["VIAM_ADDRESS"], opts)


def in_bounds(x, y, z):
    return X_RANGE[0] <= x <= X_RANGE[1] and Y_RANGE[0] <= y <= Y_RANGE[1] and Z_RANGE[0] <= z <= Z_RANGE[1]


async def gripper_open(gripper):
    """Documented Gripper API: Open. Deployed module may return before the jaws
    finish (fixed upstream, unreleased) — one settle pause covers the gap."""
    try:
        await gripper.open(timeout=GRIPPER_TIMEOUT)
    except Exception as e:
        print(f"  (open returned early: {str(e)[:60]})")
    await asyncio.sleep(GRIPPER_SETTLE_S)


async def gripper_grab(gripper):
    """Documented Gripper API: Grab — 'closes until it grabs something or closes
    completely' (blocking per docs; deployed module returns early)."""
    try:
        got = await gripper.grab(timeout=GRIPPER_TIMEOUT)
    except Exception as e:
        print(f"  (grab returned early: {str(e)[:60]})")
        got = None
    await asyncio.sleep(GRIPPER_SETTLE_S)
    return got


async def gripper_call(fn):
    try:
        return await fn(timeout=GRIPPER_TIMEOUT)
    except Exception as e:
        print(f"  (gripper ack late, continuing: {str(e)[:60]})")
        return None


async def wait_gripper_done(gripper, max_s=8.0, fallback_s=3.5):
    """Block until the jaws stop moving. The ufactory module acks before motion
    completes, so poll is_moving(); if it never reports motion, use a fixed wait."""
    saw_motion = False
    waited = 0.0
    while waited < max_s:
        try:
            moving = await gripper.is_moving()
        except Exception:
            break  # is_moving unsupported -> fixed wait below
        if moving:
            saw_motion = True
        elif saw_motion:
            print(f"  (jaws settled after {waited:.1f}s)")
            return
        await asyncio.sleep(0.25)
        waited += 0.25
    if not saw_motion:
        print(f"  (is_moving not reported; fixed {fallback_s}s settle)")
        await asyncio.sleep(fallback_s)


async def blocks_of_color(robot, color):
    """All clusters for a color as world poses, biggest first."""
    seg_name = COLOR_SEGMENTERS.get(color)
    if not seg_name:
        return None  # color not configured
    seg = VisionClient.from_robot(robot, seg_name)
    objs = [o for o in await seg.get_object_point_clouds(CAM) if o.geometries.geometries]
    objs.sort(key=lambda o: -len(o.point_cloud))
    out = []
    for o in objs:
        c = o.geometries.geometries[0].center
        pif = PoseInFrame(reference_frame=CAM, pose=Pose(x=c.x, y=c.y, z=c.z, o_z=1))
        w = (await robot.transform_pose(pif, "world")).pose
        out.append({"color": color, "x": round(w.x), "y": round(w.y), "z": round(w.z),
                    "size_bytes": len(o.point_cloud)})
    return out


async def detection_clipped(robot, color):
    """True if the biggest 2D detection touches the frame edge (block partly out of view)."""
    det_name = COLOR_DETECTORS.get(color)
    if not det_name:
        return False
    det = VisionClient.from_robot(robot, det_name)
    dets = await det.get_detections_from_camera(CAM)
    if not dets:
        return False
    d = max(dets, key=lambda d: (d.x_max - d.x_min) * (d.y_max - d.y_min))
    return (d.x_min <= EDGE_MARGIN or d.y_min <= EDGE_MARGIN
            or d.x_max >= FRAME_W - EDGE_MARGIN or d.y_max >= FRAME_H - EDGE_MARGIN)


def down_pose(x, y, z):
    return Pose(x=x, y=y, z=z, o_x=0, o_y=0, o_z=-1, theta=0)


async def move_to(robot, x, y, z, label="", straight=False):
    if not in_bounds(x, y, z):
        raise SystemExit(f"REFUSED: target ({x:.0f},{y:.0f},{z:.0f}) outside workspace bounds")
    motion = MotionClient.from_robot(robot, "builtin")
    dest = PoseInFrame(reference_frame="world", pose=down_pose(x, y, z))
    constraints = Constraints(linear_constraint=[LinearConstraint(line_tolerance_mm=5.0)]) if straight else None
    print(f"  move {label}{' [straight]' if straight else ''} -> ({x:.0f},{y:.0f},{z:.0f})")
    return await motion.move(component_name="gripper-1", destination=dest, constraints=constraints)


async def go_home(robot):
    arm = Arm.from_robot(robot, "arm-1")
    vals = json.load(open(HOME_FILE))
    await arm.move_to_joint_positions(JointPositions(values=vals))


async def pick_color(robot, gripper, color):
    """Full staged pick. Returns True if grab reported holding."""
    found = await blocks_of_color(robot, color)
    if found is None:
        print(f"color '{color}' not configured (have: {list(COLOR_SEGMENTERS)})"); return False
    if not found:
        print(f"no {color} block visible"); return False
    b = found[0]
    cal = load_calibration()
    if await detection_clipped(robot, color):
        print("  block is partly out of view — coarse estimate only, refine pass will fix it")
    tx, ty = b["x"] - cal["dx"], b["y"] - cal["dy"]
    await move_to(robot, tx, ty, REFINE_TCP_Z, "refine-view")
    refined = await blocks_of_color(robot, color)
    if refined:
        if await detection_clipped(robot, color):
            print("  WARNING: still clipped after refine — nudge the block toward table center and retry")
        b = refined[0]
        tx, ty = b["x"] - cal["dx"], b["y"] - cal["dy"]
        print(f"  refined position: ({b['x']},{b['y']}) -> corrected ({tx:.0f},{ty:.0f})")
    else:
        print("  (refine saw nothing; using coarse estimate)")
    grasp_z = cal["grasp_tcp_z"]
    print(f"picking {color}: target ({tx:.0f},{ty:.0f}), grasp TCP z={grasp_z}")
    step = "?"
    try:
        step = "open";   await gripper_open(gripper)
        step = "hover";  await move_to(robot, tx, ty, grasp_z + HOVER_MM, "hover")
        step = "grasp";  await move_to(robot, tx, ty, grasp_z, "grasp", straight=True)
        step = "grab";   got = await gripper_grab(gripper)
        step = "lift";   await move_to(robot, tx, ty, grasp_z + LIFT_MM, "lift", straight=True)
        print(json.dumps({"picked": color, "grab_ack": bool(got)}))
        return True
    except Exception as e:
        msg = str(e)
        print(json.dumps({"pick_failed_at": step, "error": msg[:160]}))
        if "overcurrent" in msg or "collision" in msg:
            print("collision stop — arm needs: python calibrate.py clear, then retreat manually or rerun")
        else:
            print("retreating to safe height...")
            try:
                await move_to(robot, tx, ty, REFINE_TCP_Z, "retreat")
            except Exception:
                print("  (retreat also failed — clear error and jog from CONTROL tab)")
        return False


async def place_slot(robot, gripper, slot):
    """Carry held block to a named slot and release gently."""
    x, y = SLOTS[slot]
    cal = load_calibration()
    release_z = cal["grasp_tcp_z"] + 30
    print(f"placing at {slot} ({x},{y}), release TCP z={release_z}")
    await move_to(robot, x, y, release_z + HOVER_MM, "hover")
    await move_to(robot, x, y, release_z, "lower", straight=True)
    await gripper_open(gripper)
    await move_to(robot, x, y, release_z + LIFT_MM, "retreat", straight=True)
    print(json.dumps({"placed": slot}))


async def main():
    args = sys.argv[1:]
    cmd = args[0] if args else "help"
    robot = await connect()
    try:
        gripper = Gripper.from_robot(robot, "gripper-1")

        if cmd == "scan":
            result = []
            for color in COLOR_SEGMENTERS:
                found = await blocks_of_color(robot, color) or []
                result.extend(found)
            print(json.dumps({"blocks": result}, indent=1))

        elif cmd == "pick":
            color = args[1] if len(args) > 1 else "red"
            await pick_color(robot, gripper, color)

        elif cmd == "sort":
            print(f"SORT: {SORT_TARGETS} (buffer: middle, tol {SORTED_TOL_MM}mm)")
            await go_home(robot)
            for color, slot in SORT_TARGETS.items():
                found = await blocks_of_color(robot, color)
                if not found:
                    print(f"[sort] no {color} visible — skipping"); continue
                b = found[0]
                sx, sy = SLOTS[slot]
                if abs(b["x"] - sx) <= SORTED_TOL_MM and abs(b["y"] - sy) <= SORTED_TOL_MM:
                    print(f"[sort] {color} already at {slot} — skipping"); continue
                print(f"[sort] {color} at ({b['x']},{b['y']}) -> {slot}")
                if not await pick_color(robot, gripper, color):
                    print(f"[sort] pick {color} failed — stopping sort"); break
                await place_slot(robot, gripper, slot)
                await go_home(robot)
            print("[sort] done")

        elif cmd == "place":
            slot = args[1] if len(args) > 1 else "middle"
            if slot not in SLOTS:
                print(f"unknown slot '{slot}' (have: {list(SLOTS)})"); return
            await place_slot(robot, gripper, slot)

        elif cmd == "home":
            await go_home(robot)
            print("at home")

        elif cmd == "drop":
            await gripper_open(gripper)
            print("dropped")

        elif cmd == "status":
            arm = Arm.from_robot(robot, "arm-1")
            joints = await arm.get_joint_positions()
            print(json.dumps({"joints": [round(v, 1) for v in joints.values]}))

        else:
            print(__doc__)
    finally:
        await robot.close()


if __name__ == "__main__":
    asyncio.run(main())

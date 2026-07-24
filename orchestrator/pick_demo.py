"""Staged pick pipeline for robot17.

    python pick_demo.py status     # read-only: detect block, print world coords (NO motion)
    python pick_demo.py save-home  # record current arm joints as the survey/home pose
    python pick_demo.py home       # move arm back to saved home pose (MOTION)
    python pick_demo.py hover      # move gripper 100mm above detected block (MOTION)
    python pick_demo.py pick       # full pick: hover -> open -> descend -> grab -> lift (MOTION)
    python pick_demo.py open       # open gripper (no arm motion)
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
from viam.proto.component.arm import JointPositions

CAM = "realsense-346222073240"
SEGMENTER = "object-segmenter"
HOME_FILE = "home_joints.json"
HOVER_MM = 100      # standoff above block for approach
LIFT_MM = 120       # lift height after grasp
GRIPPER_TIMEOUT = 15  # xArm gripper acks slowly; default 2s SDK timeout is too tight


async def gripper_call(coro_fn, *args, **kwargs):
    """Run a gripper op with a long timeout; a late/lost ack is a warning, not a crash."""
    try:
        return await coro_fn(*args, timeout=GRIPPER_TIMEOUT, **kwargs)
    except Exception as e:
        print(f"   (gripper ack issue, continuing: {str(e)[:80]})")
        await asyncio.sleep(2)  # give the physical motion time to finish
        return None


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


async def biggest_block_world(robot):
    """Detect blocks, return (world_pose, cluster_info) for the largest cluster."""
    seg = VisionClient.from_robot(robot, SEGMENTER)
    objs = await seg.get_object_point_clouds(CAM)
    if not objs:
        return None, None
    objs = [o for o in objs if o.geometries.geometries]
    obj = max(objs, key=lambda o: len(o.point_cloud))
    center = obj.geometries.geometries[0].center
    in_cam = PoseInFrame(reference_frame=CAM,
                         pose=Pose(x=center.x, y=center.y, z=center.z, o_z=1))
    in_world = await robot.transform_pose(in_cam, "world")
    return in_world.pose, {
        "clusters": len(objs),
        "points_bytes": len(obj.point_cloud),
        "cam_xyz": (round(center.x), round(center.y), round(center.z)),
    }


def grasp_pose(world_pose, dz):
    """Pose dz mm above the block center, gripper pointing straight down."""
    return Pose(x=world_pose.x, y=world_pose.y, z=world_pose.z + dz,
                o_x=0, o_y=0, o_z=-1, theta=0)


async def move_gripper(robot, pose):
    motion = MotionClient.from_robot(robot, "builtin")
    dest = PoseInFrame(reference_frame="world", pose=pose)
    ok = await motion.move(component_name="gripper-1", destination=dest)
    return ok


async def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    robot = await connect()
    try:
        arm = Arm.from_robot(robot, "arm-1")
        gripper = Gripper.from_robot(robot, "gripper-1")

        if cmd == "status":
            joints = await arm.get_joint_positions()
            print("arm joints:", [round(v, 1) for v in joints.values])
            pose, info = await biggest_block_world(robot)
            if pose is None:
                print("no block detected")
                return
            print(f"clusters seen: {info['clusters']}  biggest: {info['points_bytes']}B at cam {info['cam_xyz']}mm")
            print(f"BLOCK IN WORLD: x={pose.x:.0f} y={pose.y:.0f} z={pose.z:.0f} mm")
            print(f"(sanity: fragment says table pick height is z~-10; block-center z should be near that)")

        elif cmd == "save-home":
            joints = await arm.get_joint_positions()
            with open(HOME_FILE, "w") as f:
                json.dump(list(joints.values), f)
            print("home saved:", [round(v, 1) for v in joints.values])

        elif cmd == "home":
            vals = json.load(open(HOME_FILE))
            await arm.move_to_joint_positions(JointPositions(values=vals))
            print("at home")

        elif cmd == "open":
            await gripper_call(gripper.open)
            print("gripper open")

        elif cmd == "hover":
            pose, info = await biggest_block_world(robot)
            if pose is None:
                print("no block detected"); return
            print(f"block at world ({pose.x:.0f},{pose.y:.0f},{pose.z:.0f}), moving to {HOVER_MM}mm above...")
            ok = await move_gripper(robot, grasp_pose(pose, HOVER_MM))
            print("hover done" if ok else "move returned False")

        elif cmd == "pick":
            pose, info = await biggest_block_world(robot)
            if pose is None:
                print("no block detected"); return
            print(f"block at world ({pose.x:.0f},{pose.y:.0f},{pose.z:.0f})")
            print("1/5 hover..."); await move_gripper(robot, grasp_pose(pose, HOVER_MM))
            print("2/5 open gripper..."); await gripper_call(gripper.open)
            print("3/5 descend..."); await move_gripper(robot, grasp_pose(pose, 0))
            print("4/5 grab...")
            got = await gripper_call(gripper.grab)
            print(f"   grab returned: {got}")
            print("5/5 lift..."); await move_gripper(robot, grasp_pose(pose, LIFT_MM))
            print("PICK COMPLETE — holding block" if got else "PICK DONE but grab=False (empty?)")

        else:
            print(__doc__)
    finally:
        await robot.close()


if __name__ == "__main__":
    asyncio.run(main())

"""Calibration helper.
  python calibrate.py clear      # clear xArm error state
  python calibrate.py stage      # hover over detected block, then lower to straddle height
  python calibrate.py measure    # read gripper world pose vs detected block pose -> offsets
"""
import asyncio, json, os, sys
from sanity_check import load_env
from viam.robot.client import RobotClient
from viam.components.arm import Arm
from viam.services.motion import MotionClient
from viam.services.vision import VisionClient
from viam.proto.common import Pose, PoseInFrame

CAM = "realsense-346222073240"
STRADDLE_TCP_Z = 60  # TCP height where fingertips straddle a table block

async def connect():
    load_env()
    opts = RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"])
    return await RobotClient.at_address(os.environ["VIAM_ADDRESS"], opts)

async def detect(robot):
    seg = VisionClient.from_robot(robot, "object-segmenter")
    objs = [o for o in await seg.get_object_point_clouds(CAM) if o.geometries.geometries]
    if not objs:
        return None
    o = max(objs, key=lambda x: len(x.point_cloud))
    c = o.geometries.geometries[0].center
    pif = PoseInFrame(reference_frame=CAM, pose=Pose(x=c.x, y=c.y, z=c.z, o_z=1))
    return (await robot.transform_pose(pif, "world")).pose

async def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "measure"
    robot = await connect()
    try:
        arm = Arm.from_robot(robot, "arm-1")
        motion = MotionClient.from_robot(robot, "builtin")

        if cmd == "clear":
            resp = await arm.do_command({"clear_error": []})
            print("clear_error sent:", resp)

        elif cmd == "stage":
            vals = json.load(open("home_joints.json"))
            from viam.proto.component.arm import JointPositions
            print("  going home first...")
            await arm.move_to_joint_positions(JointPositions(values=vals))
            b = await detect(robot)
            if b is None:
                print("no block detected"); return
            print(f"block detected at ({b.x:.0f},{b.y:.0f},{b.z:.0f})")
            if not (-350 <= b.x <= 350 and 250 <= b.y <= 600):
                print("REFUSED: detection outside table workspace — is the camera seeing the table?"); return
            for z in (100, STRADDLE_TCP_Z):
                dest = PoseInFrame(reference_frame="world",
                                   pose=Pose(x=b.x + 60, y=b.y, z=z, o_x=0, o_y=0, o_z=-1, theta=0))
                print(f"  moving TCP to z={z}...")
                await motion.move(component_name="gripper-1", destination=dest)
            print("STAGED. Now slide the block so it sits centered between the fingertips, then run: measure")

        elif cmd == "measure":
            gp = await motion.get_pose("gripper-1", "world")
            tcp = gp.pose
            b = await detect(robot)
            if b is None:
                print("no block detected"); return
            dx, dy = b.x - tcp.x, b.y - tcp.y
            print(f"gripper TCP at world ({tcp.x:.0f},{tcp.y:.0f},{tcp.z:.0f})")
            print(f"camera says block at ({b.x:.0f},{b.y:.0f},{b.z:.0f})")
            print(f"==> CAL_DX={dx:.0f}  CAL_DY={dy:.0f}   (camera overshoots by this; subtract from targets)")
            print(f"==> GRASP_TCP_Z = {tcp.z:.0f} (fingers straddling now)")
            with open("calibration.json", "w") as f:
                json.dump({"dx": round(dx, 1), "dy": round(dy, 1), "grasp_tcp_z": round(tcp.z, 1)}, f)
            print("saved to calibration.json — robot.py will use it automatically")
    finally:
        await robot.close()

asyncio.run(main())

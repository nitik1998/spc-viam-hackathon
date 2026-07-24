"""Connect to robot17 and verify we can see all resources."""
import asyncio
import os

from viam.robot.client import RobotClient
from viam.components.arm import Arm
from viam.components.camera import Camera
from viam.services.vision import VisionClient


def load_env(path=".env"):
    with open(path) as f:
        for line in f:
            if "=" in line:
                k, v = line.strip().split("=", 1)
                os.environ.setdefault(k, v)


async def main():
    load_env()
    opts = RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"],
        api_key_id=os.environ["VIAM_API_KEY_ID"],
    )
    robot = await RobotClient.at_address(os.environ["VIAM_ADDRESS"], opts)
    print("connected!")
    print("\nresources:")
    for name in robot.resource_names:
        print(f"  {name.type}/{name.subtype}: {name.name}")

    arm = Arm.from_robot(robot, "arm-1")
    joints = await arm.get_joint_positions()
    print(f"\narm joints: {[round(v, 2) for v in joints.values]}")

    cam = Camera.from_robot(robot, "camera-1")
    images, _meta = await cam.get_images()
    for im in images:
        print(f"camera frame: {im.width}x{im.height} {im.mime_type} ({len(im.data)} bytes)")

    try:
        detector = VisionClient.from_robot(robot, "red-detector")
        detections = await detector.get_detections_from_camera("camera-1")
        print(f"red-detector: {len(detections)} detection(s)")
        for d in detections:
            print(f"  box=({d.x_min},{d.y_min})-({d.x_max},{d.y_max}) conf={d.confidence:.2f} class={d.class_name}")
    except Exception as e:
        print(f"red-detector check failed: {e}")

    await robot.close()


if __name__ == "__main__":
    asyncio.run(main())

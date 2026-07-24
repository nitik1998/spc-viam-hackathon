import asyncio, os
from sanity_check import load_env
from viam.robot.client import RobotClient
from viam.services.vision import VisionClient

CAM = "realsense-346222073240"

async def main():
    load_env()
    opts = RobotClient.Options.with_api_key(api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"])
    robot = await RobotClient.at_address(os.environ["VIAM_ADDRESS"], opts)

    det = VisionClient.from_robot(robot, "color-detector")
    try:
        dets = await det.get_detections_from_camera(CAM)
        print(f"color-detector: {len(dets)} detection(s)")
        for d in dets:
            print(f"  {d.class_name} conf={d.confidence:.2f} box=({d.x_min},{d.y_min})-({d.x_max},{d.y_max})")
    except Exception as e:
        print(f"color-detector ERROR: {str(e)[:150]}")

    seg = VisionClient.from_robot(robot, "object-segmenter")
    try:
        objs = await seg.get_object_point_clouds(CAM)
        print(f"object-segmenter: {len(objs)} object(s)")
        for o in objs:
            c = o.geometries.geometries[0].center if o.geometries.geometries else None
            print(f"  points={len(o.point_cloud)}B center=({c.x:.0f},{c.y:.0f},{c.z:.0f})mm" if c else "  (no geometry)")
    except Exception as e:
        print(f"object-segmenter ERROR: {str(e)[:150]}")
    await robot.close()

asyncio.run(main())

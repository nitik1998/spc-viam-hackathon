import json
import os

from viam.robot.client import RobotClient
from viam.components.arm import Arm
from viam.components.camera import Camera
from viam.components.gripper import Gripper
from viam.services.generic import Generic as GenericService
from viam.services.mlmodel import MLModelClient
from viam.services.vision import VisionClient

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
KEY_FILE = os.path.join(BASE_DIR, 'api-key.json')

async def connect() -> RobotClient:
    """Connect to your machine with your API key and return a client."""
    creds = json.load(open(KEY_FILE))
    opts = RobotClient.Options.with_api_key(
        api_key=creds["key"], api_key_id=creds["key_id"]
    )
    return await RobotClient.at_address('robot17-main.ag9khwy6jn.viam.cloud', opts)

async def main():
    async with await connect() as machine:
        print('Resources:')
        print(machine.resource_names)
        
        # arm-1
        arm_1 = Arm.from_robot(machine, "arm-1")
        arm_1_return_value = await arm_1.get_end_position()
        print(f"arm-1 get_end_position return value: {arm_1_return_value}")

        # camera-1
        camera_1 = Camera.from_robot(machine, "camera-1")
        camera_1_return_value = await camera_1.get_images()
        print(f"camera-1 get_images return value: {camera_1_return_value}")

        # gripper-1
        gripper_1 = Gripper.from_robot(machine, "gripper-1")
        gripper_1_return_value = await gripper_1.is_moving()
        print(f"gripper-1 is_moving return value: {gripper_1_return_value}")

        # table
        table = Gripper.from_robot(machine, "table")
        table_return_value = await table.is_moving()
        print(f"table is_moving return value: {table_return_value}")

        # pick-marker
        pick_marker = Gripper.from_robot(machine, "pick-marker")
        pick_marker_return_value = await pick_marker.is_moving()
        print(f"pick-marker is_moving return value: {pick_marker_return_value}")

        # place-marker
        place_marker = Gripper.from_robot(machine, "place-marker")
        place_marker_return_value = await place_marker.is_moving()
        print(f"place-marker is_moving return value: {place_marker_return_value}")

        # Note that the following block is commented out because it may actuate
        # or because its argument semantics are unknown. Use with caution.
        # code-1
        # code_1 = GenericService.from_robot(machine, "code-1")
        # code_1_return_value = await code_1.do_command({})
        # print(f"code-1 do_command return value: {code_1_return_value}")

        # effdet-coco
        effdet_coco = MLModelClient.from_robot(machine, "effdet-coco")
        effdet_coco_return_value = await effdet_coco.metadata()
        print(f"effdet-coco metadata return value: {effdet_coco_return_value}")

        # object-detector
        object_detector = VisionClient.from_robot(machine, "object-detector")
        object_detector_return_value = await object_detector.get_properties()
        print(f"object-detector get_properties return value: {object_detector_return_value}")

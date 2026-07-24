"""Record the arm's current configuration as the 'home' pose."""
import asyncio
import json

from connect import connect
from viam.components.arm import Arm


async def main() -> None:
    m = await connect()
    try:
        arm = Arm.from_robot(m, "arm-1")
        jp = await arm.get_joint_positions()
        pose = await arm.get_end_position()
        home = {
            "joints_deg": list(jp.values),
            "end_pose": {
                "x": pose.x, "y": pose.y, "z": pose.z,
                "o_x": pose.o_x, "o_y": pose.o_y, "o_z": pose.o_z,
                "theta": pose.theta,
            },
        }
        with open("home.json", "w") as f:
            json.dump(home, f, indent=2)
        print("Saved home.json:")
        print(json.dumps(home, indent=2))
    finally:
        await m.close()


if __name__ == "__main__":
    asyncio.run(main())

import asyncio
import os

from dotenv import load_dotenv
from viam.robot.client import RobotClient

load_dotenv()


async def connect() -> RobotClient:
    opts = RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"],
        api_key_id=os.environ["VIAM_API_KEY_ID"],
    )
    # Flaky cloud link: don't let a brief client-side disconnect trip the
    # session heartbeat that auto-stops the arm mid-move. Physical E-stop and
    # capped velocity remain the safety nets.
    opts.disable_sessions = True
    return await RobotClient.at_address(os.environ["VIAM_ADDRESS"], opts)


async def main() -> None:
    machine = await connect()
    print("Connected. Resources on the machine:")
    for name in machine.resource_names:
        print(f"  - {name.subtype:30} {name.name}")
    await machine.close()


if __name__ == "__main__":
    asyncio.run(main())

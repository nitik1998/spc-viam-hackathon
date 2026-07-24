"""Pull recent machine logs to see why realsense-camera fails to build."""
import asyncio
import os

from dotenv import load_dotenv
from viam.app.viam_client import ViamClient
from viam.rpc.dial import DialOptions

load_dotenv()
PART_ID = "05ce02d2-a2fe-4ba9-9040-ce039fa09126"


async def main() -> None:
    opts = DialOptions.with_api_key(
        os.environ["VIAM_API_KEY"], os.environ["VIAM_API_KEY_ID"]
    )
    client = await ViamClient.create_from_dial_options(opts)
    try:
        logs = await client.app_client.get_robot_part_logs(PART_ID, num_log_entries=200)
        for e in logs:
            blob = f"{e.level} {e.logger_name} {e.message}"
            low = blob.lower()
            if any(k in low for k in ("realsense", "segment", "color", "detections-to",
                                       "error", "fail", "reconfig", "build")):
                print(f"[{e.level}] {e.logger_name}: {e.message}")
    finally:
        client.close()


if __name__ == "__main__":
    asyncio.run(main())

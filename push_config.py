"""Push a config JSON to robot17-main via the App API. Usage: push_config.py <file>"""
import asyncio
import os
import sys

from dotenv import load_dotenv
from viam.app.viam_client import ViamClient
from viam.rpc.dial import DialOptions
import json

load_dotenv()
PART_ID = "05ce02d2-a2fe-4ba9-9040-ce039fa09126"
PART_NAME = "robot17-main"


async def main() -> None:
    path = sys.argv[1]
    with open(path) as f:
        config = json.load(f)
    opts = DialOptions.with_api_key(
        os.environ["VIAM_API_KEY"], os.environ["VIAM_API_KEY_ID"]
    )
    client = await ViamClient.create_from_dial_options(opts)
    try:
        await client.app_client.update_robot_part(
            robot_part_id=PART_ID, name=PART_NAME, robot_config=config
        )
        c = lambda k: len(config.get(k, []) or [])
        print(f"pushed {path}: {c('components')} components, {c('services')} services, "
              f"{c('modules')} modules")
    finally:
        client.close()


if __name__ == "__main__":
    asyncio.run(main())

"""Read the current machine config via the Viam App API using the existing key."""
import asyncio
import json
import os

from dotenv import load_dotenv
from viam.app.viam_client import ViamClient
from viam.rpc.dial import DialOptions

load_dotenv()
ADDR = os.environ["VIAM_ADDRESS"]
LOC_ID = ADDR.split(".")[1]  # ag9khwy6jn


async def main() -> None:
    opts = DialOptions.with_api_key(
        os.environ["VIAM_API_KEY"], os.environ["VIAM_API_KEY_ID"]
    )
    client = await ViamClient.create_from_dial_options(opts)
    try:
        app = client.app_client
        # find the robot in this location whose part address matches
        robots = await app.list_robots(location_id=LOC_ID)
        print("robots in location:", [(r.id, r.name) for r in robots])
        for r in robots:
            parts = await app.get_robot_parts(r.id)
            for p in parts:
                print(f"robot={r.name} part id={p.id} name={p.name} "
                      f"main={p.main_part} fqdn={p.fqdn}")
                if p.main_part:
                    cfg = dict(p.robot_config)
                    with open("current_config.json", "w") as f:
                        json.dump(cfg, f, indent=2)
                    print(f"  saved current_config.json with keys: {list(cfg.keys())}")
    finally:
        client.close()


if __name__ == "__main__":
    asyncio.run(main())

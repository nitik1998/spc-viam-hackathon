"""Safely recover and move the arm to the saved home configuration.

Clears any xArm error/emergency-stop state first (requirement #3), then homes.
"""
import asyncio

from connect import connect
from safety import recover_and_home


async def main() -> None:
    m = await connect()
    try:
        await recover_and_home(m)
    finally:
        await m.close()


if __name__ == "__main__":
    asyncio.run(main())

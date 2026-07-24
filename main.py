from helpers import connect
import asyncio
import sys


class Sorter:
    def __init__(self, robot):
        self.robot = robot

    async def resources(self):
        """List the resources the machine exposes (a first, safe call)."""
        for name in sorted(rn.name for rn in self.robot.resource_names):
            print(" ", name)


# verb -> method. One entry per capability.
STEPS = {
    "resources": Sorter.resources,
}

async def main(verb):
    robot = await connect()
    sorter = Sorter(robot)
    try:
        step = STEPS.get(verb)
        if step is None:
            print(f"unknown step '{verb}'. steps: {', '.join(STEPS)}")
            return
        await step(sorter)
    finally:
        await robot.close()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"usage: python palletizer.py <step>   steps: {', '.join(STEPS)}")
        sys.exit(1)
    asyncio.run(main(sys.argv[1]))
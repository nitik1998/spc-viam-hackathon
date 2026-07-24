"""Shared safety helpers: clear xArm errors, control speed, and safely home."""
import json

from viam.components.arm import Arm
from viam.proto.component.arm import JointPositions

ARM = "arm-1"


def load_home():
    return json.load(open("home.json"))


async def clear_error(arm: Arm):
    """Clear xArm error/E-stop state and re-enable motion. Safe to call always."""
    try:
        await arm.do_command({"clear_error": True})
        return True
    except Exception as e:
        print(f"clear_error failed (may be fine if no error): {e}")
        return False


async def set_speed(arm: Arm, speed_degs_per_sec: float):
    """Set the arm's joint speed at runtime (for a slow, smooth approach)."""
    try:
        await arm.do_command({"set_speed": float(speed_degs_per_sec)})
    except Exception as e:
        print(f"set_speed({speed_degs_per_sec}) failed: {e}")


async def recover_and_home(m, home=None, arm_name: str = ARM) -> Arm:
    """Requirement #3: on (re)start, clear any error/E-stop and go to home FIRST.

    Returns the Arm client so callers can keep using it.
    """
    if home is None:
        home = load_home()
    arm = Arm.from_robot(m, arm_name)
    await clear_error(arm)
    if await arm.is_moving():
        await arm.stop()
    print("safety: returning to home config first")
    await arm.move_to_joint_positions(JointPositions(values=home["joints_deg"]))
    print("safety: at home")
    return arm

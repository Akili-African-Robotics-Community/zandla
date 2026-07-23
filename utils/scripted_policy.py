import random
import numpy as np
from dm_control.utils import inverse_kinematics


def mc_ik(
    physics,
    target_pos,
    target_quat=None,
    site_name="gripperframe",
    tol=1e-3,
    max_steps=50,
):
    """
    Inverse Kinematics solver wrapper function
    """
    ik_result = inverse_kinematics.qpos_from_site_pose(
        physics=physics,
        site_name=site_name,
        target_pos=target_pos,
        target_quat=target_quat,
        tol=tol,
        max_steps=max_steps,
    )
    return ik_result.qpos[:6].copy()


def generate_push_instruction(direction="left", swap_targets=False):
    """
    Generates randomly chosen language instructions based on the direction and if the targets are swapped.
    - Standard (swap_targets=False): Left -> green target, Right -> blue target
    - Swapped (swap_targets=True): Left -> blue target, Right -> green target

    Args:
        direction: string specifying the direction of motion between left and right to choose the target colour.
        swap_targets: bool
    """
    if not swap_targets:
        target_color = "green" if direction.lower() == "left" else "blue"
    else:
        target_color = "blue" if direction.lower() == "left" else "green"

    direction_words = {
        "left": ["left", "leftward"],
        "right": ["right", "rightward"],
    }
    direction_word = random.choice(direction_words[direction.lower()])

    templates = [
        f"push the red cube {direction_word} to the {target_color} target",
        f"nudge the red block {direction_word} towards the {target_color} marker",
        f"slide the red cube {direction_word} into the {target_color} zone",
        f"move the block {direction_word} into the {target_color} area",
        f"guide the red cube {direction_word} to the {target_color} target",
        f"shove the block {direction_word}, into the {target_color} spot",
    ]

    return random.choice(templates)


def get_next_action(
    obs,
    physics,
    direction="left",
    step_count=0,
    approach_steps=25,
    push_steps=50,
    swap_targets=False,
):
    """
    Generates the next action given the current observations, actions are generated based on the current step counts
    moves the end-effector beside the cube if step_count < 25, and moves the cube to the target otherwise.
    (n.b. steps is the reliable option so far)
    Args:
        obs: dictionary containing the current observations
        physics: dm_control physics object
        direction: string specifying the direction of motion between left and right to choose the target colour.
        step_count: integer representing the current step count
        approach_steps: integer representing the number of steps to move to the point beside the cube
        push_steps: integer representing the number of steps to move the cube to the target destination
        swap_targets: bool specifying if the targets are swapped

    Returns:
        next_action: numpy array of shape (6,) containing action to carried out in the next step
    """
    cube_pos = obs["cube_position"]
    arm_joints = obs["joint_positions"]

    # swaps target location if specified
    if not swap_targets:
        target_pos = (
            obs["target_green_position"]
            if direction.lower() == "left"
            else obs["target_blue_position"]
        )
    else:
        target_pos = (
            obs["target_blue_position"]
            if direction.lower() == "left"
            else obs["target_green_position"]
        )

    # calculation of end-effector position beside the cube
    y_offset = -0.05 if direction.lower() == "left" else 0.05
    wp_beside = np.array([cube_pos[0], cube_pos[1] + y_offset, 0.025])

    # final target position
    wp_destination = np.array([cube_pos[0], target_pos[1], 0.025])

    # Generating action based on step count, if < 25 move beside the cube, else move cube to target
    if step_count < approach_steps:
        target_joints = mc_ik(physics, target_pos=wp_beside, site_name="gripperframe")
        alpha = (step_count + 1) / float(approach_steps)
        action = (1 - alpha) * arm_joints + alpha * target_joints
    else:
        push_step = step_count - approach_steps + 1
        alpha = min(1.0, push_step / float(push_steps))
        current_wp = (1 - alpha) * wp_beside + alpha * wp_destination
        action = mc_ik(physics, target_pos=current_wp, site_name="gripperframe")
    return action

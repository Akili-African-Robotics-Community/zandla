import os
import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from dm_control import mujoco as dmc_mujoco
from dm_control.rl import control
from dm_env import specs
import mujoco.viewer

from zandla.envs import PushCubeGymEnv
from utils.scripted_policy import generate_push_instruction, get_next_action


if __name__ == "__main__":
    print("Initializing PushCube Gym Environment...")
    env = PushCubeGymEnv(render_mode="human")
    # Custom initial cube position to spawn
    init_cube_pos = [0.2, 0.0, 0.015]
    for episode in range(3):
        direction = "left" if episode % 2 == 0 else "right"
        swap_target = False
        instruction = generate_push_instruction(
            direction=direction, swap_targets=swap_target
        )
        options = {
            "instruction": instruction,
            "init_cube_pos": init_cube_pos,
            "swap_target_colors": swap_target,
        }

        obs, info = env.reset(options=options)

        print(f"\n--- Episode {episode + 1} ({direction.upper()}) ---")
        print(f"Instruction : '{obs['instruction']}'")
        print(f"Cube Pos    : {obs['cube_position']}")

        step_count = 0
        total_reward = 0.0

        while True:
            # compute next action based on counts
            # moves the end-effector to beside the cube within 25 steps and moves the cube after (more stable with steps compared to distance based)
            action = get_next_action(
                obs,
                env.unwrapped.physics,
                direction=direction,
                swap_targets=swap_target,
                step_count=step_count,
            )

            # Step the simulation
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            step_count += 1
            time.sleep(0.04)
            if terminated or truncated or step_count >= 100:
                print(
                    f"Episode Finished! Total Steps: {step_count} | Total Reward: {total_reward:.4f} | Success: {info['success']}"
                )
                break
    env.close()
    print("\nTest complete.")

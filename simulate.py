import mujoco
import mujoco.viewer
import time
import os
import numpy as np
import argparse

MODEL_PATH = os.path.join("zandla", "robot_models", "SO101", "scene.xml")


def main():
    parser = argparse.ArgumentParser(description="SO101 MuJoCo Simulation CLI")
    parser.add_argument(
        "--model",
        type=str,
        default=MODEL_PATH,
        help="Path to the MuJoCo scene XML file",
    )
    parser.add_argument(
        "--span",
        type=float,
        default=0.15,
        help="Percentage of the joint range to span (0.0 to 1.0)",
    )
    parser.add_argument(
        "--freq",
        type=float,
        default=2.0,
        help="Frequency of the sinusoidal oscillation in Hz",
    )

    args = parser.parse_args()

    if not os.path.exists(args.model):
        print(f"Error: Could not find model file at {args.model}")
        return

    # Load the model and data
    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)

    # Get joint control ranges (ctrlrange) from the model
    ctrl_ranges = model.actuator_ctrlrange
    ctrl_mid = np.mean(ctrl_ranges, axis=1)
    ctrl_width = (ctrl_ranges[:, 1] - ctrl_ranges[:, 0]) * args.span

    # Logging the ranges
    print(f"\nActuator Control Ranges (Span={args.span*100}%):")
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        orig_min, orig_max = ctrl_ranges[i]
        new_min = ctrl_mid[i] - (ctrl_width[i] / 2.0)
        new_max = ctrl_mid[i] + (ctrl_width[i] / 2.0)
        print(
            f"  {name:15}: [{orig_min:7.3f}, {orig_max:7.3f}] -> [{new_min:7.3f}, {new_max:7.3f}]"
        )
    print("")

    print(f"Launching MuJoCo viewer (Freq={args.freq}Hz)...")
    print("Press 'Space' to pause/unpause, 'Backspace' to reset.")

    # Start the viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # Apply sinusoidal control signal
            elapsed = data.time
            data.ctrl[:] = ctrl_mid + (ctrl_width / 2.0) * np.sin(
                2 * np.pi * args.freq * elapsed
            )

            # Step the simulation
            mujoco.mj_step(model, data)

            # Update the viewer
            viewer.sync()

            # Maintain real-time simulation speed
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()

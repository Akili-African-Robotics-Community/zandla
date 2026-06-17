import mujoco
import mujoco.viewer
import time
import os
import numpy as np

def main():
    # Simulation parameters
    model_path = os.path.join("zandla", "robot_models", "SO101", "scene.xml")
    range_percentage = 0.15  # Span 15% of the joint range
    frequency = 2         # 2 Hz oscillation
    
    if not os.path.exists(model_path):
        print(f"Error: Could not find model file at {model_path}")
        return

    # Load the model and data
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # Get joint control ranges (ctrlrange) from the model
    # model.actuator_ctrlrange is an (nu, 2) array
    ctrl_ranges = model.actuator_ctrlrange
    ctrl_mid = np.mean(ctrl_ranges, axis=1)
    ctrl_width = (ctrl_ranges[:, 1] - ctrl_ranges[:, 0]) * range_percentage
    
    # Logging the ranges
    print(f"\nActuator Control Ranges (Span={range_percentage*100}%):")
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        orig_min, orig_max = ctrl_ranges[i]
        new_min = ctrl_mid[i] - (ctrl_width[i] / 2.0)
        new_max = ctrl_mid[i] + (ctrl_width[i] / 2.0)
        print(f"  {name:15}: [{orig_min:7.3f}, {orig_max:7.3f}] -> [{new_min:7.3f}, {new_max:7.3f}]")
    print("")

    print("Launching MuJoCo viewer...")
    print("Press 'Space' to pause/unpause, 'Backspace' to reset.")
    
    # Start the viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
              # Note: mj_step handles the integration; we just update the target
            elapsed = data.time
            data.ctrl[:] = ctrl_mid + (ctrl_width / 2.0) * np.sin(2 * np.pi * frequency * elapsed)
            
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

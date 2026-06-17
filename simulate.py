import mujoco
import mujoco.viewer
import time
import os

def main():
    # Path to the scene file
    model_path = os.path.join("zandla", "robot_models", "SO101", "scene.xml")
    
    if not os.path.exists(model_path):
        print(f"Error: Could not find model file at {model_path}")
        return

    # Load the model and data
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    print("Launching MuJoCo viewer...")
    print("Press 'Space' to pause/unpause, 'Backspace' to reset.")
    
    # Start the viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
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

from pathlib import Path
import mujoco
import mujoco.viewer
import time
import numpy as np
import json
import os

path = Path(__file__).parent
model = mujoco.MjModel.from_xml_path(str(path / "kbotwscene.xml"))
data = mujoco.MjData(model)

# Reset the simulation data
mujoco.mj_resetData(model, data)

# Set simulation parameters
model.opt.timestep = 0.001  # 1000Hz simulation


applying_force = False
torso_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "KB_B_102B_TORSO_BOTTOM")

def key_callback(key):
    global applying_force
    
    if chr(key) == ' ':
        # Reset position and velocity
        data.qpos = np.zeros(len(data.qpos))
        data.qvel = np.zeros(len(data.qvel))
        print("Reset position and velocity")
    
    elif chr(key) == 'R':
        # Reset to initial state
        mujoco.mj_resetData(model, data)
        print("Reset simulation")
    
    elif chr(key) == 'Z':
        # Toggle force application
        applying_force = not applying_force
        print(f"Applying force: {applying_force}")
    
    elif chr(key) == 'V':
        # Toggle frame visualization
        if viewer.opt.frame == 7:
            viewer.opt.frame = 1
        else:
            viewer.opt.frame = 7
        viewer.sync()
        print("Toggled frame visualization")

def main():
    global applying_force, viewer
    
    # Launch the viewer
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        target_time = time.time()
        sim_time = 0.0
        
        # Create a list to store recorded data
        recorded_data = []
        
        while viewer.is_running():
            # Apply simple control - zero control for now
            data.ctrl = np.zeros(model.nu)
            
            # Apply external force if enabled
            if applying_force:
                # Apply force in x direction to the torso
                data.xfrc_applied[torso_body_id, :3] = [10.0, 0.0, 0.0]
            
            # Record data
            sensor_data = {
                "time": sim_time,
                "base_pos": data.sensor("base_link_pos").data.tolist(),
                "force_sensor": data.sensor("force_sensor").data.tolist(),
                "KB_D_301R_R_Femur_Lower_Drive Torques": data.body("KB_D_301R_R_Femur_Lower_Drive").cfrc_int.tolist()[:3],
                "KB_D_301R_R_Femur_Lower_Drive Forces": data.body("KB_D_301R_R_Femur_Lower_Drive").cfrc_int.tolist()[3:],
                "KB_D_301R_R_Femur_Lower_Drive Force X": [data.body("KB_D_301R_R_Femur_Lower_Drive").cfrc_int.tolist()[3]],
                "KB_D_301R_R_Femur_Lower_Drive Force Y": [data.body("KB_D_301R_R_Femur_Lower_Drive").cfrc_int.tolist()[4]],
                "KB_D_301R_R_Femur_Lower_Drive Force Z": [data.body("KB_D_301R_R_Femur_Lower_Drive").cfrc_int.tolist()[5]]
            }
            recorded_data.append(sensor_data)
            
            # Step simulation
            mujoco.mj_step(model, data)
            sim_time += model.opt.timestep
            
            # Update viewer
            viewer.sync()
            
            # Maintain real-time simulation
            target_time += model.opt.timestep
            current_time = time.time()
            if target_time - current_time > 0:
                time.sleep(target_time - current_time)
        
        # Save recorded data to a JSON file when simulation ends
        with open("data/kbot_data.json", "w") as f:
            json.dump(recorded_data, f, indent=4)
        
        print(f"Data saved to kbot_data.json with {len(recorded_data)} timesteps")

if __name__ == "__main__":
    main()

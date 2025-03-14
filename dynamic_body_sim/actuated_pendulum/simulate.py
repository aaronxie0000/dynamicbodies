import mujoco
import mujoco.viewer
import time
import numpy as np
import json
from pathlib import Path
from dynamic_body_sim.utils.motion_controller import compute_torque


def run_simulation():
    path = Path(__file__).parent
    model = mujoco.MjModel.from_xml_path(str(path / "pendulum.xml"))
    data = mujoco.MjData(model)

    mujoco.mj_resetData(model, data)

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "toplink")
    applying_force = False

    def key_callback(key):
        if chr(key) == ' ':
            data.qpos = np.zeros(len(data.qpos))
            data.qvel = np.zeros(len(data.qvel))
        elif chr(key) == 'R':
            data.qpos[0] = 3
        elif chr(key) == 'Z':
            nonlocal applying_force
            applying_force = not applying_force
            print(f"Applying force: {applying_force}")
        elif chr(key) == 'T':
            if viewer.opt.frame == 7:
                viewer.opt.frame = 1
            else:
                viewer.opt.frame = 7
            viewer.sync()

    model.opt.timestep = 0.001  # 0.001 = 1000hz

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        target_time = time.time()
        # Create a list to store the data
        recorded_data = []
        sim_time = 0.0

        # viewer.opt.frame = True

        while viewer.is_running():
            # data.ctrl = compute_torque(0.0, data.joint("actjoint").qpos, data.joint("actjoint").qvel)
            data.ctrl = -5

            if applying_force:
                data.xfrc_applied[body_id, :3] = [0.0, 0.0, 50.0]

            forceAtorque = np.zeros(6)
            # Get the ground geom ID using the correct name "floor"
            floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")


            for i, con in enumerate(data.contact):
                # Check if one of the geoms is the ground/floor
                is_ground_contact = (con.geom1 == floor_id or con.geom2 == floor_id)
                
                if is_ground_contact:
                    mujoco.mj_contactForce(model, data, i, forceAtorque)
                    
                    # According to the documentation:
                    # - The contact frame X axis is the contact normal (stored in frame[0-2])
                    # - The Y and Z axes are tangential (stored in frame[3-5] and frame[6-8])
                    # - forceAtorque[0] is the force along the X axis (normal)
                    # - forceAtorque[1-2] are forces along Y and Z axes (tangential)
                    
                    # Get the contact frame axes
                    normal_axis = con.frame[0:3]  # X axis (normal)
                    tangent_y_axis = con.frame[3:6]  # Y axis (first tangential direction)
                    tangent_z_axis = con.frame[6:9]  # Z axis (second tangential direction)
                    
                    print("---contact force---")
                    print(f"  Contact position: {con.pos}")
                    print(f"  Contact normal axis: {normal_axis}")
                    print(f"  Contact tangent Y axis: {tangent_y_axis}")
                    print(f"  Contact tangent Z axis: {tangent_z_axis}")
                    print(f"  Normal force (along X axis): {forceAtorque[0]:.4f}")
                    print(f"  Tangential forces (along Y,Z axes): [{forceAtorque[1]:.4f}, {forceAtorque[2]:.4f}]")
                    print(f"  Torques: [{forceAtorque[3]:.4f}, {forceAtorque[4]:.4f}, {forceAtorque[5]:.4f}]")
                    
                    # Create the contact frame rotation matrix (3x3)
                    # Note: MuJoCo stores the frame in transposed form, so we're already working with the transpose
                    contact_frame = np.array([
                        normal_axis,       # First row: X axis (normal)
                        tangent_y_axis,    # Second row: Y axis
                        tangent_z_axis     # Third row: Z axis
                    ])
                    
                    # Create the force vector in contact frame
                    force_contact = np.array([forceAtorque[0], forceAtorque[1], forceAtorque[2]])
                    
                    # Convert force from contact frame to world frame
                    # For vectors (like forces), we only need to rotate, not translate
                    # Since contact_frame is already transposed, we multiply directly
                    force_world = contact_frame.T @ force_contact
                    
                    print(f"  Force in world coordinates: {force_world}")
                    print()

            # Record data before stepping the simulation
            force_data = {
                "time": sim_time,
                # "internal_force_toplink": data.body("toplink").cfrc_int.tolist(),
                # "external_force_toplink": data.body("toplink").cfrc_ext.tolist(),
                "internal_force_bottomlink": data.body("bottomlink").cfrc_int.tolist(),
                "external_force_bottomlink": data.body("bottomlink").cfrc_ext.tolist(),
                "Contact Force Normal": [forceAtorque[0]],
                "Contact Force Tangential": forceAtorque[1:3].tolist(),
                "Contact Force Torque": forceAtorque[3:6].tolist(),
                # "force_sensor": data.sensor("force_sensor").data.tolist(),
            }


            recorded_data.append(force_data)

            
            # if sim_time > 5.0:
            #     print("internal force top link: ", data.body("toplink").cfrc_int)
            #     print("external force top link: ", data.body("toplink").cfrc_ext)
            #     print("internal force bottom link: ", data.body("bottomlink").cfrc_int)
            #     print("external force bottom link: ", data.body("bottomlink").cfrc_ext)
            #     print("force sensor: ", data.sensor("force_sensor").data.tolist())
            #     print("body masses: ", model.body_mass)
            #     breakpoint()


            # Step simulation
            mujoco.mj_step(model, data)
            sim_time += model.opt.timestep
            viewer.sync()

            target_time += model.opt.timestep
            current_time = time.time()
            if target_time - current_time > 0:
                time.sleep(target_time - current_time)
        
        # Save the recorded data to a JSON file when simulation ends
        with open("force_data.json", "w") as f:
            json.dump(recorded_data, f, indent=4)
        
        print(f"Data saved to force_data.json with {len(recorded_data)} timesteps")


if __name__ == "__main__":
    run_simulation()




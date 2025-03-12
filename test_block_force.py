import mujoco
import mujoco.viewer
import time
import numpy as np
import json
from motion_controller import compute_torque


model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetData(model, data)

def key_callback(key):
    if chr(key) == ' ':
        data.qpos = np.zeros(len(data.qpos))
        data.qvel = np.zeros(len(data.qvel))
    elif chr(key) == 'R':
        data.qpos[0] = 3


model.opt.timestep = 0.001 # 0.001 = 1000hz


with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    target_time = time.time()
    # Create a list to store the data
    recorded_data = []
    contact_force = np.zeros((6,1), dtype=np.float64)
    sim_time = 0.0

    viewer.opt.frame = True

    while viewer.is_running():
        # data.ctrl = compute_torque(3.14, data.joint("actjoint").qpos, data.joint("actjoint").qvel)
        data.ctrl = -10


        # for j, c in enumerate(data.contact):
            # print(j)

        # Record data before stepping the simulation
        force_data = {
            "time": sim_time,
            "force_sensor": data.sensor("force_sensor").data.tolist(),
            "internal_force": data.body("toplink").cfrc_int.tolist(),
            "external_force": data.body("toplink").cfrc_ext.tolist(),
            "contact_force": contact_force.tolist(),
            "qfrc_constraint": data.qfrc_constraint.tolist(),
            "qfrc_contraint2": data.efc_force.tolist(),
            "qfrc_applied": data.qfrc_applied.tolist(),
            "passive_force": data.qfrc_passive.tolist(),
            "actuator_force": data.qfrc_actuator.tolist(),
            "applied_force": (data.qfrc_passive + data.qfrc_actuator + data.qfrc_applied).tolist(),
            "bias_force": data.qfrc_bias.tolist(),
        }
        recorded_data.append(force_data)
        
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




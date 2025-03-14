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
            data.ctrl = 5

            if applying_force:
                data.xfrc_applied[body_id, :3] = [10.0, 0.0, 0.0]

            # Record data before stepping the simulation
            force_data = {
                "time": sim_time,
                "internal_force_toplink": data.body("toplink").cfrc_int.tolist(),
                "external_force_toplink": data.body("toplink").cfrc_ext.tolist(),
                "cfrc_ext": data.cfrc_ext.tolist(),
                "internal_force_bottomlink": data.body("bottomlink").cfrc_int.tolist(),
                "external_force_bottomlink": data.body("bottomlink").cfrc_ext.tolist(),
                "force_sensor": data.sensor("force_sensor").data.tolist(),
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
        with open("data/pendulum_data.json", "w") as f:
            json.dump(recorded_data, f, indent=4)
        
        print(f"Data saved to force_data.json with {len(recorded_data)} timesteps")


if __name__ == "__main__":
    run_simulation()




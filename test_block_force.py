import mujoco
import mujoco.viewer
import time
import numpy as np
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

# breakpoint()

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    target_time = time.time()
    while viewer.is_running():

        # data.ctrl = compute_torque(3.14, data.joint("actjoint").qpos, data.joint("actjoint").qvel)
        data.ctrl = -10
        # data.cfrc_ext[0]
        mujoco.mj_step(model, data)
        viewer.sync()

        print(data.body("toplink").cfrc_int)
        print(data.body("toplink").cfrc_ext)
        
        target_time += model.opt.timestep
        current_time = time.time()
        if target_time - current_time > 0:
            time.sleep(target_time - current_time)




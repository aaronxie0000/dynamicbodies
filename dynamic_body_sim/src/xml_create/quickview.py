import mujoco
import mujoco.viewer
import time


model = mujoco.MjModel.from_xml_path("dynamic_body_sim/utils/xml_create/kbotv2p0/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetData(model, data)

model.opt.timestep = 0.001  # 0.001 = 1000hz

# Disable gravity
# model.opt.gravity[2] = 0



with mujoco.viewer.launch_passive(model, data) as viewer:

    target_time = time.time()
    sim_time = 0.0

    while viewer.is_running():

        # Step simulation
        mujoco.mj_step(model, data)
        sim_time += model.opt.timestep
        viewer.sync()

        target_time += model.opt.timestep
        current_time = time.time()
        if target_time - current_time > 0:
            time.sleep(target_time - current_time)




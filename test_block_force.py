import mujoco
import mujoco.viewer
import numpy as np
import time

# Load the scene file which includes both the robot and floor
model = mujoco.MjModel.from_xml_path("simplerobot/scene.xml")
data = mujoco.MjData(model)

# mujoco.mj_resetData(model, data)

# mujoco.mj_resetDataKeyframe(model, data, 0)

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()


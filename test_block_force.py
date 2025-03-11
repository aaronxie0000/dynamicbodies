import mujoco
import mujoco.viewer
import numpy as np
import time

# Load the scene file which includes both the robot and floor
model = mujoco.MjModel.from_xml_path("simplerobot/scene.xml")
data = mujoco.MjData(model)

# mujoco.mj_resetData(model, data)

# mujoco.mj_resetDataKeyframe(model, data, 0)

# Apply a control to the 'knee' joint to make it rotate
# Ensure the control value is within the actuator's control range
control_value = 1.0  # Adjust the control value as needed
data.ctrl[model.actuator('knee_actuator').id] = control_value

print(f"Applying control: {control_value}")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        # Debugging output to check joint position and control
        print(f"Joint position: {data.qpos[model.joint('knee').qposadr]}")
        print(f"Control input: {data.ctrl[model.actuator('knee_actuator').id]}")


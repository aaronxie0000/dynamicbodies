import mujoco
import mujoco.viewer

# Load the MJCF file
model = mujoco.MjModel.from_xml_path("kbot-v2/robot/robot.mjcf")
data = mujoco.MjData(model)

# Enable gravity (default is [0, 0, -9.81])
model.opt.gravity = [0, 0, -9.81]

# Create a viewer and render the model
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Simulate and render
    while viewer.is_running():
        # Step the physics
        mujoco.mj_step(model, data)

        print(data.time)
        print(len(data.qpos))
        # Update the viewer
        viewer.sync()
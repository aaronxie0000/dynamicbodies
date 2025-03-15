import numpy as np
from pathlib import Path
from dynamic_body_sim.core.simulate import run_simulation as run_mujoco_simulation
from dynamic_body_sim.core.simulate import mujoco
from dynamic_body_sim.core.motion_controller import compute_torque


def pendulum_control_function(data, model):
    """
    Control function for the pendulum.
    """
    return compute_torque(-np.pi/0.75, data.joint("actjoint").qpos, data.joint("actjoint").qvel)
    # return -5  # Simple constant torque

def pendulum_data_recorder(data, model, sim_time):
    """
    Record data for the pendulum simulation.
    """
    return {
        "time": sim_time,
        "internal_force_toplink": data.body("toplink").cfrc_int.tolist(),
        "external_force_toplink": data.body("toplink").cfrc_ext.tolist(),
        "internal_force_bottomlink": data.body("bottomlink").cfrc_int.tolist(),
        "external_force_bottomlink": data.body("bottomlink").cfrc_ext.tolist(),
        "force_sensor": data.sensor("force_sensor").data.tolist(),
    }

def run_simulation():
    path = Path(__file__).parent
    model_path = path / "pendulum.xml"
    output_file = path/ "data/pendulum_data.json"
    
    # Define custom key actions
    def reset_joint():
        # This function will be called when the space key is pressed
        # We need to get the model and data from the current simulation
        import inspect
        frame = inspect.currentframe()
        while frame:
            if 'data' in frame.f_locals and 'model' in frame.f_locals:
                data = frame.f_locals['data']
                model = frame.f_locals['model']
                data.qpos[0] = 0  # Reset joint angle
                data.qpos[1] = 2  # Set height of bottomlink to 2 meters
                data.qvel[:] = 0  # Reset velocities
                print("Reset joint position and velocity")
                break
            frame = frame.f_back
    
    custom_key_actions = {
        ' ': reset_joint
    }
    
    run_mujoco_simulation(
        model_path=model_path,
        output_file=output_file,
        force_body_name="bottomlink",
        force_magnitude=[100.0, 0.0, 0.0],
        timestep=0.001,
        data_recorder=pendulum_data_recorder,
        custom_key_actions=custom_key_actions,
        control_function=pendulum_control_function
    )


if __name__ == "__main__":
    run_simulation()




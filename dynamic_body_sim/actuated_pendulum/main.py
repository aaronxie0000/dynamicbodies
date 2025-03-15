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
    
    run_mujoco_simulation(
        model_path=model_path,
        output_file=output_file,
        force_body_name="bottomlink",
        force_magnitude=[100.0, 0.0, 0.0],
        timestep=0.001,
        data_recorder=pendulum_data_recorder,
        control_function=pendulum_control_function
    )


if __name__ == "__main__":
    run_simulation()




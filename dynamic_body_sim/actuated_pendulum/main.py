import numpy as np
from pathlib import Path
from dynamic_body_sim.dynamic_body_sim.simulate import force_simulation
from dynamic_body_sim.dynamic_body_sim.simulate import mujoco
from dynamic_body_sim.dynamic_body_sim.motion_controller import compute_torque


def pendulum_control_function(data, model):
    """
    Control function for the pendulum.
    """
    return compute_torque(-np.pi/0.75, data.joint("actjoint").qpos, data.joint("actjoint").qvel)
    # return -5  # Simple constant torque

def main():
    path = Path(__file__).parent
    model_path = path / "pendulum.xml"
    output_file = path/ "data/pendulum_data.json"
    
    force_simulation(
        model_path=model_path,
        output_file=output_file,
        force_body_name="bottomlink",
        force_magnitude=[100.0, 0.0, 0.0],
        body_queried="toplink",
        control_function=pendulum_control_function,
        additional_queries={
            "internal_force_bottomlink": "data.body('bottomlink').cfrc_int.tolist()",
            "external_force_bottomlink": "data.body('bottomlink').cfrc_ext.tolist()",
        }
    )


if __name__ == "__main__":
    main()
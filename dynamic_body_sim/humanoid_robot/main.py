from pathlib import Path
import numpy as np
from dynamic_body_sim.core.simulate import force_simulation

def humanoid_control_function(data, model):
    """
    Control function for the humanoid robot.
    """
    return np.zeros(model.nu)

def main():
    path = Path(__file__).parent
    model_path = path / "kbotv2p0/scene.xml"
    output_file = path / "data/kbot_data.json"

    additional_data = {
        "base_pos (m)": "data.sensor('base_link_pos').data.tolist()",
        # "base_quat": "data.sensor('base_link_quat').data.tolist()",
        # "base_vel": "data.sensor('base_link_vel').data.tolist()",
        # "base_ang_vel": "data.sensor('base_link_ang_vel').data.tolist()",
        # "imu_acc": "data.sensor('imu_acc').data.tolist()",
        # "imu_gyro": "data.sensor('imu_gyro').data.tolist()",
    }

    force_simulation(
        model_path=model_path,
        output_file=output_file,
        force_body_name="kc_d_401l_l_shin_drive",
        force_magnitude=[0.0, 0.0, 100.0],
        body_queried="kc_d_302r_femur_lower_idle",
        control_function=humanoid_control_function,
        additional_queries=additional_data
    )

if __name__ == "__main__":
    main()
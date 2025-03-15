from pathlib import Path
import numpy as np
from dynamic_body_sim.core.simulate import run_simulation, mujoco

def humanoid_control_function(data, model):
    """
    Control function for the humanoid robot.
    """
    return np.zeros(model.nu)

def humanoid_data_recorder(data, model, sim_time):
    """
    Record data for the humanoid robot simulation.
    """
    return {
        "time": sim_time,
        "base_pos": data.sensor("base_link_pos").data.tolist(),
        # "base_quat": data.sensor("base_link_quat").data.tolist(),
        # "base_vel": data.sensor("base_link_vel").data.tolist(),
        # "base_ang_vel": data.sensor("base_link_ang_vel").data.tolist(),
        # "imu_acc": data.sensor("imu_acc").data.tolist(),
        # "imu_gyro": data.sensor("imu_gyro").data.tolist(),
        "force_sensor": data.sensor("force_sensor").data.tolist(),

        "kc_d_302r_femur_lower_idle Torques": data.body("kc_d_302r_femur_lower_idle").cfrc_int.tolist()[:3],
        "kc_d_302r_femur_lower_idle Forces": data.body("kc_d_302r_femur_lower_idle").cfrc_int.tolist()[3:],
        "kc_d_302r_femur_lower_idle Force X": [data.body("kc_d_302r_femur_lower_idle").cfrc_int.tolist()[3]],
        "kc_d_302r_femur_lower_idle Force Y": [data.body("kc_d_302r_femur_lower_idle").cfrc_int.tolist()[4]],
        "kc_d_302r_femur_lower_idle Force Z": [data.body("kc_d_302r_femur_lower_idle").cfrc_int.tolist()[5]]
    }

def humanoid_viewer_setup(viewer):
    """
    Set up the viewer for the humanoid robot simulation.
    """
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_COM] = True

def main():
    path = Path(__file__).parent
    model_path = path / "kbotv2p0/scene.xml"
    output_file = path / "data/kbot_data.json"
    
    run_simulation(
        model_path=model_path,
        output_file=output_file,
        force_body_name="kc_d_401l_l_shin_drive",
        force_magnitude=[20.0, 0.0, 0.0],
        timestep=0.001,
        data_recorder=humanoid_data_recorder,
        control_function=humanoid_control_function,
        viewer_setup=humanoid_viewer_setup
    )

if __name__ == "__main__":
    main()

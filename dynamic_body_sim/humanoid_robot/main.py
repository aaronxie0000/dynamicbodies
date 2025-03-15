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
        "force_sensor": data.sensor("force_sensor").data.tolist(),
        "KB_D_301R_R_Femur_Lower_Drive Torques": data.body("KB_D_301R_R_Femur_Lower_Drive").cfrc_int.tolist()[:3],
        "KB_D_301R_R_Femur_Lower_Drive Forces": data.body("KB_D_301R_R_Femur_Lower_Drive").cfrc_int.tolist()[3:],
        "KB_D_301R_R_Femur_Lower_Drive Force X": [data.body("KB_D_301R_R_Femur_Lower_Drive").cfrc_int.tolist()[3]],
        "KB_D_301R_R_Femur_Lower_Drive Force Y": [data.body("KB_D_301R_R_Femur_Lower_Drive").cfrc_int.tolist()[4]],
        "KB_D_301R_R_Femur_Lower_Drive Force Z": [data.body("KB_D_301R_R_Femur_Lower_Drive").cfrc_int.tolist()[5]]
    }

def humanoid_viewer_setup(viewer):
    """
    Set up the viewer for the humanoid robot simulation.
    """
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_COM] = True

def main():
    path = Path(__file__).parent
    model_path = path / "kbotwscene.xml"
    output_file = path / "data/kbot_data.json"
    
    # Define custom key actions
    def set_height():
        # This function will be called when the space key is pressed
        # We need to get the model and data from the current simulation
        # This is a bit of a hack, but it works for this example
        import inspect
        frame = inspect.currentframe()
        while frame:
            if 'data' in frame.f_locals and 'model' in frame.f_locals:
                data = frame.f_locals['data']
                model = frame.f_locals['model']
                data.qpos[2] = 2.0  # The 3rd element controls vertical position
                mujoco.mj_forward(model, data)  # Update simulation state
                print("Set position to a height")
                break
            frame = frame.f_back
    
    custom_key_actions = {
        ' ': set_height
    }
    
    run_simulation(
        model_path=model_path,
        output_file=output_file,
        force_body_name="KB_B_102B_TORSO_BOTTOM",
        force_magnitude=[10.0, 0.0, 0.0],
        timestep=0.001,
        data_recorder=humanoid_data_recorder,
        custom_key_actions=custom_key_actions,
        control_function=humanoid_control_function,
        viewer_setup=humanoid_viewer_setup
    )

if __name__ == "__main__":
    main()

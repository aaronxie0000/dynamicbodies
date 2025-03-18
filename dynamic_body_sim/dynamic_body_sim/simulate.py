import mujoco
import mujoco.viewer
import time
import numpy as np
import json
from pathlib import Path
import os
from dynamic_body_sim.dynamic_body_sim.utils import setup_logger

# Setup logger using the utility function
logger = setup_logger(__name__)

def load_model(xml_path):
    """
    Load a MuJoCo model from an XML file.
    
    Args:
        xml_path (str or Path): Path to the XML file
        
    Returns:
        tuple: (model, data) MuJoCo model and data objects
    """
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    
    # Reset the simulation data
    mujoco.mj_resetData(model, data)
    
    return model, data


def create_key_callback(data, model, applying_force_ref, viewer_ref):
    """
    Create a key callback function for the MuJoCo viewer.
    
    Args:
        data: MuJoCo data object
        model: MuJoCo model object
        applying_force_ref: List containing a boolean indicating if force is being applied
        viewer_ref: List that will contain the viewer object
        custom_actions: Dictionary of custom key actions specific to the simulation
        
    Returns:
        function: Key callback function
    """
    def key_callback(key):
        char_key = chr(key)
        
        # Common actions
        if char_key == 'R':
            # Reset to initial state
            mujoco.mj_resetData(model, data)
            logger.info("Reset simulation")
        elif char_key == 'Z':
            # Toggle force application
            applying_force_ref[0] = not applying_force_ref[0]
            logger.info(f"Applying force: {applying_force_ref[0]}")
        elif char_key == 'V':
            # Toggle frame visualization
            if viewer_ref[0].opt.frame == 7:
                viewer_ref[0].opt.frame = 1
            else:
                viewer_ref[0].opt.frame = 7
            viewer_ref[0].sync()
            logger.info("Toggled frame visualization")
        elif char_key == ' ':
            if len(data.qpos) >= 3: 
                data.qpos[2] = 2  # Set height of root to 2 meters
                data.qvel[:] = 0  # Reset velocities
                logger.info("Drop from height")
            else:
                logger.warning("Root link is not free")
    
    return key_callback

def check_force_interaction(model, body_queried):
    """
    Goes through kinematic tree and based on which body is being queried,
    returns the two bodies of which's interaction force is being recorded. 

    Args:
        body_queried (str): Name of the body to check
        xml_path (str): Path to the XML file
    """
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_queried)
    parent_id = model.body_parentid[body_id]
    parent_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, parent_id)

    logger.info(f"Force measured between: --- {body_queried} --- AND --- {parent_name} ---")



def force_simulation(
    model_path,
    output_file,
    force_body_name=None,
    force_magnitude=[10.0, 0.0, 0.0],
    body_queried="world",
    force_sensor_name="force_sensor",
    control_function=None,
    additional_queries={}
):
    """
    Run a MuJoCo simulation with the given parameters.
    
    Args:
        model_path (str or Path): Path to the XML model file
        output_file (str): Path to save the recorded data
        force_body_name (str, optional): Name of the body to apply force to
        force_magnitude (list, optional): Force vector to apply [x, y, z]
        timestep (float, optional): Simulation timestep
        body_queried (str, optional): Name of the body to check force interaction
        control_function (function, optional): Function to compute control signals
        additional_queries (dict, optional): Dictionary of additional data to record
    """
    # Get the absolute path to the model file
    path = Path(model_path)
    model, data = load_model(path)
    
    # Set simulation parameters
    model.opt.timestep = 0.001 # 1000 hz
    
    # Set up force application
    applying_force = [False]  # Use a list to allow modification in the callback
    apply_force_body = None
    if force_body_name:
        apply_force_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, force_body_name)
    
    # Create a reference for the viewer
    viewer_ref = [None]
    
    # Create key callback
    key_cb = create_key_callback(data, model, applying_force, viewer_ref)
    
    # Launch the viewer
    with mujoco.viewer.launch_passive(model, data, key_callback=key_cb) as viewer:
        viewer_ref[0] = viewer
        
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_COM] = True
        
        target_time = time.time()
        sim_time = 0.0
        
        # Create a list to store recorded data
        recorded_data = []
        
        while viewer.is_running():
            # Apply control if provided
            if control_function:
                data.ctrl = control_function(data, model)

            # Apply external force if enabled
            if applying_force[0] and apply_force_body is not None:
                data.xfrc_applied[apply_force_body, :3] = force_magnitude

            if sim_time < 0.001:
                check_force_interaction(model, body_queried)
            
            sensor_data = {
                "time": sim_time,
                "force_sensor (N)": data.sensor(force_sensor_name).data.tolist(),
                f"{body_queried} Torques (Nm)": data.body(body_queried).cfrc_int.tolist()[:3],
                f"{body_queried} Forces (N)": data.body(body_queried).cfrc_int.tolist()[3:],
                f"{body_queried} Force X (N)": [data.body(body_queried).cfrc_int.tolist()[3]],
                f"{body_queried} Force Y (N)": [data.body(body_queried).cfrc_int.tolist()[4]],
                f"{body_queried} Force Z (N)": [data.body(body_queried).cfrc_int.tolist()[5]]
            }

            # Add additional data fields
            for key, expression in additional_queries.items():
                sensor_data[key] = eval(expression)

            recorded_data.append(sensor_data)
            
            # Step simulation
            mujoco.mj_step(model, data)
            sim_time += model.opt.timestep
            
            # Update viewer
            viewer.sync()
            
            # Maintain real-time simulation
            target_time += model.opt.timestep
            current_time = time.time()
            if target_time - current_time > 0:
                time.sleep(target_time - current_time)
        
        # Save recorded data to a JSON file when simulation ends
        if recorded_data and output_file:
            # Ensure directory exists
            output_dir = os.path.dirname(output_file)
            if output_dir:  # Only create directory if there is one specified
                os.makedirs(output_dir, exist_ok=True)
            with open(output_file, "w") as f:
                json.dump(recorded_data, f, indent=4)
            
            logger.info(f"Data saved to {output_file} with {len(recorded_data)} timesteps")

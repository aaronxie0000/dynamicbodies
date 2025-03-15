# Dynamic Body Simulation

This repository contains tools for simulating dynamic bodies using MuJoCo.

## Installation

```bash
pip install -e .
```

## Usage

The repository contains two main examples:

1. **Humanoid Robot**: A simulation of a humanoid robot with force sensors.
2. **Actuated Pendulum**: A simulation of a pendulum with an actuated joint.

### Running the simulations

On macOS, you need to use `mjpython` to run the simulations:

```bash
# Run the humanoid robot simulation
mjpython -m dynamic_body_sim.humanoid_robot.main

# Run the actuated pendulum simulation
mjpython -m dynamic_body_sim.actuated_pendulum.main
```

On other platforms, you can use regular Python:

```bash
python -m dynamic_body_sim.humanoid_robot.main
python -m dynamic_body_sim.actuated_pendulum.main
```

### Key Controls

Both simulations support the following key controls:

- `R`: Reset the simulation
- `Z`: Toggle force application
- `V`: Toggle frame visualization
- `Space`: Custom action (varies by simulation)
  - Humanoid Robot: Set position to a height of 2 meters
  - Actuated Pendulum: Reset joint angle and set height to 2 meters

## Creating Your Own Simulation

You can create your own simulation by using the `run_simulation` function from the `dynamic_body_sim.src.simulate` module:

```python
from pathlib import Path
from dynamic_body_sim.src.simulate import run_simulation, mujoco

def my_control_function(data, model):
    # Implement your control logic here
    return my_control_signal

def my_data_recorder(data, model, sim_time):
    # Record data at each timestep
    return {
        "time": sim_time,
        "my_sensor": data.sensor("my_sensor").data.tolist(),
        # Add more data to record
    }

def main():
    path = Path(__file__).parent
    model_path = path / "my_model.xml"
    output_file = "my_data.json"
    
    # Define custom key actions
    def my_custom_action():
        # Implement your custom action here
        pass
    
    custom_key_actions = {
        ' ': my_custom_action
    }
    
    run_simulation(
        model_path=model_path,
        output_file=output_file,
        force_body_name="my_body",
        force_magnitude=[10.0, 0.0, 0.0],
        timestep=0.001,
        data_recorder=my_data_recorder,
        custom_key_actions=custom_key_actions,
        control_function=my_control_function
    )

if __name__ == "__main__":
    main()
```

## API Reference

### `run_simulation`

```python
run_simulation(
    model_path,
    output_file,
    force_body_name=None,
    force_magnitude=[10.0, 0.0, 0.0],
    timestep=0.001,
    data_recorder=None,
    custom_key_actions=None,
    control_function=None,
    viewer_setup=None
)
```

- `model_path`: Path to the XML model file
- `output_file`: Path to save the recorded data
- `force_body_name`: Name of the body to apply force to
- `force_magnitude`: Force vector to apply [x, y, z]
- `timestep`: Simulation timestep
- `data_recorder`: Function to record data at each timestep
- `custom_key_actions`: Dictionary of custom key actions
- `control_function`: Function to compute control signals
- `viewer_setup`: Function to set up the viewer 
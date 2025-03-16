# Dynamic Body Simulation

Basic actuated pendulum example in MuJoCo for inital dev (and later testing) & scripting basic actions for robot and reading internal forces on links for use in FEA.



### Running the Actuated Pendulum Simulation

Uses entrypoints, defined in `setup.py`

```bash
pip install -e .

# Run example simulations
dybs-examples --body humanoid
dybs-examples --body pend

# Plot force data
dybs-plot --body pend  # For pendulum data
dybs-plot --body humanoid  # For humanoid data

# Plot filtered/averaged force data
dybs-filterplot --body pend  # For pendulum data
dybs-filterplot --body humanoid # For humanoid data
```


### Keyboard Controls

While the simulation is running, you can use the following keyboard controls:

- **Space**: Drop from height of 2m
- **R**: Reset the Simulation
- **Z**: Toggle applying force to the top link
- **V**: Toggle frame visualization mode between body and global

# Dynamic Body Simulation

Basic actuated pendulum example in MuJoCo for inital dev (and later testing) & scripting basic actions for robot and reading internal forces on links for use in FEA.



### Running the Actuated Pendulum Simulation

```bash
python -m actauted_pendulum.test_block_force
```

### Keyboard Controls

While the simulation is running, you can use the following keyboard controls:

- **Space**: Reset the pendulum position and velocity to zero
- **R**: Raise the pendulum 
- **Z**: Toggle applying force to the top link
- **T**: Toggle frame visualization mode


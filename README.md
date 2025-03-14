# Dynamic Body Simulation

Basic actuated pendulum example in MuJoCo for inital dev (and later testing) & scripting basic actions for robot and reading internal forces on links for use in FEA.



### Running the Actuated Pendulum Simulation

```bash
mjpython -m actauted_pendulum.test_block_force
```

### Keyboard Controls

While the simulation is running, you can use the following keyboard controls:

- **Space**: Reset the pendulum position and velocity to zero
- **R**: Raise the pendulum 
- **Z**: Toggle applying force to the top link
- **T**: Toggle frame visualization mode


### Development

Remember (aaron) to `pip install -e '.[dev]' to install the dev requirements along with this package. The `-e` flag makes it editable! Which means changes you make to the package python files will be automatically updated in your package. So you can just `import dynamic_body_sim` and it will always be the most up-to-date code~

### Contributing

Make sure everything is formatted with `make format` and pass the static checks with `make static-checks`
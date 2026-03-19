# lander-project
MATLAB
Thrust Controller comparison via a graphical representation of a physics simulation.
command_lander is the main file and calls attached function files. The project is a very simple 1-D lander simulation provided a set of initial paremeter structs that displays graphs of position, velocity, thrust, and mass over time for different controller variants (Bang-Bang Controller, Proportional-Derivative Controller, Energy Removal Controller).

1. Thrust Controller functions are created (bang_controller, PD_controller, and energy_controller) and take in simulation parameters and return discrete thrust to be encoded at array index correspondent to active timestep in simulate_lander.
2. command_lander takes in two structs to define physics simulation and graphs kinematic results (Z,V,Thrust,M).
3. simulate_lander performs physics simulation

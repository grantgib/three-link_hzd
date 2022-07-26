# three-link_hzd

Hybrid Zero Dynamics Optimization and Simulation for a 3-link walker.

Author: Grant Gibson (2018)

Based on this the book: [Feedback Control of Dynamic Bipedal Robot Locomotion](https://github.com/grantgib/three-link_hzd/tree/master/Reference)

## How to Use
* Run the `main_gait_design_fmincon.m` script to compute optimal gait parameters given the equality/inequality constraints in `Optimization_Gait_Design/Cost_Torque.m`.
* Copy and paste the solution into either `main_simulation_3link_1step.m` or `main_simulation_3link_Nsteps.m` as the variable X0 and you will see a simulation and plot of the results.

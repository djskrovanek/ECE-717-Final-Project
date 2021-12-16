# ECE-717-Final-Project
Dynamics of SEM in wind turbine

How to use:
1. Scale pu parameters from reference machine to wind turbine in ScaleParameters.m
2. Per unitize parameters in CalcBaseVals.m
3. Set operating point based on system parameters in CalcEquilibrium.m
4. Use linearizeMatrix.m to get LTI state space model
5. Use checkSystemProperties.m to check for controllability, observability, etc...
6. designPcontroller.m and designPIcontroller.m create controllers based on input poles to be placed
7. simOpenLoop.m, simClosedLoopPcontroller.m, and simClosedLoopPIcontroller.m simulate and plot outputs as their names describe.
   simLTI, and simNL are helper functions which implement the numerical integrations for each of the above scripts.




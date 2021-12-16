# ECE-717-Final-Project
Dynamics of SEM in wind turbine

How to use:
1. Scale pu parameters from reference machine to wind turbine in ScaleParameters
2. Per unitize parameters in CalcBaseVals
3. Set operating point based on system parameters in CalcEquilibrium
4. Use linearizeMatrix to get LTI state space model
5. Use checkSystemProperties to check for controllability, observability, etc...
6. designPcontroller and designPIcontroller create controllers based on input poles to be placed
7. simOpenLoop, simClosedLoopPcontroller, simClosedLoopPIcontroller, simLTI, and simNL simulate and plot outputs as their names describe.

# Model_Predictive_Control_with_Obstacle_Avoidance
The four robots head for their respective destination values. At that time, control is performed to avoid collisions with other aircraft.

![multiagent_control](https://github.com/Ramune6110/Model_Predictive_Control_with_Obstacle_Avoidance/blob/main/figures/multiagent_control.gif)

# Procedure
## STEP1
To use IPOPT solver with YALMIP, unzip [the ipopt zip](https://github.com/Ramune6110/Model_Predictive_Control_with_Obstacle_Avoidance/tree/main/packages) file of packeges and define the path to the folder in the matlab script file.

## STEP2
Clone this repository to your local environment and run [four_mobile_robot_with_range_specification.m](https://github.com/Ramune6110/Model_Predictive_Control_with_Obstacle_Avoidance/blob/main/four_mobile_robot_with_range_specification.m)
Then, a collision avoidance simulation of four robots will run as shown above.

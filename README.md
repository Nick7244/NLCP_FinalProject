# UR5 Box Sorting Robot
See the UR5 Box Sorting Robot in action:
- https://www.youtube.com/watch?v=3Oi9Dar5aA8


Instructions to run:
- terminal 1
    - roslaunch ur5_box_gazebo ur5_box.launch 
- terminal 2
    - roslaunch ur5_box_gazebo box_plugin.launch 
    - then: rosrun rtt_ros deployer -s src/UR5-Box-Sorting-Robot/ur5_box_control/scripts/ur5_control.ops 

- orocos commands:
    - cd robot
    - TPose
    - ExecuteTrajectory
    - GenerateTrajectory
    - ExecuteTrajectory
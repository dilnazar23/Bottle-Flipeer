# Demo_moveit_ur

This package provides a demonstration of the some of the utilities inside the moveit package. Move it basic and avoid demonstrate the movements, while constrained allows the user to constrain the end effector, and servo allows the user to jog the end effector. While this provides an overview of basic modes other areas of interest which aren't demonstrated here include, hybrid planning, carteasan planning, alternative planning algorithms, and attaching objects. Additional information and tutorials can be found at:
https://moveit.picknik.ai/humble/doc/tutorials/your_first_project/your_first_project.html

Refer to moveit_servo_ur to see how a moveit group interface can be instanciated within a class. 
Refer to moveit_constain to see how to use the built in moveit visualisation tools.

Nodes included:
* moveit_basic_ur
* moveit_avoid_ur
* moveit_constrain_ur
* moveit_servo_ur

All nodes should be launched with the respective launch files, this is due to them requiring the universal robotics parameters. 

Before launching any of the following nodes ensure the fake or real robot has been set up correctly.

### Launching moveit basic
*NOTE this should only be done with FAKE hardware, otherwise the real robot may run into the table*
Moves between two points 20 times to demonstrate basic planning and movement

    ros2 launch demo_moveit_ur moveit_basic_launch.py 

### Launching moveit avoid
Moves between two points 20 times to demonstrate basic planning and movement, while avoiding any part of the arm touching table and wall planes. 

    ros2 launch demo_moveit_ur moveit_avoid_launch.py 


### Launching moveit constrain
Constrained path planning allows the user to constrain the end effector within a box, plane or line. 

To display the visualisation array add the topic '/rviz_visual_tools/MarkerArray' in rviz.

When running this node it will first perform a unconstrained movement to point 1. It will then generate point 2 at (x+=0,y+=0.3,z+=0.3) and a large box between the points. It will then perform a constrained motion in which the end effector stays within the box. 

    ros2 launch demo_moveit_ur moveit_constrain_launch.py 


*Notes about moveit constrain:*
 * If not enough time is given a valid path may not be found. 
 * Constraned planes and lines may not work reliabiliy or as intended, this is currently being worked on.




### Launching moveit servo
Moveit servo allows the user to jog the joints and end effectors relative to the frame. 

Before activating move it servo two seperate service calls must be made. 
 * The first service call is to activate the servo node
 * The second is to switch from the currently active joint_trajectory_controller to a forward_position_controller.

These service calls can be made manually however they are included in the demo already.

### Running the demo 

First launch fake hardware using the '4231_scripts'

    ./setupFakeur5e.sh 

By default the arm is near a singularity, set the goal state to 'test_configuration' then plan and execute. Alternatively this could be done through a moveit node. 

Afterwards run the command:

    ros2 launch demo_moveit_ur moveit_servo_launch.py 

This demo makes the required service calls then publishes 100 movement commands. It does not switch the controllers back.
Make sure to switch the controller back to a joint controller afterwards, otherwise basic moveit controls won't execute. 


 #### Moveit servo CLI calls

To activate servo node through CLI:

    ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}

To list controllers through CLI:

    ros2 control list_controllers

To switch controllers through CLI:

    ros2 control switch_controllers --activate forward_position_controller --deactivate joint_trajectory_controller

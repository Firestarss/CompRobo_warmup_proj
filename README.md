# CompRobo_warmup_proj

## Rosbag ##

The goal for this section was to record manual teleop control into a bag file and then play back those commands from the recorded bag. One of the challenges I am running into is that the Neato does not follow the exact path as the teleop. I'm assuming this is due to the bag running in real-time and the gazebo simulation not running at the same speed. I currently do not know how to fix this discrepency.

Neato following Rosbag instructions:

![Teleop Rosbag](assets/teleop_bag.gif)

## Robot Teleop ##

The goal for this section was to program our own teleop controller for the Neato. I used a control scheme where the "WASD" keys were used to increment/decrement the linear and angular velocities of the Neato. These velocities were then sent to the Neato via Twist messages on the `/cmd_msg` topic.

TODO: add to setup.py

The Teleop program in action:

![Teleop](assets/teleop.gif)
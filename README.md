Package for [Robotics for Programmers](https://www.manning.com/books/robotics-for-programmers) book providing a Python nodes that
* `traj1d_generator`: publish a 1D trajectory as turtlesim/Pose (to be combined with turtlesim)
* `turtle_teleporter`: subscribe to the 1D trajectory (turtlesim/Pose) and teleports turtesim turtles there (using the turtesim teleport service)
* `traj1d_differentiator`: subscribe to the 1D trajectory (turtlesim/Pose) and publish derivatives, velocity as geometry_msgs/Twist and acceleration as geometry_msgs/Accel (to be plotted via rqt_plot or plotjuggler)
* `twist_integrator`: integrate a geometry_msgs/Twist topic and publish a geometry_msgs/Pose topic

Used in chapter 6.
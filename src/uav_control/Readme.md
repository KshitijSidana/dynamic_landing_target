## Launch 

It is possible that when running the script an error appears saying:

Resource not found: px4 ROS path [0] = ... ...

This means that PX4 SITL was not included in the path. To solve this add these lines at the end of the .bashrc file:
```
source ~/PX4-Autopilot/Tools/simulation/gazebo/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins
```
Now in the terminal, go to the home directory and run the following command to apply the changes above to the current terminal:

```
source .bashrc
```
After this step, every time you open a new terminal window you should not have to worry about this error anymore. If it appears again, a simple source .bashrc should fix it. This solution was obtained from this issue (opens new window)thread, where you can get more information about the problem.
```
roslaunch uav_control_py start_uav_control.launch
```
This should fire up the simulation in Gazebo


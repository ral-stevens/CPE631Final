# CPE631_final_sample, Human Robot Interaction Environment
### Requirements:
* Ubuntu 18.04
* ROS – Melodic
* The simulation environment is not guaranteed to work with any other ROS or Ubuntu environment.
### Pre-requisites:
1. C++11 compiler which is basically gcc/g++.
2. Gazebo-ROS packages, which should come with a Desktop-full installation of ROS Melodic.
### Running Instructions:
1. Suppose you have made a catkin workspace named `catkin_ws`.
1. Once you are finished with all the dependencies, you can git clone this package from github into your ROS workspace and `catkin_make` it. 
    ```shell
    cd ~/catkin_ws/src
    git clone --recurse-submodules https://github.com/ral-stevens/CPE631Final.git
    cd ~/catkin_ws
    catkin_make
    ```
1. If you installed all dependencies correctly, you shouldn’t get any errors.
1. You can run the environment by executing
    ```shell
    roslaunch pedsim_simulator Altorfer_f1.launch
    ```
    and then you should be able to see something as follows.

    ![Alt Text](other/sample.gif)

1. Once you successfully run the node, you will see topic related to laser range finder, Kinect camera and controlling the velocity of the robot. You can subscribe to and publish on these topics in your own controller to get the sensor information and publish the desired velocities to move the robot.
* Linear and angular velocity topic:
    ```shell
    /Pioneer3AT/cmd_vel
    ```
* Laser Range finder topic:
    ```shell
    /laser/scan
    ```
* Pose of each model in the world:
    ```shell
    /gazebo/model_states
    ```
* Kinect topics (There are several topics, you can choose the ones you need):
    ```shell
    /camera/*
    ```
## Reporting Issues
If something is not working or you have any questions, please report it with a [GitHub Issue](https://github.com/ral-stevens/CPE631Final/issues).

## Maintainer
* [Zhuo Chen](zchenpds@gmail.com)



## Acknowledgement
Muhammad Fahad made this simulation based on [pedsim_ros](https://github.com/srl-freiburg/pedsim_ros).

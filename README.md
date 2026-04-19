


# Trefoil trajectory

This package is called: panda_trefoil_trajectory. The aim is to make a simulation of the Franka Emika Panda robot to move and follow a Trefoil Knot trajectory.

## How to run

Be sure to have installed ROS 1, MoveIt, RViz, Gazebo and [Franka Control Interface for ROS](https://frankarobotics.github.io/docs/franka_ros.html). Then, follow the steps to create and build a catkin workspace by following the instructions that can be found on the [ROS tutorials](https://wiki.ros.org/ROS/Tutorials) webpage. Download this package and move its folder into the src folder of the catkin workspace.

------------

**Terminal 1**

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch panda_moveit_config demo_gazebo.launch
```

Only after both RViz, Gazebo, and MoveIt are correctly launched, execute the commands of the second terminal.

**Terminal 2 - Trajectory execution**

```
cd ~/catkin_ws
source devel/setup.bash
rosrun panda_heart_trajectory heart_trajectory
```

**Terminal 3 - Recording the topics using ROS bags**

In this terminal we will create a specifi folder for the rosbags to be saved and then we will run the commands necessary to record the topics during the execution. After the recording is finished, we will execute the commands to convert the bag file into .csv files:

```
rosbag record \
/franka_state_controller/joint_states \
/franka_state_controller/franka_states \
-O franka_execution.bag
```

Press ``Ctrl+C`` to stop recording (only after the execution is ended).

```
rostopic echo -b franka_execution.bag -p /franka_state_controller/joint_states > joints_data.csv
rostopic echo -b franka_execution.bag -p /franka_state_controller/franka_states > robot_data.csv
```

## Data Analysis

The scripts named: ``EE_pos.py`` and ``Joints_evolution.py`` analyze data written in the csv files created above. Do not modify manually those csv files. To see plots and data about the execution of the trajectory, run these python scripts.

**Note:** it is not necessary to run these files using ``rosrun`` commands. Use any tool you prefer.

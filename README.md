# Walker algorithm for Turtlebot3 using ROS2

ROS2 beginner walker algorithm implementation in gazebo assignment for ENPM808X course at University of Maryland.

## Dependencies

- rclcpp
- sensor_msgs
- geometry_msgs
- OS: Ubuntu Linux 22.04
- ROS Version: ROS2 Humble Hawksbill

## Build Instructions

Navigate to the source folder of the ROS2 workspace

```sh
cd ~/ros2_ws/src
```

Clone the GitHub repository

```sh
https://github.com/jayprajapati009/walker_algo.git
```

Now to build the package go to the root of the ROS2 workspace

```sh
cd ~/ros2_ws
```

check the dependencies

```sh
rosdep install -i --from-path src --rosdistro humble -y
```

and build the package

```sh
colcon build --packages-select walker_algo
```

## Run Instructions

After the successful build, to run open a new terminal,

```sh
cd ~/ros2_ws
```

```sh
. install/setup.bash
```

### Using the launch file

To run the launch file and start the ros bag file recording,

```sh
cd ~/ros2_ws/src/walker_algo/results/bag_files
```

```sh
ros2 launch walker_algo walker_launch.py bag_record:=True
```
  
## Cppcheck and Cpplint

To run the Cpplint and the cppcheck command and save the results in the results directory,

```sh
sh cppcheck_cpplint.sh
```

## References

[1] <http://docs.ros.org/en/humble/index.html>

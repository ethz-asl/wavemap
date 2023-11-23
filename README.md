# Current progress
- kiss-icp file structure:
```
cmake_agnostic_root/
|-- ros/
|   |-- ros1/
|   |-- ros2/
|   |-- CMakeLists.txt
|-- wavemap/
|   |-- core/
|   |-- io/
|   |-- CMakeLists.txt
```
- it builds in both catkin and colcon

# How to build
## ROS 1
```bash
./build1.sh
./run1.sh
catkin build
```
## ROS 2
```bash
./build2.sh
./run2.sh
colcon build
```
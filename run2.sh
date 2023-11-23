docker run -it --rm \
    --name ros2_colcon \
    -v .:/root/ros_ws/src/wavemap_agnostic \
    ros2_colcon:latest
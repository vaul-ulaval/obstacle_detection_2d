# Obstacle detection 2D ros2 base package

This package was generated with the following command :
```sh
ros2 pkg create --build-type ament_python obstacle_detection --node-name obstacle_detection
```

The node from our existing obstacle detection solution is located [here](./obstacle_detection_lidar.cpp). It can be usefull to see what are the subscribtions and publishers that you will need. I got this file from this [ros2 package](https://github.com/vaul-ulaval/obstacles_detection), you could build it to compare with your solution.

**VSCode terminal:**
```bash
    # Only before running for the first time
    colcon build --packages-select obstacle_detection --symlink
```
**Terminal 1:**
```bash
    # Launch the foxglove bridge
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```
**Foxglove:**
```
    Open connection
    Foxglove WebSocket
    ws://localhost:8765
```
**Terminal 2:**
```bash
    # Play the ros bag
    source install/setup.bash
    ros2 bag play blitz_obstacle_detection
```
**Terminal 3:**
```bash
    # Run the obstacle detection node
    source install/setup.bash
    ros2 run obstacle_detection obstacle_detection
```
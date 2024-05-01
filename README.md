# Human-robot Interaction

## Dependencies 

Please install the nav2 stack and slam-toolbox:
```bash
sudo apt install ros-humble-tf2-tools ros-humble-tf2-py ros-humble-tf-transformations ros-humble-tf2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-moveit ros-humble-nav2-simple-commander
```

Also install python packages:
```bash
# pip3 install opencv-python
sudo pip3 install transforms3d
```

## Usage

Then build with colcon:
```bash
colcon build --symlink-install
source install/setup.bash
```

To run the navigation stack please use:
```bash
# The stack
ros2 launch tidybot_navigation slam_stack.launch.py

# List params
ros2 launch tidybot_navigation slam_stack.launch.py --show-args
```


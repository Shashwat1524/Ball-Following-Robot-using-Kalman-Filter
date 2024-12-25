# Ball-Following-Robot-using-Kalman-Filter

## Requirements

- [ROS 2 (Humble recommended)](https://docs.ros.org/en/humble/Installation.html)
- [TurtleBot3 packages]([https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)) installed (`turtlebot3`, `turtlebot3_simulations`)
- [Gazebo simulation environment](https://gazebosim.org/)
- [RViz](https://docs.ros.org/en/rolling/Tutorials/Using-RViz-with-SLAM.html) for visualization

**Note** : I am using Turtlebot3 Waffle
## Installation

1. Clone this repository into your ROS 2 workspace:
   ```bash
   git clone https://github.com/Shashwat1524/Ball-Following-Robot-using-Kalman-Filter.git
   ```

2. Build your workspace

   ```bash
     colcon build
   ```
   
3. Source  your workspace
   ```bash
   source install/setup.bash
   ```
   
## Usage

### Step 1: Launch the TurtleBot3 simulation
Run the following command to launch Gazebo and RViz for the TurtleBot3 simulation:

```bash
ros2 launch turtlebot3_gazebo navigator.launch.py
```
#### Add a Red Ball in Gazebo

### Step2 : Execute the following command to start following a red ball in your environment:

```bash
ros2 run ball_follower red_ball_tracker
```


## Demonstration
### Simulation
![Ball_Tracking](ball_tracking.gif)

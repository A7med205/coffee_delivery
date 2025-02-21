## Description

The coffee_delivery package automates the handing of coffee cups from the counter to the barista delivery robot, using computer vision for guidance.

The cup slots need to be within the URe3 robot's reach and should be visible by the RGB-D camera.

---
## Main Dependencies

```
  ros-humble-joint-state-publisher
  ros-humble-robot-state-publisher
  ros-humble-joint-state-publisher-gui
  ros-humble-controller-manager
  ros-humble-ur-description
  ros-humble-xacro
  ros-humble-moveit
  ros-humble-urdf
  python3
  python3-opencv
  python3-rosdep
  python3-pip
  ros-humble-cv-bridge
  ros-humble-rosbridge-server
  numpy 1.24.* ### last numpy version below 1.25
```
Note: you can run rosdep after cloning the package to identify any missing dependencies 

---
## Start-up

- Clone repository inside /ros2_ws/src
- Build and source your workspace

1. **Moveit launch** 
```
ros2 launch moveit2_scripts moveit_bringup.launch.py //real robot
ros2 launch moveit2_scripts moveit_bringup_sim.launch.py //simulation
```

2. **Manipulation API and perception nodes**
```
ros2 launch moveit2_scripts detect_pick_place.launch.py
```

3. **Webapp launch**
```
ros2 launch moveit2_scripts web.launch.py
```

**Notes**
- Rosbridge is launched using port 9090
- Webpage is launched using port 7000
- Perception node hasn't been optimized for the simulation, you can send the web application dummy coordinates while the simulation is running:
```
ros2 topic pub -1 /display_ moveit2_scripts/msg/DisplayPos "{data: [-0.404, -0.005, -0.338, -0.044, -0.337, 0.066, -0.468, -0.039, -0.468, 0.069]}"
```

---
## Interface Usage

**Detection button:**
- Sends a request to the perception node and receives back an array of coordinates
- Slots in the web app appear with the same orientation as the camera and are clickable
- Only available slots are visible
- Control buttons won't work without sending coordinates first
- Coordinates are reset after every sequence that ends with returning to home
![](https://github.com/A7med205/coffee_delivery/blob/main/media/Detection.gif)

**Home button:**
- The home button opens the gripper and returns to the home position
- It's a good idea to enforce the home position at start-up

**Incremental Mode:**
- Incremental button completes the sequence one step at a time
![](https://github.com/A7med205/coffee_delivery/blob/main/media/Incremental.gif)

**Automatic Mode:**
- Automatic mode completes the pick and place sequence from start to finish after selecting a slot
- 
**Stop Button:**
- The stop button has a dedicated subscriber running on a separate thread in the manipulation node
- Doesn't interrupt step, only switches to incremental mode

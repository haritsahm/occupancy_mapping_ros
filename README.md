# occupanc_mapping_rosaOccupancy Grid Mapping - Mobile Robot Laser
---

This is my implementation on occupancy grid mapping from laser data using TurtleBo3 Burger in Gazebo

Usage:
---
### 1. From terminal on directory

```
catkin_make
source devel/setup.bash
```

### 2. Run Gazebo
```
roslaunch custom_world custom_world.launch
```

### 3. Run RVIZ Viewer
```
roslaunch occupancy_mapping map_viewer.launch 
```

### 4. Run Occupancy Mapping
```
roslaunch occupancy_mapping occupancy_map.launch 
```

### 5. Control Using Teleopkey
```
roslaunch turtlebot3_teleopkey turtlebot3_teleop_key.launch 
```



Youtube Video:
--
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/oZfU163FHj0/0.jpg)](https://www.youtube.com/watch?v=oZfU163FHj0)

### Requirements

    1. Qt 5.9
    2. YAML
    3. C++11

### Todo List

- [x] Mapping
- [ ] Odometry Update
- [ ] SLAM



Feel free to ask me, I will be very happy to discuss and learn from others. m.haritsah@mail.ugm.ac.id

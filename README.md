# Multi-Map Navigation and wormholes implementation 
## Objective
 To implement a multi-map navigation system using ROS where a robot can navigate between separately mapped rooms via "wormholes." The system includes mapping, SQL-based wormhole management, and an action server in C++.

## Step 1: Mapping and Wormhole Creation 
### 1.1 Map Each Room Separately

In this step, we map each room individually using SLAM so the robot can later navigate through them independently. We're using TurtleBot3 in the Gazebo house environment and the GMapping SLAM method.

#### 1. Launch the Gazebo Simulation Environment
```sh
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

- This launches the simulated house environment with the TurtleBot3 robot.
- The house consists of multiple rooms where each one will be mapped separately.

#### 2. Start the SLAM Node (GMapping)
```sh
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping 
```

- This runs the SLAM algorithm (gmapping) to build a map in real time as the robot moves.
- Make sure slam_methods:=gmapping is specified to use GMapping over Cartographer or others.

#### 3. Control the Robot Manually
```sh
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
- This launches the teleoperation node that lets you control the robot using your keyboard.
- Use this to move the robot around only one room at a time, thoroughly covering the area.

#### 4. Save the Map

rosrun map_server map_saver -f /home/adi/catkin_ws/src/maps/room1

This saves the map as:
/home/adi/catkin_ws/src/maps/room1.pgm
/home/adi/catkin_ws/src/maps/room1.yaml

Repeat for Each Room
1. Restart the simulation or teleport the robot to another room.
2. Repeat steps 2 to 4 for each room.
3. Save each map with a unique name.

### 1.2 Define Wormholes (Overlapping Regions)

After mapping each room separately, the robot needs a way to "teleport" or transition from one map (room) to another. This is done using wormholes, which are overlapping regions that act like portals between maps.

These wormholes allow you to:
- Seamlessly connect isolated maps
- Store fixed entry/exit points between rooms
- Later switch maps and re-localize the robot at a specific position

#### 1. Launch Navigation in One Map

roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/adi/catkin_ws/src/multi_map_navigation/maps/room1.yaml
This will:
  - Load the map into RViz
  - Start AMCL (Adaptive Monte Carlo Localization)
  - Let the robot localize and navigate within room1

#### 2. Use 2D Pose Estimate Tool in RViz
In RViz, select the "2D Pose Estimate" tool from the top menu.
Click on the point where you want the wormhole to exist in this map (e.g., a doorway).
When you do this:
- The robot will update its estimated pose
- You’ll see a green arrow (indicating the pose)

#### 3. Check the Pose on /amcl_pose Topic
rostopic echo /amcl_pose

```
    header: 
    seq: 0
    stamp: 
        secs: 15
        nsecs:   4000000
    frame_id: "map"
    pose: 
    pose: 
        position: 
        x: -0.042289046423793504
        y: 0.14762854738174702
        z: 0.0
        orientation: 
        x: 0.0
        y: 0.0
        z: 0.014152998453804659
        w: 0.9998998413015009

    Position (x, y): The coordinates of the wormhole in the current map
    For example
    room1 | room2 | 1.53 | 0.53 
```

## Step 2: SQL Database Integration

 ### 2.1 Open Terminal and Start SQLite

  sqlite3 wormholes.db

 - This opens or creates a file named wormholes.db in your current directory.
 - You are now in the interactive SQLite prompt (sqlite>).

### 2.2 Create the Wormholes Table

At the sqlite> prompt, run:

```
CREATE TABLE wormholes (
    from_map TEXT,  -- Source map name
    to_map TEXT,    -- Destination map name
    from_x REAL,    -- X-coordinate in source map
    from_y REAL     -- Y-coordinate in source map
);
```
This creates a table with:
- from_map: name of the map the robot is leaving
- to_map: name of the map the robot is going to
- from_x, from_y: position of the wormhole in the source map

### 2.3 Insert Wormhole Entries

Add wormhole positions using the INSERT INTO SQL command.
At the sqlite> prompt, enter:

``` 
INSERT INTO wormholes VALUES ('room1', 'room2', 1.90, 0.40);
INSERT INTO wormholes VALUES ('room2', 'room1', 1.90, 0.45);
INSERT INTO wormholes VALUES ('room2', 'room3', 6.10, -0.18);
INSERT INTO wormholes VALUES ('room3', 'room2', 6.10, -0.18);
```

- These entries define bidirectional "wormholes" between rooms. Each entry tells your navigation system:
- "If I am in from_map, and want to go to to_map, teleport me to from_x, from_y in the current map."

### 2.4 View All Entries (Verify)

To confirm entries are added:
```SELECT * FROM wormholes;```

### 2.5 Exit SQLite
```.quit```

## Step 3: C++ Code Implementation

### 3.1 File Structure
```
multi_map_navigation/
├── include/multi_map_navigation/
│   ├── map_switcher.h
│   ├── wormhole_manager.h
│   └── navigation_server.h
├── src/
│   ├── map_switcher.cpp
│   ├── wormhole_manager.cpp
│   └── navigation_server.cpp

```
### 3.2 WormholeManager
- Connects to the SQLite database
- Retrieves wormhole coordinates between maps
- Handles queries to find paths between maps

### 3.3 MapSwitcher
- Loads map YAML files from the specified directory
- Launches map_server with the appropriate map
- Manages the transition between different environments

### 3.4 NavigationServer

- Implements ROS action server for handling navigation goals
- Orchestrates the entire navigation process
- Implements navigation logic for direct and indirect paths
- Utilizes move_base for actual robot movement

## Step 4: Action Server Implementation
### 4.1 Action Definition (NavigateToGoal.action)

```
# action/NavigateToGoal.action
float64 target_x
float64 target_y
string target_map
---
bool success
string message
---
string feedback_msg

```

### 4.2 C++ Action Server Logic

- If target_map == current_map:
    Forward goal to move_base.
- Else:
    Find a path through wormholes.
    Navigate to the wormhole.
    Switch maps.
    Teleport robot.
    Continue navigation.

## Step 5: Launch Configuration

```
<launch>
  <node pkg="multi_map_navigation" type="navigation_server" name="navigation_server" output="screen">
    <param name="wormhole_db_path" value="$(find multi_map_navigation)/database/wormholes.db" />
    <param name="map_folder" value="$(find multi_map_navigation)/maps" />
  </node>
</launch>

```

## How to run 

#### 1. Clone the Package into Your Catkin Workspace:
```sh
cd ~/catkin_ws/src
git clone https://github.com/adinathkale01/multi_map_navigation.git
```

#### 2. Build the Workspace

```sh
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

#### 3.Launch the Gazebo Simulation Environment 
```sh
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

#### 4.Launching TurtleBot3 Navigation
```sh
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/adi/tb3_house_map.yaml
```

#### 5. Launch the navigation server:

```sh
roslaunch multi_map_navigation multi_map_navigation.launch
```

#### 6. Send a navigation goal: You can send a goal using an action client, or use a custom script like:

You can send navigation goals using the ROS action client or directly with rostopic:
```sh
rostopic pub /navigate_to_goal/goal multi_map_nav/NavigateToGoalActionGoal "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, goal_id: {stamp: {secs: 0, nsecs: 0}, id: ''}, goal: {target_x: 5.23, target_y: 3.0, target_map: 'room2'}}"
```
Using Python Action Client:

```py
  #!/usr/bin/env python3
  import rospy
  import actionlib
  from multi_map_nav.msg import NavigateToGoalAction, NavigateToGoalGoal

  def send_goal():
      client = actionlib.SimpleActionClient('navigate_to_goal', NavigateToGoalAction)
      client.wait_for_server()

      goal = NavigateToGoalGoal()
      goal.target_map = "room1"
      goal.target_x = -2.6
      goal.target_y = 4.8

      client.send_goal(goal)
      client.wait_for_result()

      return client.get_result()

  if __name__ == '__main__':
      rospy.init_node('navigation_client')
      result = send_goal()
      print("Result:", result.success, result.message)
  ```

### 🎥 Demo

<div style="text-align: center;">
    <a href="https://youtu.be/QrFsZ149RVE">
      <img src="multi_map_navigation/output.png" alt="demo moving from room1 to room2" width="800" height="450">
     </a>     
</div>
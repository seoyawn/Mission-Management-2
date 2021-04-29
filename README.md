Mission Management ROS package
=====================================================================

How to build Mission Management ros package
=====================================================================
    1) Clone this project to your catkin's workspace src folder
    2) Run catkin_make 

How to run Mission Management ros package
=====================================================================
There're two functionalities in mission_management ros package

I. GMapping, Localization (AMCL) and Navigation
------------------------------------------------------------
roslaunch mission_management minimal_dashgo.launch 
-- initiates hardware components of dashgo d1 or smart

roslaunch mission_management minimal_scout.launch 
-- initiates hardware components of scout agilex 2.0

roslaunch mission_management gmapping_dashgo_imu.launch
-- launches gmapping for dashgo d1 or smart with imu
-- dashgo_d1 pkg required

roslaunch mission_management gmapping_scout.launch
-- launches gmapping for scout agilex 2.0
-- scout_ros, agx_sdk pkg required

roslaunch mission_management navigation_dashgo.launch
-- amcl localization with initial pose for dashgo d1 or smart
-- you can start navigation with this launch file

roslaunch mission_management navigation_scout.launch
-- amcl localization with initial pose for scout agilex 2.0
-- you can start navigation with this launch file

You should see result in the rviz.

II. Autonomous Navigation with Mission_Management
------------------------------------------------------------
roslaunch mission_management navigation_dashgo.launch
roslaunch mission_management navigation_scout.launch
-- launches mission management nodes to allow autonomous navigation
-- To view existing waypoints, path and missions, refer to database folder


Available ROS Services
=====================================================================
Waypoint_Manager

1. Add A Waypoint
- rosservice call /mission_manager/add_waypoint + tab tab

2. Remove A Waypoint
- rosservice call /mission_manager/remove_waypoint + tab tab

3. Remove All Waypoint
- rosservice call /mission_manager/remove_all_waypoints + tab tab

4. Get A Waypoint
- rosservice call /mission_manager/get_waypoint + tab tab

5. Get All Waypoints
- rosservice call /mission_manager/get_all_waypoints + tab tab

Mission_Manager

1. Add A Mission
- rosservice call /mission_manager/add_a_mission + tab tab

2. Remove A Mission
- rosservice call /mission_manager/remove_a_mission + tab tab

3. Remove All Mission
- rosservice call /mission_manager/remove_all_missions + tab tab

4. Retrieve A Mission
- rosservice call /mission_manager/retrieve_a_mission + tab tab

5. Retrieve All Missions
- rosservice call /mission_manager/retrieve_all_missions + tab tab

Follow_Path (Autonomous Navigation)

1. Send A Goal
- rosservice call /follow_path/send_a_goal + tab tab (waypoint)

2. Send A Mission
- rosservice call /follow_path/send_a_mission + tab tab (mission)























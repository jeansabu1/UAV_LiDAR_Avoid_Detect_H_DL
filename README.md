# UAV_LiDAR_Avoid_Detect_H_DL
Simulation of UAV obstacle avoidance by using an on-board LiDAR and target (landing pad) recognition by using deep learning (DL).

Scenario: A quadrotor automatically take off and fly to a given waypoint avoiding obstacles in the flight path by using onboard LiDAR sensor. When the quadrotor approaches to the waypoint, it searches the landing pad represented by the character, 'H' by using an onboard camera. A youtube video is avaialble at:
https://youtu.be/UDUXXEQqkv8 
Note the blue circle around the quadrotor in the video represents the lidar sensing points. 


Robot operating system (ROS) communicates with ArduPilot and Gazebo: ArduPiot is a low-cost open-source autopilot software package applicable to various unmanned systems, and Gazebo is a graphical physics simulator used in robotics. In this work, ArduPilot executes low level control commands, for example, take-off and waypoint tracking. Gazebo provides graphical objects including obstacles, the quadrotor, and landing pad based on their physical models. ROS administrates the simulation by using multiple packages. For example, ROS packages generates local path planning by processing onboard LiDAR data, and generates/receives commands and data with ArduPilot and Gazebo. A simple landing pad identification by using DL is tested at the end of the simulation. For this task, a CNN model classifying alphabets are trained on handwritten-alphabet datasets (downloaded from https://www.kaggle.com/sachinpatel21/az-handwritten-alphabets-in-csv-format.) To keep the problem as a simple classification problem, I implemented openCV codes to localize each character, then a sub-image centered at the target location was fed to the CNN model. (Note DL object detection algorithm can do localization and classification at the same time, but it needs a lot more datasets).


ROS packages and nodes used in this work include:
1)  hector_quadrotor package: a quadrotor simulation package including modeling and control (http://wiki.ros.org/hector_quadrotor). 
2)  mavros package : enables communication between ROS and an autopilot based on MAVLink: (http://wiki.ros.org/mavros).
3)  Autonomy package (C++): communicates with ArduPilot and Gazebo simulator, process lidar data, and .
generate local path planning.
4)  EO_sensing node (Python): localize characters in an image by using openCV, load trained CNN model, classify each localized character, and identify the character, 'H'.

**How to run the simulation (roughly)**

Terminal 1) [turning on the autopilot]\
$ sim_vehicle.py -j4 -v ArduCopter --console

Terminal 2) [load the quadrotor, obstacles, and the landing pad with Gazebo]\
$ roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch

Terminal 3) [establish communication link between ROS and Ardupilot SITL]\
$ roslaunch autonomy sitl_copter.launch     

Terminal 4) [Send commands (e.g. take-off) to Ardupilot and control inputs (e.g., velocity) to Hector_quadrotor]\
$ rosrun autonomy mavlink_handler

Terminal 5) [Process lidar data and generate local path planning]\
$ rosrun autonomy vfh_local_planning

== Wait until the quadroter reaches close to a given waypoint ==

Terminal 6) [localize characters, load CNN model, identify 'H']\
$ rosrun autonomy EO_sensing.py



#FLYING AR-DRONE USING TAGS

#### Made by Or Hanoch
#### Project Manager: Shunmuga Pillay
#### Proffesional Adviser: Pravesh Ranchod

## ABOUT
The goal of this project is to have an AR-Drone fly and obtain instructions from markers (aka tags) on the wall in order to direct it. The code is built in such a way that the movement is not hard coded, but the instruction of each tag is - thus you can change the placements of the tags on the walls and without changing the code the drone should change its movement accordingly.

In order to obtain this goal we use ROS Kinetic in addition to some open-source github repositories.
Tested on Ubuntu 16.04.

NOTE:
The main package with my code is tags_instructing_drones. Other packages included here are packages which I used and may have edited slightly, thus it may be easier to just copy them from here rather than copy the original repositories and try to edit them to what I have.

## Preperation Instructions
It is recommended to go through the ROS tutorials and install ROS Kinetic through them:
http://wiki.ros.org/ROS/Tutorials/

After installing ROS Kinetic

Put this directory in `~/catkin_ws/src`.

The following repositories need to be downloaded and installed:

https://github.com/AutonomyLab/ardrone_autonomy
https://github.com/sniekum/ar_track_alvar
|
---	ardrone_autonomy installation instructions and wiki:
|	http://ardrone-autonomy.readthedocs.io/en/latest/installation.html
|
---	ar_track_alvar installation instructions and wiki:
	http://wiki.ros.org/ar_track_alvar

tum simulator and gazeebo - optional:
gazeebo - http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install
tum simulator - https://github.com/angelsantamaria/tum_simulator
|
--- tum simulator and gazeebo installation instructions and wiki:
|	provided in README.md of git repository aswell as http://wiki.ros.org/tum_simulator
|	
--- I added some tags to my worlds in gazeebo, as such I recommend copying my tum_simulator folder into ~/catkin_ws/src
|
--- You can use the models supplied in the gazeebo_models folder to make new gazeebo worlds of your own. These came from:
	https://github.com/mikaelarguedas/gazebo_models

At this point you should compile catkin:
`catkin_make`

## Running Instructions
Real Drone:
	- Connecting to drone: 
		`roslaunch tags_instructing_drone launch_drone.launch`
	- Starting ar_track_alvar: 
		`roslaunch tags_instructing_drone pr2_indiv_no_kinect_edited.launch`
	- View drone camera output in real time: 
		`rosrun image_view image_view image:=/ardrone/front/image_raw`
	- Run code and create log file: 
		rosrun tags_instructing_drone real_drone_flight.py | tee ~/catkin_ws/src/tags_instructing_drone/scripts/logs/real_drone_flight_$(date +%Y%m%d%H%M%S).log
	- NOTE: In order to debug effectivly I found it useful to have "Simple Screen Recorder" capture the screen with the code running aswell as the drone camera image.
	- Kill switch in case something goes wrong:
		`rostopic pub -1 /ardrone/land std_msgs/Empty`
	
Gazeebo Simulator:
	- Run simulator: roslaunch tags_instructing_drone test_simulator_tags4.launch
	- View drone camera output in real time: 
		`rosrun image_view image_view image:=/ardrone/front/image_raw`
	- Run code and create log file: 
		rosrun tags_instructing_drone real_drone_flight.py | tee ~/catkin_ws/src/tags_instructing_drone/scripts/logs/real_drone_flight_$(date +%Y%m%d%H%M%S).log
		
## Useful Sources
- ROS Tutorials:
http://wiki.ros.org/ROS/Tutorials/

- tum_simulator wiki has some commands you can control the drone with directly from terminal:
http://wiki.ros.org/tum_simulator

- Launch file came from:
https://edu.gaitech.hk/drones/ar_parrot_2/ar-parrot-2-ros.html


## Licenses
This Project is released under GPL v.3 which can be found in the license file.

Different reposetories used in this project have different open-source licenses and should be checked in the repositories themselves. As of writing this readme file the following licenses apply:
tum simulator: "We release our code under the BSD license. However, as we strongly build upon Gazebo and HectorSLAM, corresponding licenses apply as well. "

gazebo_models: Apache License

ar_track_alvar: No exceptions have been made - thus assumed to be the default for ROS wiki - Creative Commons Attribution 3.0

ardrone_autonomy: BSD License

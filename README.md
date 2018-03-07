
# FLYING AR-DRONE USING TAGS

## ABOUT
The goal of this project is to have an AR-Drone fly and obtain instructions from markers (aka tags) on the wall in order to direct it. The code is built in such a way that the movement is not hard coded, but the instruction of each tag is - thus you can change the placements of the tags on the walls and without changing the code the drone should change its movement accordingly.

In order to obtain this goal we use ROS Kinetic in addition to some open-source github repositories.
Tested on Ubuntu 16.04.

NOTE:
The main package with my code is drone_application. Other packages included here are packages which I used and may have edited slightly, thus it may be easier to just copy them from here rather than copy the original repositories and try to edit them to what I have.

## Preperation Instructions
*  It is recommended to go through the ROS tutorials and install ROS Kinetic through them:
http://wiki.ros.org/ROS/Tutorials/

*  After installing ROS Kinetic:

	*  Put the directory `drone_application` in `~/catkin_ws/src`.

	*  The following repositories need to be downloaded and installed:  
	https://github.com/AutonomyLab/ardrone_autonomy  
	https://github.com/sniekum/ar_track_alvar
		*  ardrone_autonomy wiki: 
		https://ardrone-autonomy.readthedocs.io/en/latest/  
		*  ardrone_autonomy installation instructions:  
		The official installation instructions do not mention kinetic, but an installation is available in the Ubuntu repos:  
		`sudo apt-get install ros-kinetic-ardrone-autonomy`
		*  ar_track_alvar wiki:  
		http://wiki.ros.org/ar_track_alvar  
		* ar_track_alvar installation instructions:  
		The official installation instructions do not mention kinetic, but an installation is available in the Ubuntu repos:  
		`sudo apt-get install ros-kinetic-ar-track-alvar`

	*  tum simulator and gazeebo - optional:  
	gazeebo - http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install  
	tum simulator - https://github.com/angelsantamaria/tum_simulator  
		* gazeebo installation: 
		use "Default installation: one-liner" in:
		http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install
		*  tum simulator and wiki (ignore installation instructions in wiki):  
		http://wiki.ros.org/tum_simulator 
		*  tum simulator installation instructions:  
		Follow the installation instructions in README.md of tum_simulator git repository but: 
			*  I added some tags to my gazeebo worlds within tum simulator, as such I recommend copying my tum_simulator folder into `~/catkin_ws/src` instead of downloading the folder from the github repository. Otherwise continue with installation instructions from README.md in the tum_simulator github repository.  
			*  You can use the models supplied in the gazeebo_models folder to make new gazeebo worlds of your own. These came from:  
			https://github.com/mikaelarguedas/gazebo_models

*  At this point you should compile catkin:  
`catkin_make`

## Running Instructions
*  Real Drone:  
	*  Connecting to drone:  
		`roslaunch drone_application launch_drone.launch`  
	*  Starting ar_track_alvar:  
		`roslaunch drone_application pr2_indiv_no_kinect_edited.launch`  
	*  View drone camera output in real time:  
		`rosrun image_view image_view image:=/ardrone/front/image_raw`  
	*  Run code and create log file:  
		`rosrun drone_application real_drone_flight.py | tee ~/catkin_ws/src/drone_application/logs/real_drone_flight_$(date +%Y%m%d%H%M%S).log`  
	*  NOTE: In order to debug effectivly I found it useful to have a screen capture program (like "Simple Screen Recorder") capture the screen with the code running aswell as the drone camera image.  
	*  Kill switch in case something goes wrong:  
		`rostopic pub -1 /ardrone/land std_msgs/Empty`
	
*  Gazeebo Simulator:  
	*  Run simulator: roslaunch drone_application test_simulator_tags4.launch  
	*  View drone camera output in real time:  
		`rosrun image_view image_view image:=/ardrone/front/image_raw`
	*  Run code and create log file: 
		rosrun drone_application real_drone_flight.py | tee ~/catkin_ws/src/drone_application/logs/simulator_drone_flight_$(date +%Y%m%d%H%M%S).log
		
## Editing code and testing
*  You can use markerTester2.py to be able to pick up the drone and point at the tags and see the values that ar_track_alvar gives you while the drone captures the tags in different positions. To run markerTester2.py use:  
`rosrun drone_application markerTester.py`

*  It was convenient for me to do this while haveing the drone camera output viewed on my pc and using a screen capturing tool to see the output of markerTester2.py side by side with what the camera sees

## Useful Sources
*  ROS Tutorials:
http://wiki.ros.org/ROS/Tutorials/

*  tum_simulator wiki has some commands you can control the drone with directly from terminal:
http://wiki.ros.org/tum_simulator

*  Launch file came from:
https://edu.gaitech.hk/drones/ar_parrot_2/ar-parrot-2-ros.html


## Licenses
This Project is released under GPL v.3 which can be found in the license file.

Different reposetories used in this project have different open-source licenses and should be checked in the repositories themselves. As of writing this readme file the following licenses apply:

*  tum simulator: "We release our code under the BSD license. However, as we strongly build upon Gazebo and HectorSLAM, corresponding licenses apply as well. "

*  gazebo_models: Apache License

*  ar_track_alvar: LGPL-2.1

*  ardrone_autonomy: BSD License

## Credits
Made by Or Hanoch  
Project Manager: Shunmuga Pillay  
Professional Adviser: Pravesh Ranchod

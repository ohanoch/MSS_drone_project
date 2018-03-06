#!/usr/bin/python

# Made by Or Hanoch based on code by Jason Chalom
# Project Manager: Shunmuga Pillay
# Proffesional Adviser: Pravesh Ranchod

# This code is intended to be used for checking the data that ar_track_alvar tag detection returns when the drone detects a tag.

# Usage:
# First run `roslaunch drone_application launch_drone.launch` to connect to the drone.
# Then run oslaunch drone_application pr2_indiv_no_kinect_edited.launch` tp start the tag detection.
# Then run `rosrun drone_application markerTester.py` to run this code
# It can be used with `rosrun image_view image_view image:=/ardrone/front/image_raw` so you can see what the drone sees alongside the output.

# importing ros messages
import rospy

# import the ar_alvar marker objects
from visualization_msgs.msg import Marker

# importing python libraries required for computation
from tf.transformations import euler_from_quaternion

# import miscellaniouse python libraries used within the code
import datetime
import sys

# Globals

# used in rospy.sleep to measure Hz of ros loop.
# 30Hz is what AR-Drone2 declares as the rate it subscribes to topics when connecting to it
RATE_CONST = 30

# This function gets called when a tag is detected and changes variables to make the drone react appropreatly
# input: data - a variable proveded by ar_track_alvar with the relative position and "color"=shape  (i.e green=[0,1,0]=down tag) of a tag
# output: no output - the function changes the golabl variables for movement speeds and current markers
def tag_detected(data):
    print("**************************************************************")
    print("                           TAG DETECTED                       ")
    print("**************************************************************")

    curr_roll, curr_pitch, curr_yaw = euler_from_quaternion(
        [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

    # Useful prints for understanding whats going on
    print("marker color:   r: " + str(data.color.r) + "\tg: " + str(data.color.g) + "\tb: " + str(data.color.b))
    print("marker position: \t x: " + str(data.pose.position.x) + "\ty: " + str(data.pose.position.y) + "\tz: " + str(
        data.pose.position.z))
    print("marker orientation: \t x: " + str(data.pose.orientation.x) + "\ty: " + str(
        data.pose.orientation.y) + "\tz: " + str(data.pose.orientation.z) + "\tw: " + str(data.pose.orientation.w))
    print("currRol: " + str(curr_roll) + "\tcurrPitch: " + str(curr_pitch) + "\tcurrYaw " + str(curr_yaw))

def flight():
    # initialize ros node and subscribe to topics
    rospy.init_node('drone_tag_movement', anonymous=True)

    # set ros loop rate
    rate = rospy.Rate(RATE_CONST)

    # subscribe to ar_track_alvar tag detection topic. When detected send the marker object to tagDetected function
    rospy.Subscriber("/visualization_marker", Marker, tag_detected)

    print("********************************************************************************")
    print("*                          STARTING MARKER TESTING                             *")
    print("********************************************************************************")


    # start ros loop
    while (not rospy.is_shutdown()):
        rate.sleep()


def usage():
    print("No parameters must be entered.\n\n")


if __name__ == '__main__':
    print("Drone tag recognition tester.\n\n")
    now = datetime.datetime.now()
    print(
        "time started: " + str(now.day) + "/" + str(now.month) + "/" + str(now.year) + " at: " + str(
            now.hour) + ":" + str(
            now.minute) + ":" + str(now.second) + "\n")
    if len(sys.argv) != 1:
        usage()
        sys.exit(1)

    try:
        flight()

    except rospy.ROSInterruptException:
        print('Ran into error - forcing landing and shutdown.')
        sys.exit(1)

#!/usr/bin/python

# Made by Or Hanoch based on code by Jason Chalom
# Project Manager: Shunmuga Pillay
# Proffesional Adviser: Pravesh Ranchod

# This code is intended to make a Parrot AR-Drone fly and obtain its movement directions from ar_alvar tags.
# Movement is done by continuously mving very small steps to the direction needed, until a new tag is detected that requires a change in direction

# Tested using ROS-Kinetic and an Parrot AR-Drone2

# Used the Ros tutorials to learn how to do this
# http://wiki.ros.org/ROS/Tutorials/

# Needs to be run in cunjunction with the following repositories:
# https://github.com/AutonomyLab/ardrone_autonomy
# https://github.com/sniekum/ar_track_alvar

# Notice the following files have been changed/added
# ar_track_alvar/launch/pr2_indiv_no_kinect_edited.launch
# drone_application/launch/launch_drone.launch

# Commands I used to run with real drone:
# connect to drone - `roslaunch drone_application launch_drone.launch`
# start tag tracking topic - `roslaunch ar_track_alvar pr2_indiv_no_kinect_edited.launch`
# view drone camera output in real time - `rosrun image_view image_view image:=/ardrone/front/image_raw`
# run the code and log the output - `rosrun drone_application real_drone_flight.py | tee real_drone_flight_$(date +%Y%m%d%H%M%S).log`

# In order to run in gazeebo the following repository is needed:
# https://github.com/angelsantamaria/tum_simulator/tree/master/cvg_sim_gazebo

# Notice the following files have been edited/changed in gazeebo repository
# tum_simulator/cvg_sim_gazebo/worlds/ardrone_testworld_tags4.world

# Can be run in gazeebo using - `roslaunch drone_application test_simulator_tags4.launch`


# importing ros messages
from std_msgs.msg import Empty
import rospy
from geometry_msgs.msg import Twist

# import the ar_alvar marker objects
from visualization_msgs.msg import Marker

# importing python libraries required for computation
from tf.transformations import euler_from_quaternion

# import miscellaniouse python libraries used within the code
import time
import datetime
import sys


# Class for keeping drone properties
class Drone:
    def __init__(self):
        # Current drone movement speed variables
        self.v_direction = 0  # vertival direction (up down) up = 1, down = -1
        self.h_direction = 0  # horizontal direction (left right) left = 1, right = -1
        self.f_direction = 0  # forward direction (forward backward)
        self.roll = 0  # forward/backward flip
        self.pitch = 0  # side to side swivle
        self.yaw = 0  # left/right flip
        self.land = False  # Checks if a command to land has been given

        # Marker detected history variable. [0] - current marker being locked and centered on, [1] - last marker viewed, [2] - before last marker viewed, ...
        self.curr_tags = [TAGS["empty"].value for i in range(4)]

        # Time variables used to calculate if the drone is "lost"
        self.start_time = time.time()
        self.last_tag_detected_time = time.time()

    # Print the variables of the drones movement
    def print_movement_stats(self):
        print ("v_direction: " + str(self.v_direction) +
               " h_direction: " + str(self.h_direction) +
               " f_direction: " + str(self.f_direction) +
               " roll: " + str(self.roll) +
               " pitch: " + str(self.pitch) +
               " yaw: " + str(self.yaw))

    # reset the drone movement to nothering - aka hover
    def reset_movement(self):
        print ("reseting movement")
        self.v_direction = 0
        self.h_direction = 0
        self.f_direction = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    # Change drone movement variables according to provided direction
    def change_direction(self, direction):
        print ("Entered change_direction function with direction = " + direction)
        print ("TAGS[direction].movement_arr = " + str(TAGS[direction].movement_arr))
        if self.curr_tags[0] == TAGS["transparent"].value:
            print ("changing direction according to transparent tag")
            for tag in TAGS:
                if TAGS[tag].value == self.curr_tags[1]:
                    self.v_direction = TAGS[tag].movement_arr[0]
                    self.h_direction = TAGS[tag].movement_arr[1]
                    self.f_direction = TAGS[tag].movement_arr[2]
                    self.roll = TAGS[tag].movement_arr[3]
                    self.pitch = TAGS[tag].movement_arr[4]
                    self.yaw = TAGS[tag].movement_arr[5]
                    break

        elif self.curr_tags[0] == TAGS["land"].value:
            print ("changing direction according to land tag")
            self.land = True
        else:
            print ("changing direction according to normal tag")
            self.v_direction = TAGS[direction].movement_arr[0]
            self.h_direction = TAGS[direction].movement_arr[1]
            self.f_direction = TAGS[direction].movement_arr[2]
            self.roll = TAGS[direction].movement_arr[3]
            self.pitch = TAGS[direction].movement_arr[4]
            self.yaw = TAGS[direction].movement_arr[5]
            self.print_movement_stats()

        self.curr_tags = [[-1, -1, -1]] + self.curr_tags[0:-1]

    def center_tag(self, x_position, y_position, z_position, pitch, detected_tag):
        print("centering tag...")

        # Tag is on left of capture
        # and drone direction is not right
        # Then move left
        if (x_position < -abs(X_CENTER) and
                self.h_direction != -GENERAL_SPEED):
            print("moving to the left...")
            self.h_direction = GENERAL_CENTERING_SPEED
            self.curr_tags[0] = detected_tag

        # Tag is on right of capture
        # and drone direction is not left
        # Then move right
        elif (x_position > abs(X_CENTER) and
                self.h_direction != GENERAL_SPEED):
            print("moving to the right...")
            self.h_direction = -GENERAL_CENTERING_SPEED
            self.curr_tags[0] = detected_tag

        else:
            self.h_direction = 0

        # Tag is too close
        # Then move back
        if z_position < Z_CENTER_CLOSE:
            print("moving to the back...")
            self.f_direction = -GENERAL_CENTERING_SPEED
            self.curr_tags[0] = detected_tag

        # Tag is too far
        # Then move forward
        elif z_position > Z_CENTER_FAR:
            print("moving to the front...")
            self.f_direction = GENERAL_CENTERING_SPEED
            self.curr_tags[0] = detected_tag

        else:
            self.f_direction = 0

        # Tag is on the bottom of capture
        # and drone direction is not up
        # Then move down
        if (y_position > abs(Y_CENTER) and
                self.v_direction != UP_SPEED):
            print("moving to the bottom...")
            self.v_direction = -DOWN_CENTERING_SPEED
            self.curr_tags[0] = detected_tag

        # Tag is on the top of capture
        # and drone direction is not down
        # Then move up
        elif (y_position < -abs(Y_CENTER) and
                self.v_direction != -DOWN_SPEED):
            print("moving to the top...")
            self.v_direction = UP_CENTERING_SPEED
            self.curr_tags[0] = detected_tag

        else:
            self.v_direction = GRAVITY_CONST

        # Drone is currently skewed with relation to tag - looking to the right
        # twist counterclockwies - look more to the left
        if pitch > abs(PITCH_CENTER):
            print("centering to twist counterclockwise...")
            self.pitch = GENERAL_CENTERING_SPEED

        # Drone is currently skewed with relation to tag - looking to the left
        # twist clockwies - look more to the right
        elif pitch < -abs(PITCH_CENTER):
            print("centering to twist clockwise...")
            self.pitch = -GENERAL_CENTERING_SPEED

        else:
            self.pitch = 0


class Tag:
    def __init__(self, value, max_distance, axis, movement_arr):
        self.value = value  # value given by ar_track_alvar
        self.max_distance = max_distance  # when the tag is at the very edge of the screen
        self.axis = axis  # axis of the direction of the tag
        self.movement_arr = movement_arr  # array of the direction this tag gives = [vDirection, hDirection, fDirection, roll, pitch, yaw]

    # Check if the position of the tag is on the edge of the screen
    def is_on_edge(self, x_position, y_position):
        if self.axis == "x":
            if ((self.max_distance < 0 and x_position < self.max_distance) or
                    (self.max_distance > 0 and x_position > self.max_distance)):
                return True
        elif self.axis == "y":
            if ((self.max_distance < 0 and y_position < self.max_distance) or
                    (self.max_distance > 0 and y_position > self.max_distance)):
                return True
        return False


class Pid:
    def __init__(self):
        self.last_target = [0, 0, 0]
        self.last_pid_time = 0
        self.error_sum = [0, 0, 0]

    # PID controller meant to stabalize the drone and avoid drifting and other unwanted movements
    # based off of eschnou's ardrone_autonomy prototype pid
    # https://github.com/eschnou/ardrone-autonomy/blob/master/lib/PID.js
    # input: target location to move to (1x3 list). Also uses the class's variables for past time,target and error sum
    # output: movement to be made in each direction (1x3 list)
    def apply_pid(self, target):
        curr_time = time.time()

        dt = (curr_time - self.last_pid_time)  # time difference since last pid calculation

        de = [0, 0, 0]  # error derivation
        if self.last_pid_time != 0:
            de = [(i - j) / dt for i, j in zip(target, self.last_target)]
            self.error_sum = [j + k for j, k in zip([i * dt for i in target], self.error_sum)]  # Integrate error

        print ("dt: " + str(dt) + " error_sum: " + str(self.error_sum) + " de: " + str(de) + " target: " + str(target) + " last target: " + str(self.last_target))

        # multiply the target, error_sum and de lists (of size 3) by Kp, Ki and Kd respectivly and then add the lists together to form a single movement list
        final_movement = [a + b + c for a, b, c in
                          zip([Kp * i for i in target], [Ki * i for i in self.error_sum], [Kd * i for i in de])]

        self.last_target = target
        self.last_pid_time = curr_time

        return final_movement


# Globals

# used in rospy.sleep to measure Hz of ros loop.
# 30Hz is what AR-Drone2 declares as the rate it subscribes to topics when connecting to it
RATE_CONST = 30

# PID constants
# used following link to help understand how to tune them:
# https://electronics.stackexchange.com/questions/127524/how-to-calculate-kp-kd-and-ki
Kp = 1
Ki = 0
Kd = 0.05

# Speed variables to be given at certain points.
# NOTE: Up and Down need specific speeds - presumably because of gravity
GENERAL_SPEED = 0.03  #
UP_SPEED = 0.12  # speeds for general movement between tags
DOWN_SPEED = 0.25  #
GENERAL_CENTERING_SPEED = 0.025  #
UP_CENTERING_SPEED = 0.07  # speeds for when drone is centering on a tag
DOWN_CENTERING_SPEED = 0.025  #

GRAVITY_CONST = 0.02  # speed to maintain height and fight gravity. might not be necessary.

# tag variables
TAGS = {"up": Tag([0.5, 0.5, 0], 0.7, "y", [UP_SPEED, 0, 0, 0, 0, 0]),
        "down": Tag([0, 1, 0], -0.7, "y", [-DOWN_SPEED, 0, 0, 0, 0, 0]),
        "left": Tag([0, 0, 1], 0.7, "x", [GRAVITY_CONST, GENERAL_SPEED, 0, 0, 0, 0]),
        "transparent": Tag([0.5, 0, 0.5], 999, "none", [0, 0, 0, 0, 0, 0]),
        "land": Tag([0, 0.5, 0.5], 999, "none", [0, 0, 0, 0, 0, 0]),
        "empty": Tag([-1, -1, -1], 999, "none", [0, 0, 0, 0, 0, 0])}

# variables that determine the leaniancy of when the drone is concidered "centered" on a tag
X_CENTER = 0.03  # leaniancy for left and right
Y_CENTER = 0.03  # leaniancy for up and down
Z_CENTER_CLOSE = 0.28  # leaniancy for being too close to the wall
Z_CENTER_FAR = 0.38  # leaniancy for being too far from the wall
ROLL_CENTER = 300  # roll is flipping forward and backwards. roll cant be recognized thus large number is given  to make it always within range
PITCH_CENTER = 0.15  # pitch is twisting side to side. pitch between -1.2 and 1.2 centered at 0.0
YAW_CENTER = 300  # yaw is flipping left and right. abs(pi) = good yaw, upside down = 0. drone automatically fixes yaw thus large number is given to make it always within range

# Other Globals
TIME_UNTILL_LOST = 3  # determines amount of time that drone can try to center while it does not see a tag before it is considered lost

drone1 = Drone()


def reset_vel(velocity_publisher, vel_msg):
    vel_msg.linear.x = 0.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.0

    velocity_publisher.publish(vel_msg)


# This function avoids going back to a tag you already saw if it is on the edge of the screen
# and to the opposite direction of what you got from it
def avoid_ping_pong(drone, x_position, y_position):
    if drone.curr_tags[0] == TAGS["empty"].value:
        if drone.curr_tags[2] == TAGS["transparent"].value:
            for tag in TAGS:
                if drone.curr_tags[3] == TAGS[tag].value:
                    if TAGS[tag].is_on_edge(x_position, y_position):
                        print ("--- exited to avoid ping pong " + str(tag) + " before transparent tag ---")
                        return True
                    break
        else:
            for tag in TAGS:
                if drone.curr_tags[2] == TAGS[tag].value:
                    if TAGS[tag].is_on_edge(x_position, y_position):
                        print ("--- exited to avoid ping pong " + str(tag) + " tag ---")
                        return True
                    break
    return False


# This function gets called when a tag is detected and changes variables to make the drone react appropreatly
# input: data - a variable proveded by ar_track_alvar with the relative position and "color"=shape  (i.e green=[0,1,0]=down tag) of a tag
# output: no output - the function changes the golabl variables for movement speeds and current markers
def tag_detected(data):
    print("**************************************************************")
    print("                           TAG DETECTED                       ")
    print("**************************************************************")

    global drone1

    drone1.last_tag_detected_time = time.time()

    detected_tag = [data.color.r, data.color.g, data.color.b]

    print ("drone1.curr_tags" + str(drone1.curr_tags) + " detected_tag: " +str(detected_tag))

    # Make sure that drone does not re-detect the tag it just got new directions from
    # and thus center back on it instead of moving to next tag
    if drone1.curr_tags[1] == detected_tag:
        drone1.print_movement_stats()
        print ("--- exited to avoid going back to previouse tag ---")
        return

    # Don't go back to the previouse previouse tag.
    # Essentially ignore the type of the previouse tag if it is on the edge.
    if avoid_ping_pong(drone1, data.pose.position.x, data.pose.position.y):
        return

    # When recognizing new tag drone sometimes overshoots or does funny things,
    # we make it sleep to stabalize prior to obtaining new commands
    if drone1.curr_tags[0] == TAGS["empty"].value:
        print ("recognized new tag. sleeping for 2 seconds")
        # zeroing out movement
        # necessary in order to not have drone continue moving until new moevment order is given
        drone1.reset_movement()
        rospy.sleep(2.)

    curr_roll, curr_pitch, curr_yaw = euler_from_quaternion(
        [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

    # Useful prints for understanding whats going on
    print("marker color:   r: " + str(data.color.r) + "\tg: " + str(data.color.g) + "\tb: " + str(data.color.b))
    print("marker position: \t x: " + str(data.pose.position.x) + "\ty: " + str(data.pose.position.y) + "\tz: " + str(
        data.pose.position.z))
    print("marker orientation: \t x: " + str(data.pose.orientation.x) + "\ty: " + str(
        data.pose.orientation.y) + "\tz: " + str(data.pose.orientation.z) + "\tw: " + str(data.pose.orientation.w))
    print("currRol: " + str(curr_roll) + "\tcurrPitch: " + str(curr_pitch) + "\tcurrYaw " + str(curr_yaw))
    print("currVDirection: " + str(drone1.v_direction) + "\tcurrHDirection: " + str(
        drone1.h_direction) + "\tcurrFDirection: " + str(drone1.f_direction))

    # if marker in center of capture - change direction according to it
    if (abs(data.pose.position.x) <= X_CENTER
            and abs(data.pose.position.y) <= Y_CENTER
            and data.pose.position.z >= Z_CENTER_CLOSE
            and data.pose.position.z <= Z_CENTER_FAR
            and abs(curr_roll) < ROLL_CENTER
            and abs(curr_pitch) < PITCH_CENTER
            and abs(curr_yaw) < YAW_CENTER):

        print("\nstabilizing (sleeping) for 5 seconds and then changing direction according to tag..\n")

        for tag in TAGS:
            if drone1.curr_tags[0] == TAGS[tag].value:
                # zeroing out movement
                # necessary in order to not have drone continue moving until new moevment order is given
                drone1.reset_movement()
                rospy.sleep(5.)
                drone1.change_direction(tag)
                drone1.print_movement_stats()
                return

    else:
        # tag is recognized, but not in the center of the captured image
        # here we make small adgustements in order to center it

        # The current tag we are locked on for centering is either empty (i,e, a new tag)
        # or the tag that we are detecting currently
        if drone1.curr_tags[0] == TAGS["empty"].value or drone1.curr_tags[0] == detected_tag:
            drone1.center_tag(
                data.pose.position.x, data.pose.position.y, data.pose.position.z, curr_pitch, detected_tag)

    # combat gravity
    if drone1.v_direction == 0:
        drone1.v_direction = GRAVITY_CONST


def flight():
    # initialize ros node and subscribe to topics
    rospy.init_node('drone_tag_movement', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    land_publisher = rospy.Publisher('/ardrone/land', Empty, queue_size=10)
    takeoff_publisher = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)

    # set ros loop rate
    rate = rospy.Rate(RATE_CONST)

    vel_msg = Twist()

    # Give 10 seconds for connection to be established + time to prepare screen capture and other things
    print("wait 10 seconds for takeoff...\n")
    sys.stdout.flush()
    rospy.sleep(10.)

    # send takeoff command
    print("sending takeoff command...\n")
    sys.stdout.flush()
    takeoff_publisher.publish(Empty())

    # Give drone time to stabalize after takeoff before sending new commands
    print("sleeping 12 seconds to stabalize...\n")
    sys.stdout.flush()
    rospy.sleep(12.)

    # initialize all the ros movement variables to 0
    reset_vel(velocity_publisher, vel_msg)

    # subscribe to ar_track_alvar tag detection topic. When detected send the marker object to tagDetected function
    rospy.Subscriber("/visualization_marker", Marker, tag_detected)

    print("********************************************************************************")
    print("*                            STARTING AUTO-FLIGHT                              *")
    print("********************************************************************************")

    global drone1

    # default to go up after takeoff until a tag is detected
    drone1.v_direction = UP_SPEED

    backtrace_time = time.time()  # this is used to measure time of drone being "lost" if it doesn't detect any tags

    xyz_pid = Pid()
    rpy_pid = Pid()

    # start ros loop
    while (not rospy.is_shutdown()) and not drone1.land:
        print ("")
        print ("------------------------------------------------------------------------------------")
        drone1.print_movement_stats()

        # NOTICE: drone and tag coordinates are not the same! tag_x = drone_y , tag_y = drone_z , tag_z = drone_x
        # the names of fDirection, hDirection and vDirection
        # can help understand the meaning of xyz in each part of the code
        curr_xyz_command = xyz_pid.apply_pid([drone1.f_direction, drone1.h_direction, drone1.v_direction])
        curr_rpy_command = rpy_pid.apply_pid([drone1.roll, drone1.yaw, drone1.pitch])

        print ("currXYZCommand: " + str(curr_xyz_command))
        print ("currRPYCommand: " + str(curr_rpy_command))
        print ("------------------------------------------------------------------------------------")

        # [1,0,0] - forward, [0,1,0] - left, [0,0,1] - up
        vel_msg.linear.x = curr_xyz_command[0]
        vel_msg.linear.y = curr_xyz_command[1]
        vel_msg.linear.z = curr_xyz_command[2]

        vel_msg.angular.x = 0  # currRPYCommand[0]
        vel_msg.angular.y = 0  # currRPYCommand[1]
        vel_msg.angular.z = curr_rpy_command[2]

        velocity_publisher.publish(vel_msg)

        # in case of loss of contact with tag go in the opposite way you were going
        # notice that the direction values set here will be overwritten if tag is detected
        if (drone1.curr_tags[0] != [-1, -1, -1] and
                time.time() - drone1.last_tag_detected_time > TIME_UNTILL_LOST and
                backtrace_time < drone1.last_tag_detected_time):
            print ("--------------------------------------------------------------")
            print ("lost tag " + str(drone1.curr_tags[0]) + " - going back to try and find it")
            print ("--------------------------------------------------------------")
            drone1.f_direction *= -1
            drone1.h_direction *= -1
            drone1.v_direction *= -1
            backtrace_time = time.time()

        rate.sleep()

    land_publisher.publish(Empty())


def usage():
    print("No parameters must be entered.\n\n")


if __name__ == '__main__':
    print("Drone movement by tags initialized.\n\n")
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

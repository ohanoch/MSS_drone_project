#!/usr/bin/python

# Jason Chalom 711985
# Used the Ros tutorials to learn how to do this
# http://wiki.ros.org/ROS/Tutorials/
# http://wiki.ros.org/turtlesim/Tutorials/Moving%20in%20a%20Straight%20Line
# Taken from: https://answers.ros.org/question/9783/programmatically-get-modelstate-from-gazebo/
# https://answers.ros.org/question/69754/quaternion-transformations-in-python/

# The reason there is a loop for both takingoff and landing is because we have found that gazebo sometimes does not respond to initial commands so 
# this is a hack to ensure the drone will takeoff

import rospy
from std_msgs.msg import Empty
import rospy
from gazebo_msgs.srv import *
from geometry_msgs.msg import Twist

import numpy as np
from tf.transformations import euler_from_quaternion

from visualization_msgs.msg import Marker
import time

# Globals
timeout = 0.2
burst_rate = 20
rate_const = 25

# Yaw constants
Kp_yaw = 0.2
Ki_yaw = 0.003 # 0.001
Kd_yaw = 0.0003 # 0.0001

# xyz constants
Kp = 0.5
Ki = 0.001 # 0.0002
Kd = 0.0001 # 0.00002

#ar_tag variables
startTime = time.time()
direction = 0 #0=up 1=down 2=straight 3=left 4=right 5=back
standby_time = 0 #time to hover between change of direction (to avoid drifting)
#standby_x #, standby_y, standby_z = 0,0,0

def takeoff_loop():
    takeoff_publisher = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)

    startPos = getPosition()
    currPos = startPos
#    for i in range(burst_rate):
    while (abs(currPos[2]) <= abs(startPos[2])+0.5):
        takeoff_publisher.publish(Empty())
        currPos = getPosition()
        print("trying to take off...")
    print (startPos)
    print (currPos)
    rospy.sleep(timeout)

def land_loop():
    land_publisher = rospy.Publisher('/ardrone/land', Empty, queue_size=10)
	
    currPos = getPosition()
#    for i in range(burst_rate):
    while(currPos[2]>0.4):
        land_publisher.publish(Empty())
        currPos = getPosition()
    rospy.sleep(timeout)

def resetVel(velocity_publisher, vel_msg):
    vel_msg.linear.x = 0.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.0

    velocity_publisher.publish(vel_msg)

def pid_yaw(Perr_old, Perr, Pierr, Piderr, roll, pitch, yaw):
    target = 0.0 # assumed to reset the yaw rate

    Perr = target - yaw
    Pierr = Pierr + (target - yaw)
#    Piderr = Piderr - (target - yaw)
    Piderr = Perr - Perr_old
  
    u = Kp_yaw*Perr + Ki_yaw*Pierr + Kd_yaw*Piderr

    return Perr,Pierr,Piderr,u

def pid_xyz(Perr_old, Perr, Pierr, Piderr, x, y, z, target_x, target_y, target_z):
    drone = np.array([x, y, z])
    target = np.array([target_x, target_y, target_z])

    Perr = target - drone
    Pierr = Pierr + (target - drone)
    # Piderr = Piderr - (target - drone)
    Piderr = Perr - Perr_old

    u = Kp*Perr + Ki*Pierr + Kd*Piderr

    return Perr,Pierr,Piderr,u

def detect_tag(data):
    # This will set a flag and stop executing autonomous search
    print("**************************************************************")
    print("                           TAG DETECTED                       ")
    print("**************************************************************")
    global startTime
    global direction
    global standby_time # , standby_x, standby_y, standby_z
   
    if standby_time == 0:
        standby_time = 100
    
    currRoll,currPitch,currYaw = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

    print("marker color:   r: " + str(data.color.r) + "\tg: " + str(data.color.g) + "\tb: " + str(data.color.b))
    print("marker pose: \t x: " + str(data.pose.position.x) + "\ty: " + str(data.pose.position.y) + "\tz: " + str(data.pose.position.z))

    print("marker orientation: \t x: " + str(data.pose.orientation.x) + "\ty: " + str(data.pose.orientation.y) + "\tz: " + str(data.pose.orientation.z) + "\tw: " + str(data.pose.orientation.w))
    print("currRol: " + str(currRoll) + "\tcurrPitch: " + str(currPitch) + "\tcurrYaw " +str(currYaw))


    if(abs(data.pose.position.x) <= 0.06 and abs(data.pose.position.y) <= 0.04):# if marker in center of capture.
										# and data.pose.position.z == 1):
        if(data.color.r == 0 and data.color.g == 0 and data.color.b == 1): #Marker0 = go left
            direction = 3

        if(data.color.r == 0 and data.color.g == 1 and data.color.b == 0): #Marker2 = go down
            direction = 1

        if(data.color.r == 1 and data.color.g == 0 and data.color.b == 0): #Marker1 = go right
            direction = 4
        
#        standby_x, standby_y, standby_z,_,_,_ = getPosition()
#        standby_time = 10

#    if startTime - time.time()<1:
#        detect = 0
#    else:
#        detect = 1
#        startTitme = time.time


def pid(x_input, y_input, z_input):
#    Pierr_yaw = 0.0
#    Piderr_yaw = 0.0 
    
#    Pierr = 0.0
#    Piderr = 0.0 

    rospy.init_node('lab2b_pid', anonymous=True)
#    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(rate_const) # 10hz
 #   vel_msg = Twist()

#    takeoff_loop()
 #   resetVel(velocity_publisher, vel_msg)

    rospy.Subscriber("/visualization_marker", Marker, detect_tag) 

#    print("********************************************************************************")
#    print("*                              STAGE 0 STARTED                                 *")
#    print("********************************************************************************")
#    stage = 0
#    global detect
#    detect = 0
#    global direction
#    global standby_time # , standby_x, standby_y, standby_z
#    lastDirection = direction+1
#    speed_of_movement = 0.55
#    lastPerr = 0.0
#    lastPerr_yaw = 0.0
    while not rospy.is_shutdown():

#        x,y,z,roll,pitch,yaw = getPosition()
#        if standby_time > 0:
#            standby_time = standby_time - 1
#            if lastDirection != direction:
#                standby_time = 0

#            x_input = standby_x
#            y_input = standby_y
#            z_input = standby_z
#        if direction == 0: #up
#            if lastDirection != direction:
#                x_input=x
#                y_input=y
#            z_input=z + speed_of_movement 
#        elif direction == 1: #down
#            if lastDirection != direction:
#                x_input=x
#                y_input=y
#            z_input=z - speed_of_movement
#            if z_input<=0.05:
        """
                break
#                land_loop()
        elif direction == 2: #straight
            if lastDirection != direction:
                y_input=y
                z_input=z
            x_input=x + speed_of_movement
        elif direction == 3: #left
            if lastDirection != direction:
                x_input=x
                z_input=z
            y_input=y + speed_of_movement
        elif direction == 4: #right
            if lastDirection != direction:
                x_input=x
                z_input=z
            y_input=y - speed_of_movement
        elif direction == 5: #back
            if lastDirection != direction:
                y_input=y
                z_input=z
            x_input=x-speed_of_movement

        lastDirection = direction

        Perr_yaw = 0.0

        Perr_yaw,Pierr_yaw,Piderr_yaw,yaw = pid_yaw(lastPerr_yaw, Perr_yaw,Pierr_yaw,Piderr_yaw,roll,pitch,yaw)
        lastPerr_yaw = Perr_yaw

###########     PROBLEM: always going into else because starts at 0,0,0 and moves slightly backwards. Need to correc to account got negatives!!
#        print ("x_input: "+str(x_input) + "\ty_input: " + str(y_input) + "\tz_input: " +str(z_input))
        if (z_input >= 0.0): #x_input >= 0.0) and (y_input >= 0.0) and (z_input >= 0.0):  #check for negs since simulation is in positive space
#            print ("went into if statement")
            Perr = 0.0
            Perr,Pierr,Piderr,u = pid_xyz(lastPerr, Perr, Pierr, Piderr, x, y, z, x_input, y_input, z_input)
            lastPerr = Perr
#            print ("Perr: " + str(Perr)+ "\tPierr: " + str(Pierr) + "\tPiderr: " + str(Piderr) +"\tu: "+ str(u))
            
            vel_msg.linear.x = u[0]
            vel_msg.linear.y = u[1]
            vel_msg.linear.z = u[2]
        else:
            print ("went into else statement")
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

        vel_msg.angular.x = roll
        vel_msg.angular.y = pitch
        vel_msg.angular.z = yaw
#        velocity_publisher.publish(vel_msg)
 #       else:
 #            if stage == 0:
 #                print("********************************************************************************")
 #                print("*                              STAGE 1 STARTED                                 *")
 #                print("********************************************************************************")
 #                stage = 1
 #                detect = 0
 #                continue
 #            elif stage == 1:
 #                print("********************************************************************************")
 #                print("*                              STAGE 2 STARTED                                 *")
 #                print("********************************************************************************")
 #                stage = 2
 #                detect = 0
 #                continue
 #            elif stage == 2:
 #                print("reached elif after stage 2 complete - should have never happened")
#            land_loop()
        """
        rate.sleep()

    land_loop() # probably not needed

# Taken from: https://answers.ros.org/question/9783/programmatically-get-modelstate-from-gazebo/
def gms_client(model_name, relative_entity_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
    	gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    	resp1 = gms(model_name,relative_entity_name)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def getPosition():
    res = gms_client('quadrotor','world')
    x = res.pose.position.x
    y = res.pose.position.y
    z = res.pose.position.z
    # print(res.pose)

    roll,pitch,yaw = euler_from_quaternion([res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w])
#    print "I am at x=%.2f y=%.2f z=%.2f roll=%.2f pitch=%.2f yaw=%.2f direction=%d"%(x, y, z, roll, pitch, yaw, direction)
    return x,y,z,roll,pitch,yaw

def usage():
    print("At least three parameters must be selected.\n-1 will denote inf value\nPosition: -1 -1 -1 will make the drone hover and only correct its yaw\n\n")
    print("usage: %s <position: x y z>"%sys.argv[0])

if __name__ == '__main__':
    print("Basic Drone PID Control.\nJason Chalom 711985\n2017\n\n")
    if len(sys.argv) != 4:
        usage()
        sys.exit(1)

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])

        #getPosition()
        pid(x, y, z)

    except rospy.ROSInterruptException: 
        print('Well Shit.')
        sys.exit(1)

# Commands for ar-drone from bash.
taken from:  
http://wiki.ros.org/tum_simulator

## take off :
rostopic pub -1 /ardrone/takeoff std_msgs/Empty

## land 
rostopic pub -1 /ardrone/land std_msgs/Empty

## switch camera 
rosservice call /ardrone/togglecam

## motion: 

### fly forward:
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

### fly backward:
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -1.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

### fly to left:
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

### fly to right: 
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -1.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

### fly up:
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

### fly down:
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: -1.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

### counterclockwise rotation
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 1.0}}'

### clockwise rotation:
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: -1.0}}'

### stop:
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

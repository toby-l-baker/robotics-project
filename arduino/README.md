# Directions to add rosserial to project
## Workspace Tasks
`cd <workspace>/src`\
`git clone https://github.com/ros-drivers/rosserial.git`\
`cd <workspace>`\
`catkin_make`
## Arduino Environment Tasks
`cd <arduino_sketchbook>/libraries`\
<arduino_sketchbook> in lab is: /home/cc/ee106a/fa19/class/ee106a-abo/Arduino\
`rm -rf ros_lib`\
`rosrun rosserial_arduino make-libraries.py .`

Arduino is located at `/opt/arduino`\
Notes taken from: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

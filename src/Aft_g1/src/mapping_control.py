#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from Aft_g1.msg import MappingCmd
import os
import subprocess

linear_vel = 0.1
angular_vel = 0.1
k_vel = 3
direction = 0
flag = 0  # switch
speed = 0.0

cmd_vel_pub = None

def chatter_callback(cmd):
    global flag, speed, cmd_vel_pub
    rospy.loginfo("callback function called!")
    
    if not cmd.isCmd:
        rospy.loginfo("data is direction!")
        if flag and (cmd.direction in [0, 1, 2, 3, 4, 5, 6]):
            base_cmd = Twist()
            base_cmd.linear.x = 0.0
            base_cmd.linear.y = 0.0
            base_cmd.angular.z = 0.0
            
            if cmd.direction == 1:
                base_cmd.linear.x = cmd.speed
            elif cmd.direction == 2:
                base_cmd.linear.x = -cmd.speed
            elif cmd.direction == 3:
                base_cmd.linear.y = cmd.speed
            elif cmd.direction == 4:
                base_cmd.linear.y = -cmd.speed
            elif cmd.direction == 5:
                base_cmd.angular.z = cmd.speed
            elif cmd.direction == 6:
                base_cmd.angular.z = -cmd.speed
                
            cmd_vel_pub.publish(base_cmd)
    else:
        rospy.loginfo("data is command!")
        rospy.loginfo(f"cmd.cmdId = {cmd.cmdId}, flag = {flag}")
        
        if cmd.cmdId == 1:
            flag = 1
            # Start mapping in background
            # subprocess.Popen(["roslaunch", "Aft_g1", "gmapping.launch"])
            # subprocess.Popen(["roslaunch", "Aft_g1", "gmapping_sim.launch"])
            rospy.loginfo("mapping started!")
            
        elif flag and cmd.cmdId == 2:
            rospy.loginfo("mapping cancelled!")
            flag = 0
            os.system("rosnode kill gmapping")
            # os.system("rosnode kill robot_state_publisher")
            # os.system("killall -9 rviz")
            
        elif flag and cmd.cmdId == 3:
            map_path = f"/home/robot/maps/{cmd.graphName}"
            os.system(f"rosrun map_server map_saver -f {map_path}")
            rospy.loginfo(f"Map saved to {map_path}")

def main():
    global cmd_vel_pub
    rospy.init_node('map_ctrl')
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/map_ctrl', MappingCmd, chatter_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
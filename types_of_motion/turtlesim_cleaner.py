#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

def poseCallback(pose_message):   #callback function for the subscriber
    global x
    global y, yaw
    x= pose_message.x
    y= pose_message.y
    yaw = pose_message.theta

    #print "pose callback"
    #print ('x = {}'.format(pose_message.x)) #new in python 3
    #print ('y = %f' %pose_message.y) #used in python 2
    #print ('yaw = {}'.format(pose_message.theta)) #new in python 3


def move(velocity_publisher, speed, distance, is_forward):     #for linear motion
        #declare a Twist message to send velocity commands
        velocity_message = Twist()      #for calling the twist message
        #get current location 
        global x, y
        x0=x
        y0=y

        if (is_forward):       #if the arguement for forward direction is true
            velocity_message.linear.x =abs(speed)
        else:
        	velocity_message.linear.x =-abs(speed)

        distance_moved = 0.0
        loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
        
        while True :
                rospy.loginfo("Turtlesim moves forwards")
                velocity_publisher.publish(velocity_message)     

                loop_rate.sleep()
                
                distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))   #calculating euclidian distance
                print  (distance_moved)
                print(x)
                if  not (distance_moved<distance):        #if the turtlesim moves further than the required distance
                    rospy.loginfo("reached")
                    break
        
        #finally, stop the robot when the distance is moved
        velocity_message.linear.x =0
        velocity_publisher.publish(velocity_message)
    
def rotate (velocity_publisher, angular_speed_degree, relative_angle_degree, clockwise):
    
    velocity_message = Twist()

    angular_speed=math.radians(abs(angular_speed_degree))    #converting degree to radian

    if (clockwise):
        velocity_message.angular.z =-abs(angular_speed)    #negative for clockwise rotation
    else: 
        velocity_message.angular.z =abs(angular_speed)     #positive for anti-clockwise rotation

    angle_moved = 0.0
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
    cmd_vel_topic='/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()   #instance before starting the motion

    while True :
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()    #current instance
        current_angle_degree = (t1-t0)*angular_speed_degree    #calculating the degree of rotation in real time
        loop_rate.sleep()


                       
        if  (current_angle_degree>relative_angle_degree):     #if the turtlesim rotates more than the required angle
            rospy.loginfo("reached")
            break

    #finally, stop the robot when the distance is moved
    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)


def go_to_goal(velocity_publisher, x_goal, y_goal):
    global x
    global y, yaw

    velocity_message = Twist()

    while (True):
        K_linear = 0.5   #defualt values should not be very high
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        linear_speed = distance * K_linear    #P Controller for linear speed
        #speed is proportional to relative disatance with destination. Meaning the speed reduces as we reach closer to the destination

        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-yaw)*K_angular   #P controller for angle speed

        #assigning linear and angular speed values to message 
        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)    #publishing the message
        print ('x=', x, ', y=',y, ', distance to goal: ', distance)

        if (distance <0.01):   #as the distance value is in float, it cannot be absolutely zero
            break

def setDesiredOrientation(publisher, speed_in_degree, desired_angle_degree):
    relative_angle_radians = math.radians(desired_angle_degree) - yaw
    clockwise=0
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    print ("relative_angle_radians: ",math.degrees(relative_angle_radians))
    print ("desired_angle_degree: ",desired_angle_degree)
    rotate(publisher, speed_in_degree,math.degrees(abs(relative_angle_radians)), clockwise)

def gridClean(publisher):  #conbination of all motion behaviours 
 
    desired_pose = Pose()
    desired_pose.x = 1
    desired_pose.y = 1
    desired_pose.theta = 0
 
    go_to_goal(publisher, 1,1)
 
    setDesiredOrientation(publisher, 30, math.radians(desired_pose.theta))
 
    for i in range(5):
        move(publisher, 2.0, 1.0, True)
        rotate(publisher, 20, 90, False)
        move(publisher, 2.0, 9.0, True)
        rotate(publisher, 20, 90, True)
        move(publisher, 2.0, 1.0, True)
        rotate(publisher, 20, 90, True)
        move(publisher, 2.0, 9.0, True)
        rotate(publisher, 20, 90, False)
    pass

 
 
def spiralClean(velocity_publisher, wk, rk):   #rk will change over time and wk will remain constant
    vel_msg = Twist()
    loop_rate = rospy.Rate(1)
 
    while((x<10.5) and (y<10.5)):
        rk=rk+1
        vel_msg.linear.x =rk
        vel_msg.linear.y =0
        vel_msg.linear.z =0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z =wk
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
 
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)



if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        #declare velocity publisher
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback) 
        time.sleep(2)

        #move(velocity_publisher, 1.0, 4.0, True)           #calling a function to move straight
        #rotate(velocity_publisher, 30, 90, True)          #calling a function to rotate in place for angle mantioned as a parameter
        go_to_goal(velocity_publisher, 2.0, 1.5)          #calling a function to move to the desired coordinate mantioned as a parameter
        #setDesiredOrientation(velocity_publisher, 30, 90)
        #spiralClean(velocity_publisher, 4, 0)
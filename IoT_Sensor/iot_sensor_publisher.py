#!/usr/bin/env python
# license removed for brevity
import rospy
from ros_essentials_cpp.msg import IoTSensor   #importing our custom msg file under ros_essentials package
import random

#create a new publisher. we specify the topic name, then type of message then the queue size
pub = rospy.Publisher('iot_sensor_topic', IoTSensor, queue_size=10)

#we need to initialize the node
rospy.init_node('iot_sensor_publisher_node', anonymous=True)

#set the loop rate
rate = rospy.Rate(1) # 1hz
#keep publishing until a Ctrl-C is pressed
i = 0
while not rospy.is_shutdown():
    iot_sensor = IoTSensor()   #onject creation of our custom message
    #assigning values to our fields
    iot_sensor.id = 1
    iot_sensor.name="iot_parking_01"
    iot_sensor.temperature = 24.33 + (random.random()*2)
    iot_sensor.humidity = 33.41+ (random.random()*2)
    rospy.loginfo("I publish:")
    rospy.loginfo(iot_sensor)    #printing the values of our fields
    pub.publish(iot_sensor)      #publishing as a node
    rate.sleep()
    i=i+1


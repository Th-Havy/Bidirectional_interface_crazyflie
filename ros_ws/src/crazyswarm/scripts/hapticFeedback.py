#!/usr/bin/env python

################################################################################
# Modules
################################################################################

# ROS
import rospy
from std_msgs.msg import String, Duration, Header
from rospy.numpy_msg import numpy_msg

# Python
import time

# Crazyflie
from crazyflie_driver.msg import GenericLogData


################################################################################
# Constants
################################################################################

DRONE_NAME = 'cf1'
SENSOR_TOPIC = 'log1'

MOTORS_UPDATE_RATE = 10 # Hz

################################################################################
# Global variables
################################################################################

# Sensor values

def updateSensorValues(logData):
    """ Callback called when new values are received from the multiranger deck.
        logData is a numpy-ized GenericLogData, and contains a header and an
        array of float64 values corresponding to the sensor measurements:
        logData.values = [front, back, left, right, up, down]"""

    # global sensorValues
    # TODO update sensors reading here

    return

def hapticFeedback():
    # global sensorValues

    # Create node
    rospy.init_node('hapticFeedback')
    rospy.loginfo("Initializing haptic feedback node.")

    # Subscribe to topic publishing multiranger deck sensor values
    rospy.Subscriber(DRONE_NAME + "/" + SENSOR_TOPIC, numpy_msg(GenericLogData), updateSensorValues)

    rospy.loginfo("Initialized successfully.")

    # ROS loop
    rate = rospy.Rate(MOTORS_UPDATE_RATE)
    while not rospy.is_shutdown():

        # ROS loop
        # TODO: send commands to motors here

        rate.sleep()


if __name__ == '__main__':
    try:
        hapticFeedback()
    except rospy.ROSInterruptException:
        pass

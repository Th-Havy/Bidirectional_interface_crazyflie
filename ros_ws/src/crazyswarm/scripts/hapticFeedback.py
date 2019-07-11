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
import socket
import struct
import numpy as np

# Crazyflie
from crazyflie_driver.msg import GenericLogData


################################################################################
# Constants
################################################################################

DRONE_NAME = 'cf1'
SENSOR_TOPIC = 'distances'

MOTORS_UPDATE_RATE = 10 # Hz

NB_DISTANCES = 6

HAPTIC_CLIENT_IP = "192.168.1.216"
UDP_PORT_DISTANCES = 8051

STOP_ALL_MOTORS = (float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf'))

################################################################################
# Global variables
################################################################################

# (front, back, up, down, left, right)
sensorDistances = STOP_ALL_MOTORS

################################################################################
# UDP utilities
################################################################################

# function to send distance data to client
def sendDistances(mySocket, data):

    if len(data) != NB_DISTANCES:
        rospy.logwarn("Trying to send invalid data.")
        return
    else:
        # Pack the data to a stream of bytes
        message = struct.pack('%sf' % len(data), *data)

        mySocket.sendto(message, (HAPTIC_CLIENT_IP, UDP_PORT_DISTANCES))

################################################################################
# Core functions
################################################################################

def updateSensorValues(logData):
    """ Callback called when new values are received from the multiranger deck.
        logData is a numpy-ized GenericLogData, and contains a header and an
        array of float64 values corresponding to the sensor measurements:
        logData.values = [front, back, up, down, left, right]"""
    global sensorDistances

    distances = logData.values

    if len(distances) != NB_DISTANCES:
        rospy.logwarn("Invalid data. Check that the log topic publishes the right values")
        return
    else:
        # Convert distances from float64 to float32
        distances = np.float32(distances)

        # Convert distances to meters
        distances = distances / 1000.0

        # Convert numpy array to tuple
        sensorDistances = tuple(distances)

def hapticFeedback():
    global sensorDistances

    # Create node
    rospy.init_node('hapticFeedback')
    rospy.loginfo("Initializing haptic feedback node.")

    # Subscribe to topic publishing multiranger deck sensor values
    rospy.Subscriber(DRONE_NAME + "/" + SENSOR_TOPIC, numpy_msg(GenericLogData), updateSensorValues)

    distanceSocket = socket.socket(socket.AF_INET, # Internet
                                socket.SOCK_DGRAM) # UDP

    rospy.loginfo("Initialized successfully.")

    # ROS loop
    rate = rospy.Rate(MOTORS_UPDATE_RATE)
    while not rospy.is_shutdown():
        sendDistances(distanceSocket, sensorDistances)

        rate.sleep()

    # Stop all motors at shutdown
    for i in range(10):
        sendDistances(distanceSocket, STOP_ALL_MOTORS)
        time.sleep(0.1)


if __name__ == '__main__':
    try:
        hapticFeedback()
    except rospy.ROSInterruptException:
        pass

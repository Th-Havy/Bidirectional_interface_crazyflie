#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Duration, Header
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
import tf.transformations

import time
from pynput import mouse
from math import sqrt

import threading

from crazyflie_driver.srv import *
from crazyflie_driver.msg import Position as PositionMsg
#import crazyflie_driver.srv._Takeoff as TakeoffSrv
#import crazyflie_driver.srv._GoTo as GotoSrv
#import crazyflie_driver.srv._Land as LandSrv


################################# Mocap

HAND_RIGIDBODY = "hand"
DRONE_RIGIDBODY = "cf1"

handRoomScaling = 2.0
clutchActivated = False

# if set to true only when the click is pressed, then set back to false immidiately
clutchTriggered = False

# mocapInputRotation = 0 --> you are facing the wall, and have the computers to your left and the entrance door to your right.
# mocapInputRotation = -90 --> you are facing the computers
# mocapInputRotation = 90 --> you have the computers behind you
mocapInputRotation = 0.0
observationInputRotation = 0.0 # computers behind you

rotationSpeedScaling = 0.02

rawHandPosition = Point(0.0, 0.0, 0.0)
oldRawHandPosition = Point(0.0, 0.0, 0.0)
deltaHandPosition = Point(0.0, 0.0, 0.0)

rawHandRotation = Quaternion()
handYaw = 0.0
referenceYaw = 0.0

# Target sent to the drone
handTarget = Point(0.0, 0.0, 0.0)
targetPositionInitialized = False

# Position of the drone
dronePosition = Point(0.0, 0.0, 0.0)

def sendPositionCommandThread(goToCommand):
    """thread sending the position target"""
    global handTarget

    dur = rospy.Duration.from_sec(1)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rospy.loginfo("Command: {0}".format(handTarget))
        goToCommand(groupMask=0, relative=False, goal=handTarget, yaw=0.0, duration=dur)
        rate.sleep()

    return

# mocapPos is of type PoseStamped
def updateHandPosition(mocapPos):
    global rawHandPosition
    global rawHandRotation

    rawHandPosition = mocapPos.pose.position
    rawHandRotation = mocapPos.pose.orientation

# Get drone position from mocap to initialized the target at the drone position
def updateDronePosition(mocapPos):
    global targetPositionInitialized
    global handTarget
    global dronePosition

    # Initialise target position at the current location of the drone
    if not targetPositionInitialized:
        handTarget = mocapPos.pose.position
        targetPositionInitialized = True
        rospy.loginfo("Initialized target position")

    dronePosition = mocapPos.pose.position

def initHandTracking(handRigidbodyName, droneRigidbodyName):
    global targetPositionInitialized

    rospy.Subscriber("vrpn_client_node/" + droneRigidbodyName + "/pose", PoseStamped, updateDronePosition)

    # Wait until target position initialized, which is done in the updateDronePosition() callback
    while not targetPositionInitialized:
        time.sleep(0.1)

    rospy.Subscriber("vrpn_client_node/" + handRigidbodyName + "/pose", PoseStamped, updateHandPosition)

    mouseListener = mouse.Listener(on_click=on_click)
    mouseListener.start()

# quat in Quaternion message format
def getYaw (quat):
    orientation_list = [quat.x, quat.y, quat.z, quat.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion (orientation_list)
    return yaw

# quat in Quaternion message format, point in Point message format
def rotatePoint(quat, point):
    q1 = [quat.x, quat.y, quat.z, quat.w]

    v1 = [point.x, point.y, point.z]
    q2 = list(v1)
    q2.append(0.0)
    vr = tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2),
        tf.transformations.quaternion_conjugate(q1)
    )[:3]

    return Point(vr[0], vr[1], vr[2])

# Point message type does not provide arithmetic operators ##########3
def substractPoints(p1, p2):
    p3 = Point()

    p3.x = p1.x - p2.x
    p3.y = p1.y - p2.y
    p3.z = p1.z - p2.z

    return p3

def addPoints(p1, p2):
    p3 = Point()

    p3.x = p1.x + p2.x
    p3.y = p1.y + p2.y
    p3.z = p1.z + p2.z

    return p3

def scalePoint(p, val):
    s = Point(p.x, p.y, p.z)

    s.x = s.x * val
    s.y = s.y * val
    s.z = s.z * val

    return s

################################# Mouse inputs

def on_click(x, y, button, pressed):
    global clutchActivated
    global clutchTriggered
    global referenceYaw
    global handYaw

    if button ==  mouse.Button.left:
        #rospy.loginfo('{0} at {1} {2}'.format(
        #    'Pressed' if pressed else 'Released',
        #    (x, y), time.time()))
        if (pressed):
            rospy.loginfo("pressed")
            clutchActivated = True
            clutchTriggered = True
            referenceYaw = handYaw;
        else:
            rospy.loginfo("released")
            clutchActivated = False

#################################

def clampInSafeArea(target):
    if target.x > 3:
        target.x = 3
    if target.x < -3:
        target.x = -3
    if target.y > 3:
        target.y = 3
    if target.y < -3:
        target.y = -3
    if target.z > 3:
        target.z = 3
    if target.z < 0.1:
        target.z = 0.1

    return target

def updateHeader(header):
    header.seq = header.seq + 1
    header.stamp=rospy.Time.now()

def controlDrone():
    # Create node
    rospy.init_node('handInterface')

    rospy.loginfo("Initialized hand interface node")

    initHandTracking(HAND_RIGIDBODY, DRONE_RIGIDBODY)

    # Wait for necessary service: drone commands
    rospy.wait_for_service('cf1/takeoff')
    rospy.wait_for_service('cf1/go_to')
    rospy.wait_for_service('cf1/land')

    # Acquire reference to these ServiceServer
    takeoff = rospy.ServiceProxy('takeoff', Takeoff)
    goTo = rospy.ServiceProxy('cf1/go_to', GoTo)
    land = rospy.ServiceProxy('cf1/land', Land)
    #mocap : ('vrpn_client_node/cf1/pose')
    #lowlevel: /cf1/cmd_position

    rospy.loginfo("All necessary services advertised.")

    time.sleep(0.5)


    """rospy.loginfo("Z=0.2")
    posCommandTopic = rospy.Publisher('cf1/cmd_position', PositionMsg, queue_size=10)

    header = Header(seq=1, stamp=rospy.Time.now(), frame_id='world')
    posCommandTopic.publish(PositionMsg(header=header, x=0.0, y=0.0, z=0.2, yaw=0.0))

    time.sleep(2.0)

    rospy.loginfo("Z=0.9")
    updateHeader(header)
    posCommandTopic.publish(PositionMsg(header=header, x=0.0, y=0.0, z=0.9, yaw=0.0))

    time.sleep(5.0)

    rospy.loginfo("Z=0.9, X=0.9")
    updateHeader(header)
    posCommandTopic.publish(PositionMsg(header=header, x=0.9, y=0.0, z=0.9, yaw=0.0))

    time.sleep(5.0)

    rospy.loginfo("Z=0.2, X=0.9")
    updateHeader(header)
    posCommandTopic.publish(PositionMsg(header=header, x=0.9, y=0.0, z=0.1, yaw=0.0))"""

    """rospy.loginfo("Taking off")

    dur = rospy.Duration.from_sec(1.0)

    takeoff(groupMask=0, height=0.1, duration=dur)

    time.sleep(3)

    rospy.loginfo("Z=0.9")
    pt = Point(0, 0, 0.9)

    dur = rospy.Duration.from_sec(5.0)

    goTo(groupMask=0, relative=True, goal=pt, yaw=0.0, duration=dur)

    time.sleep(10)

    rospy.loginfo("X=1.0")

    pt = Point(1.0, 0, 0)

    dur = rospy.Duration.from_sec(5.0)

    goTo(groupMask=0, relative=True, goal=pt, yaw=0.0, duration=dur)

    time.sleep(10)

    rospy.loginfo("Z=-0.9")
    pt = Point(0, 0, -0.9)

    dur = rospy.Duration.from_sec(5.0)

    goTo(groupMask=0, relative=True, goal=pt, yaw=0.0, duration=dur)

    time.sleep(10)

    rospy.loginfo("finished")

    dur = rospy.Duration.from_sec(5.0)

    land(groupMask=0, height=0.1, duration=dur)"""

    global handRoomScaling
    global rawHandPosition
    global oldRawHandPosition
    global deltaHandPosition
    global handTarget
    global clutchActivated
    global clutchTriggered

    header = Header(seq=1, stamp=rospy.Time.now(), frame_id='world')


    t = threading.Thread(target=sendPositionCommandThread, args=(goTo,))
    t.start()

    # ROS loop
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        deltaHandPosition = substractPoints(rawHandPosition, oldRawHandPosition)
        handYaw = getYaw(rawHandRotation)

        oldRawHandPosition = rawHandPosition

        # Ignore large deltas (dangerous)
        if (sqrt(deltaHandPosition.x * deltaHandPosition.x + deltaHandPosition.y * deltaHandPosition.y + deltaHandPosition.z * deltaHandPosition.z) > 1.0):
            continue

        if clutchActivated == True:
            if clutchTriggered == True:
                clutchTriggered = False
                handTarget = dronePosition
            rospy.loginfo("Clutch")
            #droneVelocityControl.desiredYawRate = Mathf.DeltaAngle(referenceYaw, handYaw) * rotationSpeedScaling
        else:
            #droneVelocityControl.desiredYawRate = 0.0

            rot = tf.transformations.quaternion_from_euler (0, observationInputRotation + mocapInputRotation, 0)

            # Convert from list to Message type
            directionRotation = Quaternion(rot[0], rot[1], rot[2], rot[3])

            handTarget = addPoints(handTarget, scalePoint(rotatePoint(directionRotation, deltaHandPosition), handRoomScaling))
            handTarget = clampInSafeArea(handTarget)

            #updateHeader(header)
            #posCommandTopic.publish(PositionMsg(header=header, x=handTarget.x, y=handTarget.y, z=handTarget.z, yaw=0.0))
            #dur = rospy.Duration.from_sec(1)
            #goTo(groupMask=0, relative=False, goal=handTarget, yaw=0.0, duration=dur)

        rospy.loginfo("Target: {0}".format(handTarget))
        rospy.loginfo("Hand position: {0}".format(rawHandPosition))
        rate.sleep()


if __name__ == '__main__':
    try:
        controlDrone()
    except rospy.ROSInterruptException:
        pass

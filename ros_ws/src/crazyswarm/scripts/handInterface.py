#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Duration, Header
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import time
from pynput import mouse
from math import sqrt

from crazyflie_driver.srv import *
from crazyflie_driver.msg import Position as PositionMsg
#import crazyflie_driver.srv._Takeoff as TakeoffSrv
#import crazyflie_driver.srv._GoTo as GotoSrv
#import crazyflie_driver.srv._Land as LandSrv


################################# Mocap

HAND_RIGIDBODY = "hand"

handRoomScaling = 8.0
clutchActivated = False

# mocapInputRotation = 0 --> you are facing the wall, and have the computers to your left and the entrance door to your right.
# mocapInputRotation = -90 --> you are facing the computers
# mocapInputRotation = 90 --> you have the computers behind you
mocapInputRotation = 0.0
observationInputRotation = 90.0 # computers behind you

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

# mocapPos is of type PoseStamped
def updateHandPosition(mocapPos):
    rawHandPosition = mocapPos.pose.position
    rawHandRotation = mocapPos.pose.orientation

# Get drone position from mocap to initialized the target at the drone position
def initTargetPosition(mocapPos):
    if not targetPositionInitialized:
        handTarget = mocapPos.pose.position
        targetPositionInitialized = True

def initHandTracking(rigidbodyName):
    rospy.Subscriber("vrpn_client_node/" + HAND_RIGIDBODY + "/pose", PoseStamped, initTargetPosition)

    # Wait until target position initialized
    while not targetPositionInitialized:
        time.sleep(0.1)

    rospy.Subscriber("vrpn_client_node/cf1/pose", PoseStamped, updateHandPosition)

    mouseListener = mouse.Listener(on_click=on_click)
    mouseListener.start()

# quat in Quaternion message format
def getYaw (quat):
    orientation_list = [quat.x, quat.y, quat.z, quat.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
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

################################# Mouse inputs

def on_click(x, y, button, pressed):
    if button ==  mouse.Button.left:
        rospy.loginfo('{0} at {1} {2}'.format(
            'Pressed' if pressed else 'Released',
            (x, y), time.time()))
        if (pressed):
            clutchActivated = True

            dronePositionControl.target = transform;
            referenceYaw = handYaw;
        else:
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
    if target.z < 0:
        target.z = 0

def updateHeader(header):
    header.seq = header.seq + 1
    header.stamp=rospy.Time.now()

def controlDrone():
    # Create node
    rospy.init_node('handInterface')

    rospy.loginfo("Initialized hand interface node")

    #initHandTracking(HAND_RIGIDBODY)

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

    header = Header(seq=1, stamp=rospy.Time.now(), frame_id='')

    posCommandTopic.publish(PositionMsg(header=header, x=0.0, y=0.0, z=0.2, yaw=0.0))

    updateHeader(header)
    posCommandTopic.publish(PositionMsg(header=header, x=0.0, y=0.0, z=0.2, yaw=0.0))

    time.sleep(5.0)

    rospy.loginfo("Z=0.3")
    updateHeader(header)
    posCommandTopic.publish(PositionMsg(header=header, x=0.0, y=0.0, z=0.3, yaw=0.0))

    time.sleep(2.0)

    rospy.loginfo("Z=0.1")
    updateHeader(header)
    posCommandTopic.publish(PositionMsg(header=header, x=0.0, y=0.0, z=0.1, yaw=0.0))"""

    dur = rospy.Duration.from_sec(1.0)

    takeoff(groupMask=0, height=0.1, duration=dur)

    time.sleep(1)

    pt = Point(0, 0, 0.1)

    dur = rospy.Duration.from_sec(1.0)

    goTo(groupMask=0, relative=True, goal=pt, yaw=0.0, duration=dur)

    time.sleep(0.5)

    dur = rospy.Duration.from_sec(1.0)

    land(groupMask=0, height=0.05, duration=dur)

    # ROS loop
    """rate = rospy.Rate(50)
    while not rospy.is_shutdown():

        deltaHandPosition = rawHandPosition - oldRawHandPosition
        handYaw = getYaw(rawHandRotation)

        oldRawHandPosition = rawHandPosition

        # Ignore large deltas (dangerous)
        if (sqrt(deltaHandPosition.x * deltaHandPosition.x + deltaHandPosition.y * deltaHandPosition.y + deltaHandPosition.z * deltaHandPosition.z) > 1.0f):
            continue

        if clutchActivated == True:
            ;
            #droneVelocityControl.desiredYawRate = Mathf.DeltaAngle(referenceYaw, handYaw) * rotationSpeedScaling
        else:
            #droneVelocityControl.desiredYawRate = 0.0
            directionRotation = quaternion_from_euler (0, observationInputRotation + mocapInputRotation, 0)
            handTarget = handTarget + rotatePoint(directionRotation, deltaHandPosition) * handRoomScaling;
            clampInSafeArea(handTarget)

            updateHeader(header)
            posCommandTopic.publish(PositionMsg(header=header, x=handTarget.x, y=handTarget.y, z=handTarget.z, yaw=0.0))

        rate.sleep()"""


if __name__ == '__main__':
    try:
        controlDrone()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

from __future__ import print_function, division

import random

import cv2
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv_bridge

bridge = cv_bridge.CvBridge()
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
SLICE_SIZE = 20
last_direction = 2

def follower(ros_image):
    global last_direction
    image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    centre, angle = get_path(image)

    if centre is None or angle is None:
        move = Twist()
        move.angular.z = last_direction
        pub.publish(move)
        return

    THRESHOLD_LEFT = image.shape[0] / 2 - SLICE_SIZE
    THRESHOLD_RIGHT = image.shape[0] / 2 + SLICE_SIZE
    off_centre_left = centre < THRESHOLD_LEFT
    on_centre = THRESHOLD_LEFT <= centre <= THRESHOLD_RIGHT
    off_centre_right = centre > THRESHOLD_RIGHT

    THRESHOLD_ANGLE = np.pi / 6
    path_left = angle > THRESHOLD_ANGLE
    path_straight = -THRESHOLD_ANGLE <= angle <= THRESHOLD_ANGLE
    path_right = angle < -THRESHOLD_ANGLE

    move = Twist()
    move.linear.x = 2
    if off_centre_left:
        move.angular.z = 3
    elif off_centre_right:
        move.angular.z = -3

    last_direction = move.angular.z

    pub.publish(move)

def get_path(frame):
    """
    Take a cv2 image and find the current centre of the path and the angle it's moving in.

    :param frame: a CV2 image, BGR
    :returns: centre in pixels, path angle in radians
    """
    slice = frame[-(3 * SLICE_SIZE) :]

    _, s, _ = cv2.split(cv2.cvtColor(slice, cv2.COLOR_BGR2HSV))
    path = s > 125

    _, segment = np.where(path[-SLICE_SIZE:])

    centre = segment.mean()
    y = frame.shape[0] - SLICE_SIZE // 2
    if np.isnan(centre):
        centre = None

    _, next_segment = np.where(path[-(3 * SLICE_SIZE) : -SLICE_SIZE])
    next_centre = next_segment.mean()
    next_y = round(y - 1.5 * SLICE_SIZE)

    if not (centre is None or np.isnan(next_centre)):
        dy = y - next_y
        dx = next_centre - centre
        angle = np.arctan(dy/dx)
    else:
        angle = None

    return centre, angle

rospy.init_node("line_robot", anonymous=True)
subscriber = rospy.Subscriber("/robot/camera1/image_raw", Image, follower)
rospy.spin()

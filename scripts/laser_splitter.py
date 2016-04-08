#!/usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3Stamped
import numpy
import copy

laser_range = [None, 1.9199]
height_range = [2.059, 2.164]
height_offset = 0.44
last_h = None
skip_cnt = 0

def laser_callback(msg, pub):
    assert laser_range[1]<msg.angle_max
    assert msg.angle_min < height_range[0] < height_range[1] < msg.angle_max
    
    laser_max_idx = int((laser_range[1] - msg.angle_min) / msg.angle_increment)
    assert laser_max_idx < len(msg.ranges), 'Assert: laser_max_idx(=%d) < len(msg.ranges)(=%d) failed.' % (
        laser_max_idx, len(msg.ranges))

    # extract normal laser segment
    laser_msg = LaserScan()
    laser_msg.header = msg.header
    # laser_msg.header.frame_id = "world"
    laser_msg.angle_min = msg.angle_min
    laser_msg.angle_increment = msg.angle_increment
    laser_msg.time_increment = msg.time_increment
    laser_msg.scan_time = msg.scan_time
    laser_msg.range_min = msg.range_min
    laser_msg.range_max = msg.range_max
    laser_msg.angle_max = laser_range[1]
    laser_msg.ranges = list(msg.ranges[:laser_max_idx])
    laser_msg.intensities = list(msg.intensities[:laser_max_idx])

    height_idx = (int((height_range[0] - msg.angle_min) / msg.angle_increment),
                  int((height_range[1] - msg.angle_min) / msg.angle_increment))
    
    # extract height segment
    h_lst = list(msg.ranges[height_idx[0]:(height_idx[1] + 1)])
    h_mid = numpy.median(h_lst);
    #print(h_mid)
    inliers = [];
    for x in h_lst:
        if abs(x-h_mid)<0.10:
            inliers.append(x)
    # print(inliers)
    if not inliers:
        return
    h = numpy.mean(inliers)

    global last_h,skip_cnt
    if last_h is None:
        last_h = h
    if abs(h - last_h) > 0.1:
        skip_cnt += 1
        if skip_cnt > 40:
            last_h = None
            skip_cnt = 0
        return
    else:
        last_h = h
        skip_cnt = 0

    height_msg = Vector3Stamped()
    height_msg.header = msg.header
    height_msg.vector.x = 0
    height_msg.vector.y = 0
    height_msg.vector.z = h - height_offset

    # publish messages
    pub['laser'].publish(laser_msg)
    pub['height'].publish(height_msg)

    laser_msg.ranges = h_lst
    laser_msg.angle_min = height_range[0]
    laser_msg.angle_max = height_range[1]
    pub['hbeams'].publish(laser_msg)

if __name__ == "__main__":
    rospy.init_node('laser_splitter')

    laser_pub = rospy.Publisher('~scan_out', LaserScan, queue_size=10)
    height_pub = rospy.Publisher('~height', Vector3Stamped, queue_size=10)
    hbeams_pub = rospy.Publisher('~hbeams', LaserScan, queue_size=10)

    laser_sub = rospy.Subscriber('~scan_in', LaserScan, laser_callback, callback_args=dict(
        laser=laser_pub, height=height_pub, hbeams=hbeams_pub))

    laser_range[1]  = rospy.get_param('~laser_range_upper_limit', laser_range[1])
    height_range[0] = rospy.get_param('~height_range_lower_limit', height_range[0])
    height_range[1] = rospy.get_param('~height_range_upper_limit', height_range[1])
    height_offset   = rospy.get_param('~height_offset', height_offset)

    rospy.spin()

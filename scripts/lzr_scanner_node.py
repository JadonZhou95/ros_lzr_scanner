#!/usr/bin/env python3
"""This script contains a ros driver for the lzr scanner"""
import sys
import numpy as np

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import lzr_scanner as lzr

port = '/dev/ttyUSB0'
baud_rate = 460800


def lzr_scanner_publisher(ser):
    """Main function for capturing and publishing the point cloud"""
    # init ros node and publisher
    pub = rospy.Publisher('lzr_scanner_node', PointCloud2, queue_size=10)
    rospy.init_node('lzr_scanner', anonymous=True)

    rate = rospy.Rate(15)
    
    # loop for capturing and publishing the points continuously
    while True:

        # capture the scanning results
        if not ser.capture_frame():
            
            # get the scanner reading
            points = np.zeros((274, 4, 3))
            points[:, 0, :] = ser.p1_points
            points[:, 1, :] = ser.p2_points
            points[:, 2, :] = ser.p3_points
            points[:, 3, :] = ser.p4_points

            points /= 1000

            # convert the scanner reading into the ros msg
            msg = PointCloud2()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "lzr_scanner_frame"
            msg.height = 4
            msg.width = 274
            msg.fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)]
            msg.is_bigendian = False
            msg.point_step = 12
            msg.row_step = 12 * msg.width
            msg.is_dense = True
            msg.data = np.asarray(points, np.float32).tostring()

            # publish the ros message
            pub.publish(msg)
            rate.sleep()




if __name__ == '__main__':
    # initialize the device
    print("Python version: ", sys.version)
    scanner = lzr.LzrScanner(port, baud_rate)
    scanner.connect()

    try:
        lzr_scanner_publisher(scanner)
    except rospy.ROSInterruptException:
        pass


#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu

class ImuOffsetFilterNode:
    def __init__(self, topic_in, topic_out, N, frame_id=None):
        self.N = N
        self.imu_pub = rospy.Publisher(topic_out, Imu, queue_size=1)
        self.imu_sub = rospy.Subscriber(topic_in, Imu, self.imu_cb)

        self.offset_done = False

        self.la_offset = np.zeros(3)
        self.av_offset = np.zeros(3)

        self.offset_array = np.zeros((6,N))
        self.offset_array[:] = np.nan

        self.header = None
        self.frame_id = frame_id

        self.orientation = None
        self.orientation_covariance = np.zeros(9)

        self.angular_velocity = np.zeros(3)
        self.angular_velocity_covariance = np.zeros(9)

        self.linear_acceleration = np.zeros(3)
        self.linear_acceleration_covariance = np.zeros(9)

    def imu_cb(self, imu_msg):

        self.header = imu_msg.header

        # sets frame_id if it was set
        if self.frame_id:
            self.header.frame_id = self.frame_id

        self.orientation = imu_msg.orientation
        self.orientation_covariance = imu_msg.orientation_covariance
        self.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance
        self.angular_velocity_covariance = imu_msg.angular_velocity_covariance

        self.linear_acceleration[0] = imu_msg.linear_acceleration.x
        self.linear_acceleration[1] = imu_msg.linear_acceleration.y
        self.linear_acceleration[2] = imu_msg.linear_acceleration.z


        self.angular_velocity[0] = imu_msg.angular_velocity.x
        self.angular_velocity[1] = imu_msg.angular_velocity.y
        self.angular_velocity[2] = imu_msg.angular_velocity.z

        if self.offset_done:
            self.get_offsets()
            self.set_offsets()
        else:
            self.publish()

    def get_offsets(self):

        for accel_x in np.nditer(self.offset_array[0], op_flags=['readwrite']):
            if np.isnan(accel_x):
                accel_x[...] = self.linear_acceleration[0]

        for accel_y in np.nditer(self.offset_array[1], op_flags=['readwrite']):
            if np.isnan(accel_y):
                accel_y[...] = self.linear_acceleration[1]

        for accel_z in np.nditer(self.offset_array[2], op_flags=['readwrite']):
            if np.isnan(accel_z):
                accel_z[...] = self.linear_acceleration[2]

        for vel_x in np.nditer(self.offset_array[3], op_flags=['readwrite']):
            if np.isnan(vel_x):
                vel_x[...] = self.angular_velocity[0]

        for vel_y in np.nditer(self.offset_array[4], op_flags=['readwrite']):
            if np.isnan(vel_y):
                vel_y[...] = self.angular_velocity[1]

        for vel_z in np.nditer(self.offset_array[5], op_flags=['readwrite']):
            if np.isnan(vel_z):
                vel_z[...] = self.angular_velocity[2]

        if not np.isnan(self.offset_array[0,-1]):
            self.set_offsets()
            rospy.loginfo("Offsets set")

            rospy.loginfo("Offset linear x={}".format(self.la_offset[0]))
            rospy.loginfo("Offset linear y={}".format(self.la_offset[1]))
            rospy.loginfo("Offset linear z={}".format(self.la_offset[2]))
            rospy.loginfo("Offset angular x={}".format(self.av_offset[0]))
            rospy.loginfo("Offset angular y={}".format(self.av_offset[1]))
            rospy.loginfo("Offset angular z={}".format(self.av_offset[2]))

            self.offset_done = True

    def set_offsets(self):
        self.la_offset[0] = np.average(self.offset_array[0])
        self.la_offset[1] = np.average(self.offset_array[1])
        self.la_offset[2] = np.average(self.offset_array[2])

        self.av_offset[0] = np.average(self.offset_array[3])
        self.av_offset[1] = np.average(self.offset_array[4])
        self.av_offset[2] = np.average(self.offset_array[5])

    def publish(self, frame_id):
        imu_msg = Imu()
        imu_msg.header = self.header

        imu_msg.orientation_covariance = self.orientation_covariance
        imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance
        imu_msg.angular_velocity_covariance = self.angular_velocity_covariance


        imu_msg.linear_acceleration.x = self.linear_acceleration[0] - self.la_offset[0]
        imu_msg.linear_acceleration.y = self.linear_acceleration[1] - self.la_offset[1]
        imu_msg.linear_acceleration.z = self.linear_acceleration[2] - self.la_offset[2]

        imu_msg.angular_velocity.x = self.angular_velocity[0] - self.av_offset[0]
        imu_msg.angular_velocity.y = self.angular_velocity[1] - self.av_offset[1]
        imu_msg.angular_velocity.z = self.angular_velocity[2] - self.av_offset[2]

        self.imu_pub.publish(imu_msg)

if __name__ == '__main__':
    rospy.loginfo("Initiallising imu_offset_filter node");
    rospy.init_node("imu_offset_filter_node")

    N = rospy.get_param("N", 1000)
    topic_in = rospy.get_param("in", "/imu/data_raw")
    topic_out = rospy.get_param("out", "/imu/filtered")
    frame_id = rospy.get_param("frame_id", "imu_link")

    imu_offset_filter_node = ImuOffsetFilterNode(topic_in=topic_in,
                                                 topic_out=topic_out,
                                                 N=N,
                                                 frame_id=frame_id)

    rospy.loginfo("Gathering data for offsets")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if imu_offset_filter_node.offset_done:
            imu_offset_filter_node.publish()
        rate.sleep()

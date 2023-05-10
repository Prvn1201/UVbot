#!/usr/bin/env python

import rospy
import tf2_ros

def main():
    # Initialize ROS node
    rospy.init_node('tf_buffer_example')

    # Create a new TF buffer with a 10 second cache time
    tf_buffer = tf2_ros.Buffer(rospy.Duration(50.0))

    # Create a TF listener to listen for transformations
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Loop to print the available frames every second
    rate = rospy.Rate(1.0) # 1 Hz
    while not rospy.is_shutdown():
        all_frames = tf_buffer.all_frames_as_string()
        print("Available frames: %s" % all_frames)
        rate.sleep()

if __name__ == '__main__':
    main()


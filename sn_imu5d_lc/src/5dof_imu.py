#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import serial

def imu_publisher():
    rospy.init_node('imu_publisher', anonymous=True)
    imu_pub = rospy.Publisher('imu_data', Imu, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz publishing rate

    # Arduino serial communication setup
    serial_port = '/dev/ttyUSB0'  # Update with the correct port
    serial_baud = 115200  # Update with the correct baud rate
    imu_msg = Imu()

    with serial.Serial(serial_port, serial_baud, timeout=1) as ser:
        while not rospy.is_shutdown():
            line = ser.readline().decode().strip()  # Decode bytes to string
            data = line.split('\t')

            if len(data) >= 5:
                x_current_estimate = float(data[0])
                y_current_estimate = float(data[1])
                z_current_estimate = float(data[2])
                gyro_x_current_estimate = float(data[3])
                gyro_y_current_estimate = float(data[4])

                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.linear_acceleration.x = x_current_estimate
                imu_msg.linear_acceleration.y = y_current_estimate
                imu_msg.linear_acceleration.z = z_current_estimate
                imu_msg.angular_velocity.x = gyro_x_current_estimate
                imu_msg.angular_velocity.y = gyro_y_current_estimate

                imu_pub.publish(imu_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass

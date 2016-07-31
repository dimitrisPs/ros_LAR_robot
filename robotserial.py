#!/usr/bin/env python

import serial
import struct
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

STRUCT_BYTE_SIZE = 12 * 3
'''
struct from mcu contains in order:

float32 gyro[3]
float32 accelerometer[3]
float32 magnitometer[3]
float32 imutemp
float32 encoder[2]
float32 volatage
float32 current



'''
# def robot_pub():
# configure serial
ser = serial.Serial(baudrate=576000, port='/dev/ttyUSB0')
ser.bytesize = serial.EIGHTBITS
ser.parity = serial.PARITY_NONE

# initialize ros publicer

rospy.init_node('robot', anonymous=True)

# publisher for 6dof imu raw dara (accel and gyro)
pub_imu_6_raw = rospy.Publisher('imu/data_raw', Imu, queue_size=10)

# publisher for 3dof magnetometer sensor
pub_imu_mag = rospy.Publisher('imu/mag', MagneticField, queue_size=10)

msg_Imu_6_DOF = Imu()
msg_Imu_mag = MagneticField()

rate = rospy.Rate(100)  # 100hz

# flush tx-rx buffers
ser.flushInput()
ser.flushOutput()
rospy.loginfo('flushed serial')
while not rospy.is_shutdown() & ser.isOpen():
    if ser.inWaiting() > STRUCT_BYTE_SIZE:

        msg_Imu_6_DOF.header.stamp = rospy.Time.now()
        msg_Imu_mag.header.stamp = rospy.Time.now()

        # read gyro data and put the in gyro tuple
        raw_bytes = ser.read(4)
        gyro = struct.unpack('f', raw_bytes)
        raw_bytes = ser.read(4)
        gyro += struct.unpack('f', raw_bytes)
        raw_bytes = ser.read(4)
        gyro += struct.unpack('f', raw_bytes)

        # read accel data and put them in accel tuple
        raw_bytes = ser.read(4)
        accel = struct.unpack('f', raw_bytes)
        raw_bytes = ser.read(4)
        accel += struct.unpack('f', raw_bytes)
        raw_bytes = ser.read(4)
        accel += struct.unpack('f', raw_bytes)

        # read mag data and put them in mag tuple
        raw_bytes = ser.read(4)
        mag = struct.unpack('f', raw_bytes)
        raw_bytes = ser.read(4)
        mag += struct.unpack('f', raw_bytes)
        raw_bytes = ser.read(4)
        mag += struct.unpack('f', raw_bytes)


        msg_Imu_6_DOF.angular_velocity.x = gyro[0]
        msg_Imu_6_DOF.angular_velocity.y = gyro[1]
        msg_Imu_6_DOF.angular_velocity.z = gyro[2]

        msg_Imu_6_DOF.linear_acceleration.x = accel[0]
        msg_Imu_6_DOF.linear_acceleration.y = accel[1]
        msg_Imu_6_DOF.linear_acceleration.z = accel[2]

        msg_Imu_mag.magnetic_field.x = mag[0]
        msg_Imu_mag.magnetic_field.y = mag[1]
        msg_Imu_mag.magnetic_field.z = mag[2]

        msg_Imu_6_DOF.header.frame_id = '/robot'
        msg_Imu_mag.header.frame_id = '/robot'




        pub_imu_6_raw.publish(msg_Imu_6_DOF)
        pub_imu_mag.publish(msg_Imu_mag)
        rate.sleep()

if (ser.isOpen()):
    ser.close()


    '''
            print ("g_x: %+-6.2f r/s \t g_y: %+-6.2f r/s \t g_z: %+-6.2f r/s \n " % (gyro[0],gyro[1],gyro[2]))
            print ("a_x: %+-6.2f r/s \t a_y: %+-6.2f r/s \t a_z: %+-6.2f r/s \n " % (accel[0], accel[1], accel[2]))
            print ("m_x: %+-6.2f r/s \t m_y: %+-6.2f r/s \t m_z: %+-6.2f r/s \n " % (mag[0], mag[1], mag[2]))
    '''
    # if __name__ == '__main__':
    #     try:
    #         robot_pub()
    #     except rospy.ROSInterruptException:
    #         pass

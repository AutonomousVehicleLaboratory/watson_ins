import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyploy as plt
import matplotlib.animation as animation
import numpy as np
import time

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
ax2 = fig.add_subplot(1,1,2)

watson_msg = Imu()
vmu_msg = Imu()

watson_x_accel = []
vmu_x_accel = []

rospy.init_node('imu_test')

rospy.Subscriber("/imu_raw", Imu, watson_callback)
#rospy.Subscriber("~gyro", Vector3Stamped, vmu_gyro_callback)
#rospy.Subscriber("~accelerometer", Vector3Stamped, vmu_accel_callback)
#rospy.Subscriber("~quaternion", QuaternionStamped, vmu_orientation_callback)
rospy.Subscriber("~data_raw", Imu, vmu_imu_callback)


def graph():
    print ("watson length", len(watson_x_accel))
    print ("vmu length", len(vmu_x_accel))
    xs = []
    ys = np.arange(len(watson_x_accel))

    ax1.plot(watson_x_accel, ys)
    ax1.plot(vmu_x_accel, ys)

    fig.show()


def watson_callback(data):
    watson_msg = data
    watson_x_accel.append(watson_msg.linear_acceleration.x)


def vmu_imu_callback(data):
    vmu_msg = data
    vmu_x_accel.append(vmu_msg.linear_acceleration.x)


start_time = time.time()
elapsed_time = time.time() - start_time

while (elapsed_time < 10):
    continue

graph()

rospy.spin()

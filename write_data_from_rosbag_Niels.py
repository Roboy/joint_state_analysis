import rospy
import numpy as np

import message_filters
import std_msgs.msg
from sensor_msgs.msg import JointState
from roboy_middleware_msgs.msg import MagneticSensor

rospy.init_node("write_rosbag")

body_part = "shoulder_right"
joint_names = [body_part + "_axis" + str(i) for i in range(3)]

record = open("./data/" + body_part + "_cf_data1.log", "w")
record.write("roll pitch yaw roll_cf pitch_cf yaw_cf\n")

numberOfSamples = 300000
samples = np.zeros((numberOfSamples, 9))
roll = 0
pitch = 0
yaw = 0
sample = 0

tracker_down = False
tracker_down_cnt = 0
last_tracker_down = 0


def tracker_down_callback(data):
    global tracker_down, tracker_down_cnt
    tracker_down = data.data
    # tracker_down_cnt += 1

def data_2_callback(tracking_data, cardsflow_data):

    global tracker_down, last_tracker_down
    # if tracker_down_cnt - last_tracker_down == 0:
    #     tracker_down = False

    if not tracking_data.name[0] == joint_names[0]:
        return

    global sample

    position = [0, 0, 0]
    position_cf = [0, 0, 0]
    for i in range(3):
        id_ref = tracking_data.name.index(joint_names[i])
	id_cf  = cardsflow_data.name.index(joint_names[i])
        position[i] = tracking_data.position[id_ref]
        position_cf[i] = cardsflow_data.position[id_cf]

    roll = position[0]
    pitch = position[1]
    yaw = position[2]

    roll_cf = position_cf[0]
    pitch_cf = position_cf[1]
    yaw_cf = position_cf[2]

    if tracker_down:
        rospy.loginfo("Down " + str(roll) + " " + str(pitch) + " " + str(yaw))
        # last_tracker_down = tracker_down_cnt
        return

    record.write(
        str(roll) + " " + str(pitch) + " " + str(yaw) + " " +
        str(roll_cf) + " " + str(pitch_cf) + " " + str(yaw_cf) + "\n")

    sample = sample + 1
    rospy.loginfo_throttle(5, "%s: \n Data collection progress: %f%%" % (
        body_part, float(sample) / float(numberOfSamples) * 100.0))


tracking_sub = message_filters.Subscriber('/roboy/pinky/external_joint_states', JointState)
cardsflow_sub = message_filters.Subscriber('/roboy/pinky/control/cardsflow_joint_states', JointState)
magnetic_sub = message_filters.Subscriber('/roboy/pinky/middleware/MagneticSensor', MagneticSensor)

ts = message_filters.ApproximateTimeSynchronizer([tracking_sub, cardsflow_sub], 10, 1.0)
ts.registerCallback(data_2_callback)

tracking_loss_sub = rospy.Subscriber('/tracking_loss', std_msgs.msg.Bool, tracker_down_callback)
rospy.spin()

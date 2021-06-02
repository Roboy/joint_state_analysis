import rospy
import numpy as np

import message_filters
import std_msgs.msg
from sensor_msgs.msg import JointState
from roboy_middleware_msgs.msg import MagneticSensor
from roboy_middleware_msgs.msg import MotorState, MotorCommand

rospy.init_node("write_rosbag")

motorstate_set_names = ""
for i in range(16):
    motorstate_set_names+=(" ms_s"+str(i))

motorcommand_set_names = ""
for i in range(16):
    motorcommand_set_names+=(" mc_s"+str(i))

motor_enc_names = ""
for i in range(16):
    motor_enc_names+=(" m_e"+str(i))

motor_disp_names = ""
for i in range(16):
    motor_disp_names+=(" m_d"+str(i))

motor_current_names = ""
for i in range(16):
    motor_current_names+=(" m_cur"+str(i))

body_part = "shoulder_right"
joint_names = [body_part + "_axis" + str(i) for i in range(3)]

timestamps_names = " timestamp_cf timestamp_htc timestamp_enc0"

joints_cf_htc_names = "roll_htc pitch_htc yaw_htc roll_cf pitch_cf yaw_cf"

record = open("./data/" + body_part + "_cf_data5.log", "w")
record.write(joints_cf_htc_names + motorstate_set_names + motor_enc_names + motor_disp_names + motor_current_names + motorcommand_set_names + timestamps_names + "\n")

numberOfSamples = 300000
samples = np.zeros((numberOfSamples, 9))
roll = 0
pitch = 0
yaw = 0
sample = 0

tracker_down = False
tracker_down_cnt = 0
last_tracker_down = 0

#motor_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55]

def tracker_down_callback(data):
    global tracker_down, tracker_down_cnt
    tracker_down = data.data
    # tracker_down_cnt += 1

def data_2_callback(tracking_data, cardsflow_data, motorstate_data, motorcommand_data):

    global tracker_down, last_tracker_down
    # if tracker_down_cnt - last_tracker_down == 0:
    #     tracker_down = False

    if not tracking_data.name[0] == joint_names[0]:
        return

    global sample

    position_htc = [0, 0, 0]
    position_cf = [0, 0, 0]
    setpoint_motorstate = [0] * 16
    encoder0_motor = [0] * 16
    displacement_motor = [0] * 16
    setpoint_motorcommand = [0] * 16
    current_motor = [0] * 16

    for i in range(3):
        id_ref = tracking_data.name.index(joint_names[i])
        id_cf  = cardsflow_data.name.index(joint_names[i])
        position_htc[i] = tracking_data.position[id_ref]
        position_cf[i] = cardsflow_data.position[id_cf]

    roll_htc = position_htc[0]
    pitch_htc = position_htc[1]
    yaw_htc = position_htc[2]

    roll_cf = position_cf[0]
    pitch_cf = position_cf[1]
    yaw_cf = position_cf[2]

    timestamp_cf = cardsflow_data.header.stamp
    timestamp_htc = tracking_data.header.stamp

    for j in range(16):
        setpoint_motorstate[j] = motorstate_data.setpoint[j]
        encoder0_motor[j] = motorstate_data.encoder0_pos[j]
        displacement_motor[j] = motorstate_data.displacement[j]
        current_motor[j] = motorstate_data.current[j]
        setpoint_motorcommand[j] = motorcommand_data.setpoint[j]

    timestamp_enc0 = motorstate_data.header.stamp

    all_data_array = position_htc + position_cf + setpoint_motorstate + encoder0_motor + displacement_motor + current_motor + setpoint_motorcommand + [timestamp_cf] + [timestamp_htc] + [timestamp_enc0]

    if tracker_down:
        rospy.loginfo("Down " + str(roll_htc) + " " + str(pitch_htc) + " " + str(yaw_htc))
        # last_tracker_down = tracker_down_cnt
        return
    
    log_data = ""
    for i in range(len(all_data_array)):
       log_data+=(" "+str(all_data_array[i]))

    """record.write(
        str(roll_htc) + " " + str(pitch_htc) + " " + str(yaw_htc) + " " +
        str(roll_cf) + " " + str(pitch_cf) + " " + str(yaw_cf) + "\n")"""

    record.write(log_data + "\n")
    #record.write(all_data_array)

    sample = sample + 1
    rospy.loginfo_throttle(5, "%s: \n Data collection progress: %f%%" % (
        body_part, float(sample) / float(numberOfSamples) * 100.0))


tracking_sub = message_filters.Subscriber('/roboy/pinky/external_joint_states', JointState)
cardsflow_sub = message_filters.Subscriber('/roboy/pinky/control/cardsflow_joint_states', JointState)
magnetic_sub = message_filters.Subscriber('/roboy/pinky/middleware/MagneticSensor', MagneticSensor)
motorstate_sub = message_filters.Subscriber('/roboy/pinky/middleware/MotorState', MotorState) 
motorcommand_sub = message_filters.Subscriber('/roboy/pinky/middleware/MotorState', MotorCommand)

ts = message_filters.ApproximateTimeSynchronizer([tracking_sub, cardsflow_sub, motorstate_sub, motorcommand_sub], 10, 0.01)
ts.registerCallback(data_2_callback)

tracking_loss_sub = rospy.Subscriber('/tracking_loss', std_msgs.msg.Bool, tracker_down_callback)
rospy.spin()

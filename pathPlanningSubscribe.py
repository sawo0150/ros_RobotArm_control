#!/usr/bin/env python3

import time
import threading
import evdev
from evdev import InputDevice, categorize, ecodes
import Adafruit_PCA9685
import math

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

# PCA9685 서보 드라이버 설정
servo_driver = Adafruit_PCA9685.PCA9685()
# 주파수를 50hz로 지정
servo_driver.set_pwm_freq(50)

# 서보 모터 채널 설정
servo_channels = [0, 1, 2, 3, 4, 5]  # 각각의 모터에 대한 채널 (예: [0, 1, 2, 3, 4, 5])

pulse_mapping = {   # 순서대로 MAX Pulse, Stop Pulse, Min Pulse / -90 0 90
    0: [108, 305, 502],     #MAX STOP MIN
    1: [107, 304, 501],     #-90 0 90
    2: [109, 306, 503],     #-90 0 90
    3: [113, 310, 507],     #-90 0 90
    4: [100, 305, 510],     #MAX STOP MIN
    5: [250, 303, 430]      #relase? 303 Grab?
}

# 모터 초기화 (정지 상태)
for i in range(6):
    channel = servo_channels[i]
    servo_driver.set_pwm(channel, 0, pulse_mapping[i][1])
    time.sleep(1)
print("pathplanning file excuted")

def BTN_360_set_motor_pwm(index, position, speed=0.5):
    channel = servo_channels[index]
    min_pulse, stop_pulse, max_pulse = pulse_mapping[channel]
    if index !=5:
        if position == 'stop':
            pulse = stop_pulse
        elif position == 'forward':
            pulse = stop_pulse + int(abs(max_pulse - stop_pulse) * speed)
        elif position == 'backward':
            pulse = stop_pulse - int(abs(stop_pulse - min_pulse) * speed)
        else:
            raise ValueError("Invalid position value")
    else:
        if position == 'stop':
            pulse = stop_pulse
        elif position == 'forward':
            pulse = min_pulse
        elif position == 'backward':
            pulse = max_pulse
        else:
            raise ValueError("Invalid position value")
        
    servo_driver.set_pwm(channel, 0, pulse)

# 서모모터 동작 각도와 pulse 값을 맵핑시켜 주기위한 함수
def map(value,min_angle, max_angle, min_pulse,max_pulse) :
     angle_range=max_angle-min_angle
     pulse_range=max_pulse-min_pulse
     scale_factor=float(angle_range)/float(pulse_range)
     return min_pulse+((value-min_angle)/scale_factor)

# 서보모터 각도 이동을 위한 함수
def set_angle(channel,angle):
     pulse=int(map(angle,-90,90,pulse_mapping[channel][0],pulse_mapping[channel][2]))
    #  print(angle, pulse)
     servo_driver.set_pwm(channel,0,pulse)

motorAngle_dic = {
    1 : [0],   #순서대로 currentAngle, goalAngle
    2 : [0],
    3 : [0]
}

turnON = True

def joint_state_callback(msg):
    # /plan_Joints topic에서 수신한 데이터를 처리하는 콜백 함수
    rospy.loginfo("Received JointState: %s", msg)
    print(msg.position)
    scale = 180/math.pi
    set_angle(1,msg.position[1]*scale)
    set_angle(2,msg.position[2]*scale)
    set_angle(3,msg.position[3]*scale)

def ps3_controller_callback(msg):
    # /PS3Controller_bools topic에서 수신한 데이터를 처리하는 콜백 함수
    print("Received PS3Controller bools: %s", msg.data)
    
    btn_L1,btn_L2,btn_R1,btn_R2,btn_select,btn_Mode  = msg.data
    btn_list = []
    motorEventCode_dic = {
    0 : [btn_L1, btn_R1],
    4 : [btn_L2, btn_R2],
    5 : [btn_select, btn_Mode]
    }
    for i in motorEventCode_dic.keys:
        ec_list = motorEventCode_dic[i]
        if len(ec_list) == 2:
            value1, value2 = ec_list
            print(ec_list, value1, value2)
            if value1 == value2:
                if i !=5:
                    BTN_360_set_motor_pwm(i, 'stop')
            else:
                if value1 == 1:
                    BTN_360_set_motor_pwm(i, 'forward',speed=0.2)
                else:
                    BTN_360_set_motor_pwm(i, 'backward',speed=0.2)

def multiple_topic_subscriber():
    rospy.init_node('multiple_topic_subscriber', anonymous=True)
    
    print("main 함수 실행중")
    rospy.Subscriber('/plan_Joints', JointState, joint_state_callback, queue_size=10)
    rospy.Subscriber('/PS3Controller_bools', Int32MultiArray, ps3_controller_callback, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        multiple_topic_subscriber()
# 메인 스레드에서 모터 제어 로직 실행
    except KeyboardInterrupt:
        print("종료 중...")

#!/usr/bin/env python3

import rospy
import time
import RPi.GPIO as GPIO
from airship.msg import Range as RangeMsg

ECHO = 23
TRIG = 24

RATE = 10 # Hz

TIMEOUT = 1

SPEED_OF_SOUND_IN_AIR_DIV2 = 171.50

class SonicRanger:
    def __init__(self):
        GPIO.setup(TRIG,GPIO.OUT)
        GPIO.setup(ECHO,GPIO.IN)

        GPIO.output(TRIG,False)
        self.pub = rospy.Publisher('airship/alt_range', RangeMsg, queue_size=10)
        rospy.init_node('SonicRanger')

    def measure_range_m(self) -> float:
        time_call = time.time()
        GPIO.output(TRIG,True)
        time.sleep(0.00001)
        GPIO.output(TRIG,False)

        while GPIO.input(ECHO) ==0:
            pulse_start = time.time()
            if pulse_start - time_call > TIMEOUT:
                return -1
        while GPIO.input(ECHO) ==1:
            pulse_end = time.time()
            if pulse_end - time_call > TIMEOUT:
                return -1
        pulse_duration = pulse_end - pulse_start
        return (pulse_duration*SPEED_OF_SOUND_IN_AIR_DIV2)


    def run(self):
        rate = rospy.Rate(RATE)
        count = 0
        while not rospy.is_shutdown():
            msg = RangeMsg()
            msg.header.stamp = rospy.Time.now()
            msg.header.seq=count
            msg.range = self.measure_range_m()
            self.pub.publish(msg)
            print("Got Range of ",round(msg.range,4),"m")
            rate.sleep()
    
    def __del__(self):
        GPIO.cleanup([ECHO,TRIG])

if __name__ == '__main__':
    ranger = SonicRanger()
    ranger.run()
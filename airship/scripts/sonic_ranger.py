#!/usr/bin/env python3

import rospy
import time
from gpiozero import DistanceSensor, Device
from airship.msg import Range as RangeMsg
from gpiozero.pins.pigpio import PiGPIOFactory
ECHO = 23
TRIG = 24

RATE = 10 # Hz

Device.pin_factory = PiGPIOFactory()

class SonicRanger:
    def __init__(self):
        self.pub = rospy.Publisher('airship/alt_range', RangeMsg, queue_size=10)
        rospy.init_node('SonicRanger')
        self.sensor = DistanceSensor(echo=ECHO,trigger=TRIG,max_distance=5)

    def measure_range_m(self) -> float:
        rangelist = []
        for x in range(10):
            rangelist.append(self.sensor.distance)
        #subranges = list(dict.fromkeys(rangelist))
        #if len(subranges) < 8:
            #return -1
            #print("Duplicates = ",len(rangelist) - len(subranges))

        return sum(rangelist) / len(rangelist)


    def run(self):
        rate = rospy.Rate(RATE)
        count = 0
        while not rospy.is_shutdown():
            msg = RangeMsg()
            msg.header.stamp = rospy.Time.now()
            msg.header.seq=count
            msg.range = self.measure_range_m()
            if msg.range == -1:
                print("Got bad range value!")
                rate.sleep()
                continue
            
            self.pub.publish(msg)
            print("Got Range of ",round(msg.range,4),"m")
            rate.sleep()
            count+=1



if __name__ == '__main__':
    ranger = SonicRanger()
    ranger.run()

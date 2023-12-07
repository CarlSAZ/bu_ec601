#!/usr/bin/env python3

import rospy
import time
from gpiozero import DistanceSensor
from airship.msg import Range as RangeMsg

ECHO = 23
TRIG = 24

RATE = 10 # Hz


class SonicRanger:
    def __init__(self):
        self.pub = rospy.Publisher('airship/alt_range', RangeMsg, queue_size=10)
        rospy.init_node('SonicRanger')
        self.sensor = DistanceSensor(echo=ECHO,trigger=TRIG,max_distance=5)

    def measure_range_m(self) -> float:
        rangelist = []
        for x in range(10):
            rangelist.append(self.sensor.distnace)
        subranges = list(dict.fromkeys(rangelist))
        if len(subranges) < 8:
            return -1
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


if __name__ == '__main__':
    ranger = SonicRanger()
    ranger.run()
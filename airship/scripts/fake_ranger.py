#!/usr/bin/env python3
import rospy
import time
from airship.msg import AirshipParams
from airship.msg import Range as RangeMsgDef
from std_msgs.msg import Header

PERIOD = 90

RATE_S = 0.5
TARGET_HEIGHT = 2
RANGE_DELTA = 1

class FakeRanger:
    def __init__(self):
        rospy.init_node('fake_ranger',anonymous=True) 
        self.pub = rospy.Publisher('airship/alt_range', RangeMsgDef, queue_size=1)
        self.pub_pilot = rospy.Publisher('airship/pilot_params', AirshipParams, queue_size=1)

        self.fake_time = 0

        print("Finished publisher setup")
        PilotMsg = AirshipParams()
        PilotMsg.height_target_m = TARGET_HEIGHT
        PilotMsg.altitude_control_flag = True
        self.pub_pilot.publish(PilotMsg)
        print("Sent Pilot Msg")

        RangeMsg = RangeMsgDef()
        RangeMsg.header.stamp = self.fake_time
        RangeMsg.range = TARGET_HEIGHT
        self.pub.publish(RangeMsg)
        print("Finished fake ranger init")

    def sendFakeRange(self):
        RangeMsg = RangeMsgDef()
        RangeMsg.header.stamp = self.fake_time
        tdiff = self.fake_time % PERIOD
        if tdiff < 10:
            rdiff = RANGE_DELTA*tdiff/10
        elif tdiff < 20:
            rdiff = RANGE_DELTA
        elif tdiff < 30:
            rdiff = RANGE_DELTA*(30-tdiff)
        elif tdiff < 40:
            rdiff = 0
        elif tdiff < 50:
            rdiff = -RANGE_DELTA*(tdiff-40)
        elif tdiff < 60:
            rdiff = -RANGE_DELTA
        elif tdiff < 70:
            rdiff = -RANGE_DELTA*(70-tdiff)
        else:
            rdiff = 0

        RangeMsg.range = TARGET_HEIGHT + rdiff
        self.pub.publish(RangeMsg)
        print("Sent Fake Range of ",RangeMsg.range,"m")

    def run(self):
        while not rospy.is_shutdown():
            self.sendFakeRange()
            time.sleep(RATE_S)
            self.fake_time += RATE_S


if __name__ == '__main__':
    fake = FakeRanger()
    print("Starting fake ranger")
    fake.run()


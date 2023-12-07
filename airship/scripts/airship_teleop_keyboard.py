#!/usr/bin/env python3

""" Based upon https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py"""

from __future__ import print_function

import threading
import copy
import sys

#import roslib
from select import select
import rospy
from airship.msg import RotorPair, Rotor, AirshipParams


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

g : 
a : 

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""
BASE_PWM = 20
BASE_VERT_PWM = 20
INVERT_LEFT = False
INVERT_RIGHT = False
INVERT_VERT = False
RATE_HZ = 10 # Hz

moveBindings = {
        'k':(0,0,0,0),
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'K':(0,0,0,0),
        'O':(1,0,0,-1),
        'I':(1,0,0,0),
        'J':(0,0,0,1),
        'L':(0,0,0,-1),
        'U':(1,0,0,1),
        '<':(-1,0,0,0),
        '>':(-1,0,0,1),
        'M':(-1,0,0,-1),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
        'T':(0,0,1,0),
        'B':(0,0,-1,0)
    }


AltitudeBindings={
        'a':(-.10), # increment set height by -10cm
        's':(-.01), # increment by 1cm
        'd':(.01),  # increment by 1cm
        'f':(.10), # increment by 10cm
        'A':(-10), # increment set height by -1m
        'S':(-.10), # increment by 10cm
        'D':(.10),  # increment by 10cm
        'F':(1.0), # increment by 1m
}
speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.yaw_publisher = rospy.Publisher('yaw_rotors', RotorPair, queue_size = 1)
        self.alt_publisher = rospy.Publisher('altitude_rotor', Rotor, queue_size = 1)
        self.pilot_publisher = rospy.Publisher('/airship/pilot_params', AirshipParams, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.z_target = 1
        self.enable_alt_control = False
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.yaw_publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.yaw_publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, newx, newy, newz, newth, newspeed, newturn):
        self.condition.acquire()
        self.x = newx
        self.y = newy
        self.z = newz
        self.th = newth
        self.speed = newspeed
        self.turn = newturn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def updateAlt(self,new_ztarget,new_enable):
        self.condition.acquire()
        self.z_target = new_ztarget
        self.enable_alt_control = new_enable
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def convertToYaw(self):
        rospy.loginfo("\nState = [%f,%f,%f] Speed = %f",self.x,self.y,self.z,self.speed)
        rotor_msg = RotorPair()
        left_speed = self.speed*self.x + self.turn*self.th
        right_speed = self.speed*self.x - self.turn*self.th
        rotor_msg.left.pwm = int(abs(left_speed))
        rotor_msg.left.direction = bool(left_speed > 0 != INVERT_LEFT)
        rotor_msg.right.pwm = int(abs(right_speed))
        rotor_msg.right.direction = bool(right_speed > 0 != INVERT_RIGHT)
        return rotor_msg

    def convertToZMotor(self):
        rotor_msg = Rotor()
        z_speed = self.speed*self.z
        rotor_msg.pwm = int(abs(z_speed))
        rotor_msg.direction = bool(z_speed > 0 != INVERT_VERT)
        return rotor_msg



    def run(self):
        rotor_msg = RotorPair()
        last_rotor_msg = RotorPair()
        Zrotor_msg = Rotor()
        pilot_msg = AirshipParams()
        last_pilot_msg = AirshipParams()

        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            rotor_msg = self.convertToYaw()

            Zrotor_msg = self.convertToZMotor()
            pilot_msg.header.stamp = rospy.Time.now()
            pilot_msg.height_target_m = float(self.z_target)
            pilot_msg.altitude_control_flag = bool(self.enable_alt_control)
            self.condition.release()


            if last_pilot_msg != pilot_msg:
                last_pilot_msg = copy.deepcopy(pilot_msg)
                rospy.loginfo(f"Set new altitude control: enabled = {pilot_msg.altitude_control_flag}, height = {pilot_msg.height_target_m :.3g}m")
            self.pilot_publisher.publish(pilot_msg)
            rospy.loginfo("Finished updating pilot!")

            # Publish.
            if last_rotor_msg != rotor_msg:
                last_rotor_msg = copy.deepcopy(rotor_msg)
                rospy.loginfo("\nSet rotor pair pwms: %s, %s, directions: %s, %s", rotor_msg.left.pwm, rotor_msg.right.pwm, rotor_msg.left.direction, rotor_msg.right.direction)
            self.yaw_publisher.publish(rotor_msg)
            rospy.loginfo("Finished updating yaw motors!")

            self.alt_publisher.publish(Zrotor_msg)
            rospy.loginfo("Finished updating messages!")
        # Publish stop message when thread exits.
        rotor_msg.left.pwm = 0
        rotor_msg.right.pwm = 0
        Zrotor_msg.pwm = 0
        Zrotor_msg.direction = 0

        self.yaw_publisher.publish(rotor_msg)
        self.alt_publisher.publish(Zrotor_msg)


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print("Got key:" + key)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", BASE_PWM)
    turn = rospy.get_param("~turn", BASE_PWM)
    speed_limit = rospy.get_param("~speed_limit", 90)
    turn_limit = rospy.get_param("~turn_limit", 1000)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    ztarget = 0
    enable = False

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey(settings, key_timeout)
            if key in moveBindings:
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in AltitudeBindings:
                ztarget = ztarget + AltitudeBindings[key]
            elif key in ['g','G']:
                enable = not enable
            elif key in speedBindings:
                speed = min(speed_limit, speed * speedBindings[key][0])
                turn = min(turn_limit, turn * speedBindings[key][1])
                if speed == speed_limit:
                    print("Linear speed limit reached!")
                if turn == turn_limit:
                    print("Angular speed limit reached!")
                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)
            pub_thread.updateAlt(ztarget,enable)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)

#!/usr/bin/env python3

# based on altitude controller by open airship

## Controls the airship altitude
import rospy
import time
import sys
from sensor_msgs.msg import MagneticField,Imu
from airship.msg import Range, Rotor, AirshipParams
#from airshippi_vicon import testmodule

# Global params
P = 15
I = 0
D = 0

MIN_HEIGHT = 0
INVERT_VERT_PROP = False

class AirshipAltitudeController:
    def __init__(self,set_height_m=1):
        if set_height_m < MIN_HEIGHT:
            set_height_m = MIN_HEIGHT 
        self.set_height_m = set_height_m
        self.height_current_m = set_height_m
        self.pwm_out = 0
        self.direction_out = 0
        self.integral = 0
        self.last_error = 0
        self.last_imu = Imu()
        self.last_range = Range()
        self.enabled = False
        self.z_current = 0
    
        # Initialise node
        rospy.init_node('altitude_controller', anonymous=True)
        # Initialise publisher and subscriber
        self.pub = rospy.Publisher('/altitude_rotor', Rotor, queue_size=1)
        #rospy.Subscriber('bno08x/raw', Imu, self.imu_update_callback, queue_size=1)
        rospy.Subscriber('/airship/alt_range',Range, self.range_update_callback,queue_size=1)
        rospy.Subscriber('/airship/pilot_params',AirshipParams,self.new_params,queue_size=10)
        
        print("Finished Altitude Init")
        rospy.spin()

    def imu_update_callback(self,imu_data: Imu):
        self.last_imu = imu_data

    def new_params(self,pilot_params: AirshipParams):
        self.enabled = pilot_params.altitude_control_flag
        self.set_height_m = pilot_params.height_target_m
        print("Got new altitude: enabled = ",self.enabled," height = ",self.set_height_m,"m")

    def convertRange(self,range):
        return range

    def range_update_callback(self,alt_update: Range):
        
        self.z_current = self.convertRange(alt_update.range)

        tdiff = alt_update.header.stamp - self.last_range.header.stamp

        # Get error
        error = self.set_height_m - self.z_current

        # Get integral
        self.integral += error/tdiff.to_sec()

        # Get derivative
        derivative = (error - self.last_error)*tdiff.to_sec()

        # Get raw motor value
        raw_pwm = (P*error + I*self.integral + D*derivative)
        # Save pwm and direction
        self.direction_out = int(raw_pwm > 0 != INVERT_VERT_PROP)

        # If height is above capacity, turn off the motor
        self.pwm_out = int(max(0, min(raw_pwm, 255)))

        # Update error
        self.last_error = error
        rotor = Rotor(pwm=self.pwm_out, direction=self.direction_out)
        if self.enabled:
            self.pub.publish(rotor)
        print("Got range update; height = ",self.z_current," error = ",error,". New rotor speed = ",raw_pwm)
        
        
    def __del__(self):
        rotor = Rotor(pwm=0, direction=0)
        self.pub.publish(rotor)
        

if __name__ == '__main__':
    alt_control = AirshipAltitudeController(0)

#!/usr/bin/env python3
import yaml
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from sensor_msgs.msg import Joy
from clifford_vesc.msg import CliffordDriveCommand

class joy2command(object):
    def __init__(self):
        rospy.init_node('joy2command', anonymous=True)

        self.last_command_time = rospy.Time.now()
        self.commandPub = rospy.Publisher('/motor_controller/command', CliffordDriveCommand, queue_size=1)
        self.initialized = [False,False]
        self.auto = False
        self.scale = 1
        joy_config = rospy.get_param('joy2command/joystick_config')
        with open(joy_config) as f:
            self.joyConfig = yaml.safe_load(f)
        
        rospy.Subscriber("/joy", Joy, self.callback)
        rospy.Subscriber("/cmd_auto", CliffordDriveCommand, self.auto_callback)
                
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if not hasattr(self,'lastPubTime') or (rospy.Time.now() - self.lastPubTime).to_sec() > 0.5:
                self.pubCommand(CliffordDriveCommand())
            rate.sleep()
    
    def callback(self,data):
        if (rospy.Time.now() - self.last_command_time).to_sec() > 0.5:
            self.initialized = [False,False]
        
        self.last_command_time = rospy.Time.now()
        
        if data.axes[self.joyConfig["L_TRIGGER"]]>0.1:
            self.initialized[0] = True
        if data.axes[self.joyConfig["R_TRIGGER"]]>0.1:
            self.initialized[1] = True
        
        self.auto = data.buttons[self.joyConfig["BUTTON_A"]] == 1
        
        if not data.axes[self.joyConfig["DPAD_HORIZ"]] == 0:
            self.scale = 1
            # print('joy scale: ' + str(self.scale))
        elif data.axes[self.joyConfig["DPAD_VERT"]] > 0:
            self.scale = min(2.0 * self.scale,1.0)
            # print('joy scale: ' + str(self.scale))
        elif data.axes[self.joyConfig["DPAD_VERT"]] < 0:
            self.scale= self.scale/2.0
            # print('joy scale: ' + str(self.scale))
        
        reverseThrot = 0.5 * data.axes[self.joyConfig["L_TRIGGER"]] - 0.5 if self.initialized[0] else 0 
        forwardThrot = 0.5 - 0.5 * data.axes[self.joyConfig["R_TRIGGER"]] if self.initialized[1] else 0
        throt = reverseThrot + forwardThrot
        frontSteer = data.axes[self.joyConfig["R_JOY_HORIZ"]]
        rearSteer = data.axes[self.joyConfig["L_JOY_HORIZ"]]
        if data.buttons[self.joyConfig["BUTTON_RB"]] == 0:
            throt /= 2.0
        if not self.auto:
            command = CliffordDriveCommand()
            command.throttle = throt*self.scale
            command.front_steering = frontSteer
            command.rear_steering = rearSteer
            self.pubCommand(command)
    def auto_callback(self,data):
        if self.auto:
            self.pubCommand(data)
    def pubCommand(self,command):
        if command.header.stamp.secs == 0 and command.header.stamp.nsecs == 0:
            command.header.stamp = rospy.Time.now()
        self.commandPub.publish(command)
        self.lastPubTime = rospy.Time.now()

if __name__ == '__main__':
    joy2command()
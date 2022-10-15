#!/usr/bin/env python3
"""
Copyright 2022 Jonathan Shulgach

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

Contact: Jonathan Shulgach (jonathan@shulgach.com)


"""
import serial
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState

class ArduinoTalker(Node):
    def __init__(self, node_name='arduino_node', rate=1, COM='/dev/ttyACM0', baudrate=9600, verbose=False):
        """This node controls communication to the microcontroller handling stepper motor commands 
        
        """    
        super().__init__(node_name)
        self.rate = rate
        self.COM = COM
        self.baudrate = baudrate
        self.verbose = verbose
        
        self.joint_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_CB, 1)
        self.joint_states = None
                
        # Try to connect to device
        self.ser = None
        self.connected = False
        self.connect()

        self.create_timer(1/self.rate, self.timer_callback) # frequency rate of running commands
    
                
    def connect(self, COM='/dev/ttyACM0', baudrate=9600):
    
        if self.connected:
            self.logger("Already connected to {} with baudrate {}".format(self.COM, self.baudrate))
            return
            
        while not self.connected:
            self.logger("Waiting for connection...")
            self.ser = serial.Serial(COM, baudrate, timeout=5)           
            self.ser.reset_input_buffer()
            self.connected = True

        if self.verbose:
            self.logger("Successful connection to {} with baudrate {}".format(COM, baudrate))


    def joint_state_CB(self, msg):
        # Just get the most recent joint position 
        self.joint_states = msg.position


    def logger(self, msg):
        # Send message to terminal
        self.get_logger().info(str(msg))


    def timer_callback(self):
        # Just write to the microcontroller
        while self.ser:
            # Interpret /joint_state topic and convert to 
            robot_msg = 'M:' + ','.join(self.joint_states)
            self.send(robot_msg)
            self.read()
        
                
    def send(self, msg, msg_type='utf-8'):
        self.ser.write(str(msg)).encode(msg_type)
        

        
    def read(self):
      if (ser.inWaiting() > 0):
        # read the bytes and convert from binary array to ASCII
        data_str = ser.read(ser.inWaiting()).decode('ascii') 
        # print the incoming string without putting a new-line
        # ('\n') automatically after every print()
        print(data_str, end='') 

      # Put the rest of your code you want here
    
      # Optional, but recommended: sleep 10 ms (0.01 sec) once per loop to let 
      # other threads on your PC run during this time. 
      time.sleep(0.01) 



def main(args=None):
    rclpy.init(args=args)
    node = ArduinoTalker()
    
    while rclpy.ok():
        rclpy.spin(node)
        
    # Explicitly destroy node
    node.destroy_node()
    rclpy.shutdown()
    

if __name__=="__main__":
    main()
    
    

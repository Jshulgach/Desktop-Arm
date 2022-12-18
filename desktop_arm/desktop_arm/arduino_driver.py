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
    def __init__(self, node_name='arduino_node', rate=1, COM='/dev/ttyACM0', baudrate=115200, data_topic="/joint_states", verbose=True):
        """ This is the primary node that controls communication to the microcontroller handling stepper
         motor commands 
         
        Parameters:
        -----------
        node_name : str
            Unique name associated with the ROS2 node instance
        rate : int
            Publishing rate for sending data to the connected microcontroller
        COM : str
            The COM port to connect to; specific to OS version, the default assumes raspberry pi running
            Ubuntu 22.04
        baudrate : int
            Communication baudrate to set along with the COM port for successful microcontroller 
            communication 
        data_topic : str
            ROS2 topic to listen to which contains the joint state data of the robot
        verbose : bool
            ENable/disable debugging text output on the terminal
            
        """    
        super().__init__(node_name)
        self.rate = rate
        self.COM = COM
        self.baudrate = baudrate
        self.verbose = verbose
        
        self.joint_sub = self.create_subscription(JointState, data_topic, self.joint_state_CB, 1)
        self.joint_states = None
        self.ser = None
        self.connected = False
                        
        # Try to connect to device
        self.connect(self.COM, self.baudrate)
        self.create_timer(1/self.rate, self.timer_callback) # frequency rate of running commands
    
                
    def connect(self, COM='/dev/ttyACM0', baudrate=115200):
        """ Function that makes a connect attempt given com port and baudrate
        
        Parameters:
        ----------
        COM : str
            The COM port to connect to; specific to OS version, the default assumes raspberry pi running
            Ubuntu 22.04
        baudrate : int
            Communication baudrate to set along with the COM port for successful microcontroller 
            communication 
        
        """
    
        if self.connected:
            self.logger("Already connected to {} with baudrate {}".format(self.COM, self.baudrate))
            return
            
        while not self.connected:
            self.logger("Waiting for connection...")
            self.ser = serial.Serial(COM, baudrate, timeout=0.5)           
            self.ser.reset_input_buffer()
            self.connected = True
            self.logger("Successful connection to {} with baudrate {}".format(COM, baudrate))

    def disconnect(self):
        """ Function to explicitly disconnect from microcontroller if connected and clean serial use 
        to allow new connection 
        """
        if self.ser or self.connected:
            self.ser.close()
            self.ser = None
            self.connected = False
        

    def joint_state_CB(self, msg):
        """ Callback function for the incoming data subscriber. It just gets the joint state data 
        and saves it as a list of the most recent joint position.
        """
        self.joint_states = list(msg.position)


    def logger(self, msg):
        """ Send message to terminal, enforce as string
        """
        self.get_logger().info(str(msg))


    def timer_callback(self):
        """ Callback function for the timer object when triggered to write to the microcontroller
        
        Notes:
        ------
        The mapping of the input assumes a list of integers, floats, or doubles, but should also work 
        with list of strings
            
        """
        if self.ser and self.joint_states:
            robot_msg = 'M:' + ','.join(map(str, self.joint_states)) 
            self.send(robot_msg)
            self.read()


    def send(self, msg, msg_type='ascii'):
        """ function that converts the incoming message into the appropriate encoded data 
        type and write it to the connected serial port
        
        Parameters:
        -----------
        msg : str, int
            Incoming message to send to the arduino
        msg_type : str
            Encoding style for the message to be broken into: 'ascii', 'utf-8', etc.
            
        Notes:
        ------
        The encoding style may change depending on the ubuntu version, as the ROS2 foxy 
        version had self.ser.write(str(msg)).encode(msg_type) before...
        """
        bytes_to_send = bytes(msg, msg_type)
        if self.verbose: self.logger("Sending '{}' to Device at {}".format(msg, self.COM))
        self.ser.write(bytes_to_send)
        

        
    def read(self):
        """ Function that listens for incoming data,  reads the bytes and convert from 
        binary array to the reading form ASCII
    
        """
        if (self.ser.inWaiting() > 0):
            data_str = self.ser.read(self.ser.inWaiting()).decode('ascii') 
            # print the incoming string without putting a new-line
            # ('\n') automatically after every print()
            print(data_str, end='') 
 
        # Optional, but recommended: sleep 10 ms (0.01 sec) once per loop to let 
        # other threads on your PC run during this time. 
        time.sleep(0.01) 



def main(args=None):
    rclpy.init(args=args)
    node = ArduinoTalker()
    
    while rclpy.ok():
        rclpy.spin(node)
        
    # Disconnect from microcontroller and explicitly destroy node
    node.disconnect()
    node.destroy_node()
    rclpy.shutdown()
    

if __name__=="__main__":
    main()
    
    

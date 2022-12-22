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
from threading import Timer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState


class ArduinoTalker(Node):
    def __init__(self, node_name='arduino_node', 
                       rate=4, 
                       COM='/dev/ttyACM0', 
                       baudrate=115200, 
                       verbose=False
                ):
        """This class controls communication to the microcontroller handling stepper 
        motor commands. It reads the data from the '/joint_state' ROS2 topic and converts 
        it into the 'COMMAND:VALUE,VALUE,...' format
        
        Parameters:
        -----------
        node_name : str
            Specific name to register the ROS2 node
        rate : int
            Publishing rate to send data to the microcontroller
        COM : str
            The COM port that the microcontroller is on, usually 'tty/ACM0'
        baudrate : int
            The baudrate setting for microcontroller communication speed
        verbose : bool
            Enable/disable verbose output for debug on the terminal
        
        """    
        super().__init__(node_name)
        self.rate = rate
        self.COM = COM
        self.baudrate = baudrate
        self.verbose = verbose
        
        self.joint_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_CB, 1)
        self.joint_states = [0.0, 0.0, 0.0, 0.0, 0.0]
                
        # Try to connect to device
        self.ser = None
        self.connected = False
        self.connect()
        self.timer = self.create_timer(1/self.rate, self.timer_callback) # frequency rate of running commands

                
    def connect(self, COM='/dev/ttyACM0', baudrate=115200):
        """ Function that attempts to connect to the port with selected baudrate
        
        Parameters:
        -----------
        COM : str
            The COM port that the microcontroller is on, usually 'tty/ACM0'
        baudrate : int
            The baudrate setting for microcontroller communication speed
                
        """  
        if self.connected:
            self.logger("Already connected to {} with baudrate {}".format(self.COM, self.baudrate))
            
        while not self.connected:
            self.logger("Waiting for connection...")
            self.ser = serial.Serial(COM, baudrate, timeout=5)           
            self.ser.reset_input_buffer()
            self.connected = True
            self.logger("Successful connection to {} with baudrate {}".format(COM, baudrate))


    def joint_state_CB(self, msg):
        """ Callback function when new data is published to the '/joint_state' topic
        
        Parameters:
        -----------
        msg : JointState
            ROS2 message used to get the most recent joint position 
        """
        try:
            self.joint_states = list(msg.position)
            #self.logger(self.joint_states)
        except:
            self.logger("error in joint state callback")


    def logger(self, msg):
        # Send message to terminal
        self.get_logger().info(str(msg))


    def timer_callback(self):
        """ Callback function that executes at a specified rate from the objects' 'rate' parameter
        """
        if self.ser and len(self.joint_states)>1:
                #self.logger("joint state message type: {}".format(type(self.joint_states)))
                robot_msg = 'M:' + ','.join(map(str, self.joint_states))
                self.logger("Sending {} to Arduino".format(robot_msg))

                self.send(robot_msg, 'ascii')
                #self.read()
        
                
    def send(self, msg, msg_type='ascii'):
        """ Function that sends a string message to the connected serial port by 
        encoding it into a byte format first
        
        Parameters:
        -----------
        msg : str
            message to send to the serial port
        msg_type : str
            Encoding type for message before sending
        
        """
        self.ser.write(bytes(msg, msg_type))
        

        
    def read(self):
        """ Read the bytes and convert from binary array to ASCII
        """

        if (self.ser.inWaiting() > 0):
            data_str = self.ser.read(self.ser.inWaiting()).decode('ascii') 
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
    try:
        while rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        # Explicitly destroy node
        #node.timer.stop()
        node.destroy_node()
    #rclpy.shutdown()

if __name__=="__main__":
    main()
    
    

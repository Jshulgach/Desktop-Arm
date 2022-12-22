#!/usr/bin/env python3
"""
Copyright 2022 Jonathan Shulgach

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

Contact: Jonathan Shulgach (jonathan@shulgach.com)

"""
import socket
import threading
import sys
import rclpy
import time
import select
from math import modf
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Header



class Listener(Node):
    def __init__(self,
                 node_name='comm_server_node',
                 ip='192.168.1.180',
                 port=10000,
                 publish_topic='/joy',
                 device_name='Microsoft X-Box 360 pad',
                 verbose=True):
        """
        This node handles server or client communication between 2 computers using TCP/IP.

        It creates a publisher for the '/joy' topic using the incomming commands heard from the
        ip address (wired or wireless network) and makes user controls heard on the same wired network..


        Parameters:
        -----------
        node_name : str
            specific node name
        ip : str
            The IP address to use to start the server, usually the address of the localhost
        port : int
            Port number on ip address to allow client connections
        publish_topic : str 
            The ROS2 topic that the controller commands get upblished to after conversion to Joy
        device_name : str
            Device specific option for mapping controller key indices to the Joy message 
        verbose : bool
            Enable/disable verbose output text to the terminal

        Notes:
        ------
        - Only allows one server and one client. In the future it should be able to handle multiple clients.
        - Eventually will use the name "broadcaster" because it's broadcasting its ROS topics

        """
        super().__init__(node_name)
        self.ip = ip # another default could be your PC address on your home network, i.e. '192.168.1.180'
        self.port = port
        self.device_name = device_name
        self.verbose = verbose

        # create publisher for joystick data
        self.joy_pub = self.create_publisher(Joy, publish_topic, 1)
        self.last_publish_time = 0

        # Create a TCP/IP socket for the server to run
        self.connected = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
        #self.sock.setblocking(1)
        #self.sock.settimeout(1)
        server_address = (self.ip, self.port) # Bind the socket to the port. 
        self.sock.bind(server_address)
        self.sock.listen(1) # Listen for incoming connections, maximum one client
        self.logger('Starting server on {}. Waiting for connection...'.format(server_address))

        self.wait_for_client()


    def wait_for_client(self):
        """ Initializes the socket and waits for client connections. Rus until stopped by
        keyboard interrupt or explicit method stop()
    
        """
        
        self.connection, client_address = self.sock.accept() # Wait for a connection.
        self.logger('New connection from {}'.format(client_address))
        self.connected = True
        while True:
            while self.connected:
                data = self.connection.recv(1024).decode() # Grab all available bytes
                if self.verbose: print('received {!r}'.format(data))
                
                if data:
                    self.publishTopicData(data)


    def publishTopicData(self, data):
        """ Function that fills the ROS2 Joy message with the incoming data array and publishes to
        the topic

        Parameters:
        -----------
        data : list
            A list of values corresponding to the controller inputs
        """
        msg = self.convertToJoyMsg(data)
        current_time = modf(time.time())
        msg.header.stamp.sec = int(current_time[1])
        msg.header.stamp.nanosec = int(current_time[0] * 1000000000) & 0xffffffff
        self.joy_pub.publish(msg)
        self.last_publish_time = time.time()
                        

    def convertToJoyMsg(self, str_msg):
        """ Function that converts incoming data into ROS2 'Joy' data type

        Parameters:
        -----------
        str_msg : str
            A long ascii string containing the joystick state for buttons and joysticks

        Return:
        -------
        msg : sensor_msgs.msg.Joy
            The ROS2 data type with the fields filled in matching the incoming data string

        Notes:
        ------
        It'ss a custom mapping from the string input to the ROS2 Joy message. The current
        order of controller inputs being read are specific to the controller type, in this
        case the Xbox controller.
    
        """
        split_str = str_msg.split(',')
        data = list(map(float, split_str))

        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()

        # Hard coded for xbox controller for now...
        joy_msg.axes = data[0:7] # elements should be type float here
        joy_msg.buttons = list(map(int, data[8:-1])) # but buttons should be type int
        return joy_msg

    def logger(self, msg):
        """ Send message to terminal
        """
        self.get_logger().info(str(msg))


    def disconnect(self):
        """ Disconnect from any clients and clean up the connection.
        """
        self.connection.close()

    def stop(self):
        """ Explicit command to stop the controller object
        """
        self.disconnect()



def main(args=None):

    #node = None
    rclpy.init(args=args)
    node = Listener()
    try:
        while rclpy.ok():
            rclpy.spin(node)

    except KeyboardInterrupt:
        print("Oh! you pressed CTRL + C. Program interrupted..")
    finally:   
        # Explicitly destroy node
        node.disconnect()
        node.destroy_node()
        rclpy.shutdown()
    
if __name__=="__main__":
    main()

    

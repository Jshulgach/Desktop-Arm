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
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState


class Listener(Node):
    def __init__(self, node_name='wireless_comm_node', ip='localhost', port=10000, control_type='client', verbose=True):
        """
        This node handles server or client communication between 2 computers using
        TCP/IP.

        Potentially, It's like an extension to the ROS communication between topics on the
        same wired network, but this helps with making them seen wirelessly.


        Parameters:
        -----------
        node_name : str
            specific node name
        ip : str
            The IP address to use to start the server
        port : int
            Port number on ip address to allow client connections
        control_type : str
            controlling type option for the class
        verbose : bool
            Enable/disable verbose output text to the terminal

        Notes:
        ------
        - Only allows one server and one client. In the future it should be able to handle multiple clients.
        - Eventually will use the name "broadcaster" because it's broadcasting its ROS topics

        """
        super().__init__(node_name)
        self.control_type = control_type
        self.ip = ip # another default could be your PC address on your home network, i.e. '192.168.1.180'
        self.port = port
        self.verbose = verbose
        self.connection = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Create a TCP/IP socket. 

        server_address = (self.ip, self.port) # Bind the socket to the port. 
        self.sock.bind(server_address)
        if verbose: print('Starting up server on {} port {}'.format(server_address, self.port))

        self.wait_for_client()



    def wait_for_client(self):
        """ Initializes the socket and waits for client connections. 
    
        """
        # Listen for incoming connections.
        self.sock.listen(10)
  
        while True:
            # Wait for a connection.
            print('waiting for a connection...')
            self.connection, client_address = self.sock.accept()

            try:
                print('connection from', client_address)
                # Receive the data in small chunks and retransmit it.
                while True:
                    data = self.connection.recv(1024)
                    print('received {!r}'.format(data))

            finally:
                pass

    def disconnect(self):
        # Clean up the connection.
        self.connection.close()

def main(args=None):

    node = None
    rclpy.init(args=args)
    node = NetworkTalker()
    try:

        while rclpy.ok():
            rclpy.spin(node)

    except KeyboardInterrupt:
        print("Oh! you pressed CTRL + C.")
        print("Program interrupted.")
        
    # Explicitly destroy node
    node.disconnect()
    node.destroy_node()
    rclpy.shutdown()
    
if __name__=="__main__":
    main()

    

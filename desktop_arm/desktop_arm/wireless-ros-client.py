import socket
import sys
from threading import Timer
from xbox_control import XboxController

class Operator():
    def __init__(self, node_name='xbox_controller_broadcaster', rate=1, ip='localhost', port=10000, control_type='client', verbose=True):
        """
        This node handles server or client communication between 2 computers using
        TCP/IP. It's like an etension to the ross comunication between topics on the
        same wired network, but this helps with making them seen wirelessly.


        Parameters:
        -----------
        node_name : str
            specific node name
        rate : int
            Frequency of publishing rate for data to IP address
        ip : str
            The IP address to use to start the server
        port : int
            Port number on ip address to allow client connections
        control_type : str
            controlling type option for the class
        verbose : bool
            Enable/disable verbose output text to the terminal

        """
        self.node_name = node_name
        self.rate = rate
        self.ip = ip
        self.port = port
        self.verbose = verbose
        self.timer = Timer(1/self.rate, function=self.run_transmitter)

        # Create controller object with button event handling
        self.joy = XboxController()

        # Attempt to connect to server
        self.connect()

    def run_transmitter(self):
       data = self.joy.read()
       #message = b'This is the message.  It will be repeated.'
       message  = bytes(','.join(map(str, data)),'ascii')
       print('sending {!r}'.format(message))
       self.sock.sendall(message)
       # Look for the response.
       #amount_received = 0
       #amount_expected = len(message)

       #while amount_received < amount_expected:
       #    data = self.sock.recv(16)
       #    amount_received += len(data)
       #    print('received {!r}'.format(data))


    def connect(self):
        """ Function that makes an attempt to connect the socket to the port where the server
        is listening.If successful, it starts the timer object to send data to IP address at specified rate

        """
            
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Create a TCP/IP socket.
        server_address = (self.ip, self.port)
        self.sock.connect(server_address)
        print('connecting to {} port {}'.format(server_address, self.port))

        self.timer.start()



    def disconnect(self):
            """ Function to explicitly disconnect from microcontroller if connected and clean serial use 
            to allow new connection 
            """
            if self.sock:
                self.sock.close()
                print('closing socket')


if __name__ == '__main__':
    
    try:
        operator = Operator()
        while True:
            operator.run()
    except KeyboardInterrupt:
        print("Oh! you pressed CTRL + C.")
        print("Program interrupted.")
    finally:
        print("This was an important code, ran at the end.")
        

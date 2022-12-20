import socket
import sys
from threading import Timer
from xbox_control import XboxController


class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False
        

class ControllerClient():
    def __init__(self,
                 node_name='xbox_controller_broadcaster',
                 rate=1,
                 ip='localhost',
                 port=10000,
                 control_type='Microsoft X-Box 360 pad',
                 verbose=True
                 ):
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
        self.connected = False
        self.controller_exists = False
        self.timer = RepeatedTimer(1/self.rate, function=self.timer_callback)

        # Create controller object with button event handling
        self.xbox = XboxController()
        if self.verbose: print("Joystick control enabled. Connect a joystick controller to transmit...")

        # Attempt to connect to server
        if self.verbose: print('Connecting to {} port {}'.format(self.ip, self.port))
        self.connect(self.ip, self.port)


    def timer_callback(self):
        """ A callback function that gets called every "1/rate" seconds
        The function reads the current state of the joystick and sends it as a string to the
        connected ip address and port

        """
        if len(self.xbox.gamepads) > 0:
        
            data = self.xbox.read()    
            message  = bytes(','.join(map(str, data)),'ascii')
            
            try:
                self.sock.send(message)
                if self.verbose: print('sending {!r}'.format(message))
            
            except KeyboardInterrupt:
                if self.verbose: print("KeyboardInterrupt detected")
                self.connected = False
                self.timer.stop()        

            except socket.error:
                if self.verbose: print('Connection to the server lost, attempting to reconnect...')
                self.connected = False
                self.timer.stop()
                self.connect(self.ip, self.port)
    

    def connect(self, ip='192.168.1.180', port=10000):
        """ Function that makes an attempt to connect the socket to the port where the server
        is listening. If successful, it starts the timer object to send data to IP address at specified rate

        Parameters:
        -----------
        ip : str
            The IP address that the server is on
        port : int
            Port number specific to server

        """
        while not self.connected:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Create a TCP/IP socket.
                server_address = (ip, port)
                self.sock.connect(server_address)
                self.connected = True
                if self.verbose: print('Connection to server at {} successful'.format(server_address))
                self.timer.start()
    
            except socket.error:
                self.connected = False
                self.timer.stop()
                #if self.verbose: print('Connection to the server lost, attempting to reconnect...')
                #self.connect(self.ip, self.port)

        
    def disconnect(self):
        """ Function to explicitly disconnect from microcontroller if connected and clean serial use 
        to allow new connection 
        """
        if self.sock:
            self.sock.close()
            if self.verbose: print('closing socket')

    def stop(self):
        """ Explicit command to stop the client
        """
        self.timer.stop()
        self.disconnect()


if __name__ == '__main__':
    
    try:
        xbox = ControllerClient()
        while True:
            pass
    except KeyboardInterrupt:
        print("Oh! you pressed CTRL + C. Program interrupted...")
    finally:
        xbox.stop()        

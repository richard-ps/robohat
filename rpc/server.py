#
#   Hello World server in Python
#   Binds REP socket to tcp://*:5555
#   Expects b"Hello" from client, replies with b"World"
#

from socket import socket
from symtable import Class
import zmq

class ZeroMQServer:

    def __init__(self):
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.bind("tcp://*:5556")

    def listen(self):
        #  Wait for next request from client
        message = self.socket.recv()
        print(f"Received request: {message}")
        #  Send reply back to client
        self.socket.send(b"Message received")
        return message
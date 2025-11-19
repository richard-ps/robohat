#
#   Hello World client in Python
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Hello" to server, expects "World" back
#

from pyclbr import Class
from socket import socket
import zmq

class ZeroMQClient:

    def __init__(self):
        context = zmq.Context()
        #  Socket to talk to server
        print("Connecting to hello world server…")
        self.socket = context.socket(zmq.REQ)
        self.socket.connect("tcp://10.15.3.174:5556")

    def send_request(self, command):
        print(f"Sending command [{command}] …")

        self.socket.send(command.encode())

        #  Get the reply.
        message = self.socket.recv()
        print(f"Received reply [ {message} ]")

    def run(self):
        commands = ["WAIT", "ROAM", "TARGET_FOUND",
                    "WALK", "SEARCH", "CHARGER_FOUND", "EXIT"]
        
        self.print_menu()

        while True:
            command_index = int(input())
            command = commands[command_index - 1]

            if command_index == 0:
                break

            if command_index not in range(1,7):
                print("Invalid command. Please try again.")
                self.print_menu()
        
            self.send_request(command)

    def print_menu(self):
        print("Select a command to send to the server:")
        print("1. WAIT")
        print("2. ROAM")
        print("3. TARGET_FOUND")
        print("4. WALK")
        print("5. SEARCH")
        print("6. CHARGER_FOUND")
        print("0. Exit")

if __name__ == "__main__":
    client = ZeroMQClient()
    client.run()

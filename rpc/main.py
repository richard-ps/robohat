from server import ZeroMQServer
from client import ZeroMQClient
from fsm import FSM
import sys

def start_server():
    server = ZeroMQServer()
    server.listen()

def start_client():
    client = ZeroMQClient()
    client.run()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python main.py [server|client]")
        sys.exit(1)

    role = sys.argv[1].lower()
    if role == "server":
        start_server()
    elif role == "client":
        start_client()
    elif role == "fsm":
        fsm = FSM()
        fsm.search()
    else:
        print("Invalid role. Use 'server' or 'client'.")
        sys.exit(1)

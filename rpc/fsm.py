import os, sys

currDir = os.path.dirname(os.path.realpath(__file__))
rootDir = os.path.abspath(os.path.join(currDir, '..'))
if rootDir not in sys.path: # add parent dir to paths
    sys.path.append(rootDir)

from rpc.server import ZeroMQServer
from rpc.controller.na_cpg import NaCPG, create_fully_connected_adjacency
import torch
import matplotlib.pyplot as plt
# from SerTest import SerTestClass
import time
import math

try:
    from robohatlib.Robohat import Robohat
    from robohatlib import RobohatConstants
    from robohatlib.hal.assemblyboard.PwmPlug import PwmPlug
    from robohatlib import RobohatConfig
    from robohatlib.hal.assemblyboard.ServoAssemblyConfig import ServoAssemblyConfig
    from robohatlib.hal.assemblyboard.servo.ServoData import ServoData
    from robohatlib.hal.datastructure.Color import Color
    from robohatlib.hal.datastructure.ExpanderDirection import ExpanderDir
    from robohatlib.hal.datastructure.ExpanderStatus import ExpanderStatus
    from robohatlib.driver_ll.datastructs.IOStatus import IOStatus

    from testlib import TestConfig

except ImportError:
    print("Failed to import all dependencies for SerTestClass")
    raise

possible_states = ["WAIT", "ROAM", "TARGET_FOUND",
                   "WALK", "SEARCH", "CHARGER_FOUND"]

class FSM:
    def __init__(self):
        self.state = "WAIT"

        self.robohat = Robohat(TestConfig.SERVOASSEMBLY_1_CONFIG, TestConfig.SERVOASSEMBLY_2_CONFIG, TestConfig.TOPBOARD_ID_SWITCH)
        self.robohat.init(TestConfig.SERVOBOARD_1_DATAS_LIST, TestConfig.SERVOBOARD_2_DATAS_LIST)
        self.robohat.do_buzzer_beep()

        adj_dict = create_fully_connected_adjacency(8)
        self.controller = NaCPG(adj_dict, angle_tracking=True)

        print(f"Initial state: {self.state}")

    def transition(self, new_state):
        if new_state in possible_states:
            print(f"Transitioning from {self.state} to {new_state}")
            self.state = new_state
            self.execute_state_action()
        else:
            print(f"Invalid state transition attempted: {new_state}")

    def run(self):
        server = ZeroMQServer()
        
        while True:
            print("Waiting for command from client...")
            command = server.listen()

            print("Received command:", command)
            if command == "WAIT":
                print("In WAIT state")
            elif command == "ROAM":
                print("In ROAM state")
            elif command == "TARGET_FOUND":
                print("In TARGET_FOUND state")
            elif command == "WALK":
                print("In WALK state")
            elif command == "SEARCH":
                print("In SEARCH state")
                self.search()
            elif command == "CHARGER_FOUND":
                print("In CHARGER_FOUND state")

    def execute_state_action(self):
        if self.state == "WAIT":
            self.wait()
        elif self.state == "ROAM":
            self.roam()
        elif self.state == "TARGET_FOUND":
            self.target_found()
        elif self.state == "WALK":
            self.walk()
        elif self.state == "SEARCH":
            self.search()
        elif self.state == "CHARGER_FOUND":
            self.charger_found()

    def wait(self):
        pass

    def roam(self):
        pass

    def target_found(self):
        pass

    def walk(self):
        params = {'phase': [ 1.84770147,  0.63517527, -6.2581397 ,  5.79072771,  3.92560091,
       -3.12365597, -5.64161065,  1.61769557], 'w': [ -3.82029392, -10.09482901,   7.54136557,  -9.10054186,
        -9.74108929,  16.49676456,  22.68651804,  12.8404796 ], 'amplitudes': [ 0.41219116,  4.24333571, -4.81600483, -2.72279552,  3.86351227,
        4.13802726, -5.23314684, -4.10500115], 'ha': [0.73259639, 0.84562198, 0.60696611, 0.74627439, 0.26942003,
       0.16558826, 0.04662044, 0.05292187], 'b': [-20.68912344,  59.14947598,  50.71959856,  15.96840235,
       -88.9128633 ,  33.98478395, -33.86570571,  85.19837797]}

        self.controller.set_param_with_dict(params)

        # b has been added, but not in use because current parameters were trained with previous version of NA CPG
        angles = [.0]*16  # Initialize with dummy values

        update_count = 0
        while True:
            angles_radians = self.controller.forward(3000.0).tolist()
            angles_degrees = [int((angle + math.pi) / (2 * math.pi) * 180) for angle in angles_radians]
            angles[:8] = angles_degrees
            print("Servo angles (degrees):", angles)
            self.robohat.update_servo_data_direct(angles)
            update_count += 1
            time.sleep(.1)
            if update_count % 10 == 0:
                return

    def search(self):
        params = {'phase': [-6.28318531, -1.14581646,  0.44064909, -1.52731181,  1.43443942,
        6.28318531,  4.0185173 ,  6.28318531], 'w': [ -1.70297506, -11.52369599,   1.03248715,  -3.81787687,
        -0.15725899,  -1.96215584,   0.37505434,  -0.15555631], 'amplitudes': [-1.57079633,  0.82234301, -1.57079633, -0.23272299, -1.30865045,
        1.05665486, -0.59531369, -1.3773699 ], 'ha': [0.02582832, 0.01784225, 0.02252759, 0.01019621, 0.00621575,
        0.00118579, 0.03031356, 0.01800144], 'b': [ 1.29011006e-01,  1.00000000e+02, -1.00000000e+02,  4.83189888e-01,
        1.00000000e+02, -1.51697539e-02, -8.23871361e-01, -2.00560108e+00]}

        self.controller.set_param_with_dict(params)

        # b has been added, but not in use because current parameters were trained with previous version of NA CPG
        angles = [.0]*16  # Initialize with dummy values

        update_count = 0
        while True:
            angles_radians = self.controller.forward(3000.0).tolist()
            angles_degrees = [int((angle + math.pi) / (2 * math.pi) * 180) for angle in angles_radians]
            angles[:8] = angles_degrees
            print("Servo angles (degrees):", angles)
            self.robohat.update_servo_data_direct(angles)
            update_count += 1
            time.sleep(.1)
            if update_count % 10 == 0:
                return


    def charger_found(self):
        pass


                
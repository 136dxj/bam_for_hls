# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0
import sys
import os
import math
sys.path.append("/media/jdx/jdx/FTServo_Python")
sys.path.append("/media/jdx/jdx/bam/bam")
import json
import datetime
import argparse
import time
# from .etherban import Client
from scservo_sdk import *                   # Uses FTServo SDK library
from trajectory import *

arg_parser = argparse.ArgumentParser()
# arg_parser.add_argument("--host", type=str, default="127.0.0.1")
# arg_parser.add_argument("--offset", type=float, required=True, help="Offset in radians for the zero position")
arg_parser.add_argument("--mass", type=float, required=True)
# arg_parser.add_argument("--arm_mass", type=float, required=True)
arg_parser.add_argument("--length", type=float, required=True)
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--trajectory", type=str, default="sin_time_square")
arg_parser.add_argument("--motor", type=str, required=True)
arg_parser.add_argument("--kp", type=int, default=10.0)
arg_parser.add_argument("--damping", type=float, required=True)
args = arg_parser.parse_args()

if args.trajectory not in trajectories:
    raise ValueError(f"Unknown trajectory: {args.trajectory}")

def angle_wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

trajectory = trajectories[args.trajectory]

# eth = Client(args.host)
portHandler = PortHandler('/dev/ttyUSB0') #ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = hls(portHandler)
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baudrate 1000000
if portHandler.setBaudRate(1000000):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

# eth.run_background()
packetHandler.SetPosKp(1, int(args.kp))
packetHandler.SetPosKd(1, int(args.damping))
# packetHandler.reSet(1)
packetHandler.reOfsCal(1, 0)
goal_position, torque_enable = trajectory(0)
# eth.wait_stability(0)
# eth.goto_safe(0, args.offset + goal_position)

start = time.time()
data = {
    "mass": args.mass,
    "arm_mass": 0,
    "length": args.length,
    "kp": args.kp,
    "damping": args.damping,
    "motor": args.motor,
    "trajectory": args.trajectory,
    "entries": []
}

while time.time() - start < trajectory.duration:
    t = time.time() - start
    goal_position, torque_enable = trajectory(t) ###rad
    goal_position1 = goal_position * 180.0 / math.pi ### rad2degree
    print(goal_position1)
    if torque_enable:
        # eth.position_control(0, args.offset + goal_position, 0.0, args.kp, args.damping)
        packetHandler.WritePosEx(1, int(goal_position1/0.087), 1000, 0, 1000)
    else:
        # eth.set_order(0, "torque", 0.0)
        packetHandler.DisableMotor(1)
    # eth.sync()
    scs_present_position, scs_present_speed, scs_comm_result, scs_error = packetHandler.ReadPosSpeed(1)
    scs_present_current, scs_comm_result, scs_error = packetHandler.ReadCurrent(1)
    scs_present_position = scs_present_position * 0.087 / 180 * math.pi ###degree2rad
    # status = eth.get_statuses()[0]
    entry = {
        "position": scs_present_position,
        "speed": scs_present_speed * 0.732 * 0.104719753,
        # "torque_demand": status["torque_demand"],
        "control": scs_present_current * 6.5 * 1e-4,
        "timestamp": time.time() - start,
        "goal_position": goal_position,
        "torque_enable": torque_enable,
    }

    data["entries"].append(entry)

# eth.set_order(0, "torque", 0.0)
# eth.stop()
packetHandler.DisableMotor(1)

#Format YYYY-MM-DD_HH:mm:ss"
date = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")

filename = f"{args.logdir}/{date}.json"
json.dump(data, open(filename, "w"))

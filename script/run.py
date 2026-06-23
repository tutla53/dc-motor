# Pico Script

import time
import os

import Board.Pico as Board
import BasicFunction.Motor as Motor
import Config.Motor0 as Motor0Config
import Config.Motor1 as Motor1Config
import Tool.plotter
import Tool.FileProcessing

from Features.MotorControl import MotorControl
from Tool.MotorSim import SystemModel
from Tool.visualize import *
from base_url import *

# -------------------------------------- Initialization -------------------------------------- #
yaml_path = base_url+"/DeviceOpFuncs/DCMotor.toml"
p = Board.Pico(yaml_path)

m0 = MotorControl(move_motor = Motor.MoveMotor(device = p, configfile = Motor0Config))
m1 = MotorControl(move_motor = Motor.MoveMotor(device = p, configfile = Motor1Config))

# -------------------------------------- Playground -------------------------------------- #
# TODO: Create the Information Guide
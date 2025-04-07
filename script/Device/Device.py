import Board.Pico as Board
import FWLogger.FWLogger as Logger
import BasicFunction.Motor as Motor
import Config.Motor0 as config

yaml_path = "YAML/commands.yaml"
p = Board.Pico(yaml_path)
motor0 = Motor.MoveMotor(device = p, configfile = config)
logger = Logger.FWLogger(p)
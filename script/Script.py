import Device.Pico as Pico
import FWLogger.FWLogger as Logger
import time

yaml_path = "YAML/commands.yaml"
p = Pico.Device(yaml_path, "0")
logger = Logger.FWLogger(p)

def test_log():
    logger.run()
    print(p.get_motor_pos_pid_param(0))
    time.sleep(5)
    logger.stop()
    time.sleep(0.1)
    logger.print_logged_data()

def motor_test(speed_rpm):
    speed = int ((speed_rpm * 48.4)/60)
    logger.run(5, 10)
    p.move_motor_speed(0, speed)
    time.sleep(3)
    logger.stop()
    p.stop_motor(0)

def trapezoid_test(position_rotation, speed, acc, duration=3):
    position = int(position_rotation * 48.4)
    logger.run(5, 5)
    p.move_motor_abs_pos_trapezoid(0, position, speed, acc)
    time.sleep(duration)
    logger.stop()
    p.stop_motor(0)
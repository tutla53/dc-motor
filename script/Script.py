from Device.Device import *
import time

def speed_test(speed_rpm, time_sampling = 5):
    logger.run(time_sampling=time_sampling, mask=10)
    motor0.move_motor_speed(speed_rpm)
    time.sleep(3)
    logger.stop()
    p.stop_motor(0)

def pos_trapezoid_test(position_rotation, speed, acc, duration=3, time_sampling=5):
    logger.run(time_sampling=time_sampling, mask=5)
    motor0.move_motor_pos_trapezoid(position_rotation, speed, acc)
    time.sleep(duration)
    logger.stop()
    p.stop_motor(0)

def pos_step_test(position_rotation, duration=3, time_sampling=5):
    logger.run(time_sampling=time_sampling, mask=5)
    motor0.move_motor_pos_step(position_rotation)
    time.sleep(duration)
    logger.stop()
    p.stop_motor(0)
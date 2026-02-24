import time
import math
import FWLogger.FWLogger as Logger

from Tool.visualize import *

class MoveMotor:
    def __init__(self, device, configfile):
        self.dev = device
        self.config = configfile
        self.motor_id = configfile.motor_id
        self.logger = Logger.FWLogger(device)

    def __wait_move_done(self, timeout=180):
        start = time.perf_counter()
        elapsed = 0
        self.dev.event_queue.clear()
        
        while elapsed < timeout:
            if len(self.dev.event_queue) > 0:
                event = self.dev.event_queue.pop()
                event_str = "MOVE_MOTOR_DONE"
                if event_str in event:
                    if event[event_str] == self.motor_id:
                        printg(f"Move Done, Elapsed: {elapsed:.2f} s")
                        return True
            time.sleep(0.1)
            elapsed = time.perf_counter() - start

        printr("[TIMEOUT] waiting for move done.")
        return False
    
    def start_log(self, mask, time_sampling,):
        self.logger.run(motor_id = self.motor_id, mask=mask, time_sampling=time_sampling)
    
    def stop_log(self):
        self.logger.stop(self.motor_id)
    
    def move_motor_speed(self, speed_rpm):
        # Move Motor Speed in RPM
        speed = int ((speed_rpm * 48.4)/60)
        self.dev.move_motor_speed(self.motor_id, speed)

    def move_motor_pos_step(self, position_rotation, wait_move = True):
        # Move Motor Position in Rotation with Step Profile
        current_pos = self.get_motor_pos()['pos_rotation']
        printy(f"Current Pos: {current_pos:.2f}")

        position = int(position_rotation * 48.4)
        self.dev.move_motor_abs_pos(self.motor_id, position)
        if wait_move:
            self.__wait_move_done()

    def move_motor_pos_trapezoid(self, position_rotation, speed_rpm, acc_rpm, wait_move = True):
        # Move Motor Position in Rotation with Trapezoidal Profile
        current_pos = self.get_motor_pos()['pos_rotation']
        duration = self.calculate_motion_time(position_rotation-current_pos, speed_rpm, acc_rpm)

        printy(f"Current Pos: {current_pos:.2f}")
        printy(f"Duration (est): {duration:.2f} s")

        position = int(position_rotation * 48.4)
        speed = int ((speed_rpm * 48.4)/60)
        acc = int ((acc_rpm * 48.4)/60)
        self.dev.move_motor_abs_pos_trapezoid(self.motor_id, position, speed, acc)
        if wait_move:
            self.__wait_move_done(duration+30)

    def move_motor_open_loop(self, pwm):
        # Move Motor Speed Open Loop
        self.dev.move_motor_open_loop(self.motor_id, pwm)

    def stop_motor(self):
        # Stop the Motor
        self.dev.stop_motor(self.motor_id)
    
    def get_motor_pos(self):
        # Get Motor Position 
        pos = self.dev.get_motor_pos(self.motor_id)
        pos['pos_rotation'] = pos['pos_count']/48.4
        return pos
    
    def get_motor_speed(self):
        # Get Motor Speed
        speed  = self.dev.get_motor_speed(self.motor_id)
        speed['speed_rpm'] = (speed['speed_cps']*60)/48.4
        return speed

    def calculate_motion_time(self, target_distance, speed_rpm, acceleration_rpm_per_s):
        # Convert RPM and RPM/s to rotations per second and rotations per second squared
        v_max = speed_rpm / 60.0  # rotations per second
        a_max = acceleration_rpm_per_s / 60.0  # rotations per second squared
        
        displacement_abs = abs(target_distance)
        
        # Handle edge cases with zero acceleration or zero velocity
        if a_max <= 0:
            if v_max <= 0:
                return 0.0  # No movement possible
            else:
                return displacement_abs / v_max
        
        # Calculate minimum distance required for trapezoidal profile
        d_min = (v_max ** 2) / a_max
        
        if displacement_abs >= d_min:
            # Trapezoidal profile
            t_acc = v_max / a_max
            t_coast = (displacement_abs - d_min) / v_max
            t_total = 2 * t_acc + t_coast
        else:
            # Triangular profile
            t_acc = math.sqrt(displacement_abs / a_max)
            t_total = 2 * t_acc
        
        return t_total
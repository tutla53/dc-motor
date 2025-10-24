class MoveMotor:
    def __init__(self, device, configfile):
        self.dev = device
        self.config = configfile
        self.motor_id = configfile.motor_id

    # Move Motor Speed in RPM
    def move_motor_speed(self, speed_rpm):
        speed = int ((speed_rpm * 48.4)/60)
        self.dev.move_motor_speed(self.motor_id, speed)

    # Move Motor Position in Rotation with Step Profile
    def move_motor_pos_step(self, position_rotation):
        position = int(position_rotation * 48.4)
        self.dev.move_motor_abs_pos(self.motor_id, position)

    # Move Motor Position in Rotation with Trapezoidal Profile
    def move_motor_pos_trapezoid(self, position_rotation, speed_rpm, acc_rpm):
        position = int(position_rotation * 48.4)
        speed = int ((speed_rpm * 48.4)/60)
        acc = int ((acc_rpm * 48.4)/60)
        self.dev.move_motor_abs_pos_trapezoid(self.motor_id, position, speed, acc)

    # Move Motor Speed Open Loop
    def move_motor_open_loop(self, pwm):
        self.dev.move_motor_open_loop(self.motor_id, pwm)

    def stop(self):
        self.dev.stop_motor(self.motor_id)
    
    def get_motor_pos(self):
        pos = self.dev.get_motor_pos(self.motor_id)
        pos['pos_rotation'] = pos['pos_count']/48.4
        return pos
    
    def get_motor_speed(self):
        speed  = self.dev.get_motor_speed(self.motor_id)
        speed['speed_rpm'] = (speed['speed_cps']*60)/48.4
        return speed
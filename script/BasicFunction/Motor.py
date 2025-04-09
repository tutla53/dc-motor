class MoveMotor:
    def __init__(self, device, configfile):
        self.dev = device
        self.config = configfile
        self.motor_id = configfile.motor_id

    def move_motor_speed(self, speed_rpm):
        """
        Move Motor Speed in RPM
        """
        speed = int ((speed_rpm * 48.4)/60)
        self.dev.move_motor_speed(self.motor_id, speed)

    def move_motor_pos_step(self, position_rotation):
        """
        Move Motor Position in Rotation with Step Profile
        """
        position = int(position_rotation * 48.4)
        self.dev.move_motor_abs_pos(self.motor_id, position)

    def move_motor_pos_trapezoid(self, position_rotation, speed_rpm, acc_rpm):
        """
        Move Motor Position in Rotation with Trapezoidal Profile
        """
        position = int(position_rotation * 48.4)
        speed = int ((speed_rpm * 48.4)/60)
        acc = int ((acc_rpm * 48.4)/60)
        self.dev.move_motor_abs_pos_trapezoid(self.motor_id, position, speed, acc)
    
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
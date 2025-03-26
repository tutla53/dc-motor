import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import threading
import math
import time
import csv

"""
USB Handler
Available Command:
    - start_logger
        - op: 1
        - args: [time_samplng: u64, logmask: u32]
    - stop_logger
        - op: 2
        - args: None
    - move_motor_speed
        - op: 3
        - args: [motor_id: u8, speed: i32]
    - move_motor_abs_pos
        - op: 4
        - args: [motor_id: u8, pos: i32]
    - stop_motor
        - op: 5
        - args: [motor_id: u8]
    - set_motor_pos_pid_param
        - op: 6
        - args: [motor_id: u8, kp: f32, ki: f32, kd:f32]  
    - get_motor_pos_pid_param
        - op: 7
        - args: [motor_id: u8]
        - return: [kp: f32, ki: f32, kd:f32]
    - set_motor_speed_pid_param
        - op: 8
        - args: [motor_id: u8, kp: f32, ki: f32, kd:f32]  
    - get_motor_speed_pid_param
        - op: 9
        - args: [motor_id: u8]
        - return: [kp: f32, ki: f32, kd:f32]
"""

class SThread:
    def __init__(self,func,daemon,*args,runimmidietly=True,withlock=True):
        self.func = func
        self.arg = args
        self.daemon = daemon
        self.thread = None
        self.lock = threading.Lock()
        self.withlock = withlock
        if runimmidietly:self.run(*self.arg ,withlock=withlock)
        
    def run(self,*args,withlock=False):
        if withlock:
            with self.lock:

                self.thread = threading.Thread(target=self.func, args=(*args,))
                self.thread.daemon = self.daemon
                self.thread.start()
        else:
           #Q args = (*args,)
            #print (*args,"args")
            self.thread = threading.Thread(target=self.func, args=(*args,))
            self.thread.daemon = self.daemon
            self.thread.start()


    def stop(self):
        self.thread.join()

class Device:
    def __init__(self):
        self.ser = None
        self.newinput = None
        self.defaultCOM = "0"
        self.baud_rate = 115200
        self.lock = threading.RLock()  # Use RLock for reentrant locking

    def connect(self, port):
        if port is None:
            port = self.defaultCOM
            self.newinput = self.defaultCOM
        port = "/dev/ttyACM" + port
        stat = True

        try:
            with self.lock:
                self.ser = serial.Serial(port, self.baud_rate, timeout=1)
                time.sleep(1)
                print(f"Connected to {port}")
                return True
        except:
            self.newinput = input(f"Failed to connect to {port}. Input new COM Port /enter to end: ")
            stat = False
        finally:
            if self.newinput == "":
                return False
            if not stat:
                self.defaultCOM = self.newinput
                self.connect(self.newinput)

    def send_cmd(self, cmd, print_echo=True):
        with self.lock:  # Use the RLock to synchronize
            try:
                self.ser.write(cmd)
                time.sleep(0.1)

                if print_echo:
                    while self.ser.in_waiting:
                        data = self.ser.readline()
                        print(data.decode("UTF-8").strip())

            except serial.SerialException as e:
                print(f"Serial error: {e}")
            except Exception as e:
                print(f"Unexpected error: {e}")
    
    def start_logger(self, time_sampling, logmask):
        opcode = "1"
        cmd = (opcode +" "+ str(time_sampling)+" "+str(logmask)).encode("UTF-8")
        self.send_cmd(cmd, False)

    def stop_logger(self):
        opcode = "2"
        cmd = (opcode).encode("UTF-8")
        self.send_cmd(cmd, False)

    def move_motor_speed(self, motor_id, speed):
        opcode = "3"
        cmd = (opcode +" "+ str(motor_id)+" "+str(speed)).encode("UTF-8")
        self.send_cmd(cmd)

    def move_motor_abs_pos(self, motor_id, pos):
        opcode = "4"
        cmd = (opcode +" "+ str(motor_id)+" "+str(pos)).encode("UTF-8")
        self.send_cmd(cmd)

    def stop_motor(self, motor_id):
        opcode = "5"
        cmd = (opcode +" "+ str(motor_id)).encode("UTF-8")
        self.send_cmd(cmd)

    def set_motor_pos_pid_param(self, motor_id, kp, ki, kd):
        opcode = "6"
        cmd = (opcode +" "+ str(motor_id)+" "+str(kp)+" "+str(ki)+" "+str(kd)).encode("UTF-8")
        self.send_cmd(cmd)  

    def get_motor_pos_pid_param(self, motor_id):
        opcode = "7"
        cmd = (opcode +" "+ str(motor_id)).encode("UTF-8")
        self.send_cmd(cmd)

    def set_motor_speed_pid_param(self, motor_id, kp, ki, kd):
        opcode = "8"
        cmd = (opcode +" "+ str(motor_id)+" "+str(kp)+" "+str(ki)+" "+str(kd)).encode("UTF-8")
        self.send_cmd(cmd)  

    def get_motor_speed_pid_param(self, motor_id):
        opcode = "9"
        cmd = (opcode +" "+ str(motor_id)).encode("UTF-8")
        self.send_cmd(cmd)
    
    def get_logged_item(self):
        opcode = "10"
        cmd = (opcode).encode("UTF-8")
        self.send_cmd(cmd, False)
    
    def clear_logged_item(self):
        opcode = "11"
        cmd = (opcode).encode("UTF-8")
        self.send_cmd(cmd, False)
    
    def move_motor_abs_pos_trapezoid(self, motor_id, target, vel, acc):
        opcode = "12"
        cmd = (opcode +" "+ str(motor_id)+" "+str(target)+" "+str(vel)+" "+str(acc)).encode("UTF-8")
        self.send_cmd(cmd)  

class FWLogger:
    def __init__(self, p: Device):
        self.p = p
        self.ser = self.p.ser
        self.logging = False
        self.logged_data = []
        self.mask = 0
        self.mask_names = {  # Map mask flags to column names
            1 << 0: 'MotorPosition',
            1 << 1: 'MotorSpeed',
            1 << 2: 'CommandedPosition',
            1 << 3: 'CommandedSpeed',
        }

    def __MainLogger(self, limit=1_000_000):
        try:
            count = 0
            while self.logging and count < limit:
                with self.p.lock:  # Ensure atomic command and read
                    self.p.get_logged_item()
                    data = self.ser.readline()
                self.logged_data.append(data)
                count += 1
                time.sleep(0.001)
        except Exception as e:
            print(f"Logger error: {e}")

    def run(self, time_sampling=10, mask=15, limit=1_000_000):
        try:
            self.logged_data.clear()
            self.mask = mask
            time.sleep(0.1)
            with self.p.lock:
                self.p.start_logger(time_sampling, mask)
            self.logging = True
            self.logger_thread = SThread(self.__MainLogger, True, limit)
        except Exception as e:
            print(f"Failed to start logger: {e}")

    def stop(self):
        self.logging = False
        with self.p.lock:
            self.p.stop_logger()
        while True:
            with self.p.lock:
                self.p.get_logged_item()
                data = self.ser.readline()
            if len(data) == 3:
                break
            self.logged_data.append(data)
        
        tag = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
        self.save_to_csv("LOG/"+tag+".csv")
    
    def get_n_col(self, mask: int):
        count = 0
        while mask > 0:
            count = count + mask%2
            mask = mask >> 1
        
        return count+1

    def print_logged_data(self):
        print("\nLogged data:")
        for line in self.logged_data:
            value = line.decode('utf-8', errors='replace').strip()
            try:
                integer_list = [int(x.strip()) for x in value.split(' ')]
                n_col = self.get_n_col(self.mask)
            
                idx = 1
                for i in range(0, integer_list[0]):
                    output = ""
                    for j in range(0, n_col):
                        output = output + str(integer_list[idx]) + " "
                        idx += 1
                    print(output)
            except:
                continue
    
    def save_to_csv(self, filename):
        try:
            # Generate column headers based on mask
            columns = ['Timestamp']
            for bit in range(4):  # Check bits 0-3 from the enum
                flag = 1 << bit
                if self.mask & flag:
                    columns.append(self.mask_names.get(flag, f'Unknown Bit {bit}'))
            
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(columns)
                
                n_col = self.get_n_col(self.mask)  # Total columns per entry
                
                for data_line in self.logged_data:
                    try:
                        decoded = data_line.decode('utf-8', errors='replace').strip()
                        integer_list = list(map(int, decoded.split()))
                        count = integer_list[0] if integer_list else 0
                        idx = 1
                        
                        for _ in range(count):
                            if idx + n_col > len(integer_list):
                                break  # Prevent index errors
                            
                            entry_data = integer_list[idx:idx + n_col]
                            timestamp = entry_data[0]
                            data_values = entry_data[1:]
                            
                            # Create row with timestamp followed by data values
                            writer.writerow([timestamp] + data_values)
                            idx += n_col
                    except Exception as e:
                        print(f"Skipping malformed line: {e}")
                        continue
        except Exception as e:
            print(f"Error saving to CSV: {e}")

p = Device()
p.connect("0")

logger = FWLogger(p)

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


def trapezoid_test(position, speed, acc):
    logger.run(5, 5)
    p.move_motor_abs_pos_trapezoid(0, position, speed, acc)
    time.sleep(3)
    logger.stop()
    p.stop_motor(0)
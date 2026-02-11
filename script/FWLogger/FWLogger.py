import threading
import time
import csv
import Config.Motor0 as config
import queue

# Logger Mask
N_ENUM = 5

LOG_MASK = [
    'Motor_Position(rotation)', #1
    'Motor_Speed(RPM)', #2
    'Commanded_Position(rotation)', #4
    'Commanded_Speed(RPM)', #8
    'Commanded_PWM', #16
]

SCALE_OFFSET_MOTOR = [
    [(config.ROTATION_PER_PULSE), 0],
    [(60.0*config.ROTATION_PER_PULSE), 0],
    [(config.ROTATION_PER_PULSE), 0],
    [(60.0*config.ROTATION_PER_PULSE), 0],
    [1, 0],
]

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
            self.thread = threading.Thread(target=self.func, args=(*args,))
            self.thread.daemon = self.daemon
            self.thread.start()

    def stop(self):
        self.thread.join()

class FWLogger:
    def __init__(self, p):
        self.p = p
        self.ser = self.p.ser
        self.logging = False
        self.logged_data = []
        self.mask = 0
        self.mask_names = LOG_MASK

    def __MainLogger(self, limit=1_000_000):
        try:
            count = 0
            while self.logging and count < limit:
                try:
                    data = self.p.log_queue.get_nowait()
                    self.logged_data.append(data)
                    count += 1

                    if count >= limit:
                        break

                except queue.Empty:
                    time.sleep(0.001)
                    continue
        except Exception as e:
            print(f"Logger error: {e}")

    def run(self, motor_id, mask, time_sampling, limit=1_000_000):
        try:
            self.logged_data = []
            self.p.stop_logger(motor_id)
            self.p.clear_log_queue()
            self.p.ser.reset_input_buffer()
            self.mask = mask

            with self.p.lock:
                self.p.start_logger(motor_id, time_sampling)
            self.logging = True
            self.logger_thread = SThread(self.__MainLogger, True, limit)
            print(f"Starting Logger")
        except Exception as e:
            print(f"Failed to start logger: {e}")

    def stop(self, motor_id):
        print("Stopping Logger")
        self.logging = False

        with self.p.lock:
            self.p.stop_logger(motor_id)
        self.p.clear_log_queue()

        if hasattr(self, 'logger_thread'):
            self.logger_thread.stop()

        tag = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
        self.save_to_csv("LOG/"+tag+".csv")
    
    def save_to_csv(self, filename):
        if not self.logged_data:
            print("Save failed: No data was collected.")
            return
        
        try:
            # Generate column headers based on mask
            selected_data = []
            columns = ["Timestamp(ms)"]

            for bit in range(N_ENUM):
                flag = 1 << bit
                if self.mask & flag:
                    selected_data.append(bit)
                    columns.append(self.mask_names[bit])
            
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(columns)
                
                for data_line in self.logged_data:
                    row = [data_line[0]] # Include Timestamp

                    for idx in selected_data:

                        scale = SCALE_OFFSET_MOTOR[idx][0]
                        offset = SCALE_OFFSET_MOTOR[idx][0]

                        processed_value = (data_line[idx+1] * scale) + offset
                        row.append(processed_value)
                    
                    try:
                        writer.writerow(row)
                    
                    except Exception as e:
                        print(f"Skipping malformed line: {e}")
                        continue
        except Exception as e:
            print(f"Error saving to CSV: {e}")

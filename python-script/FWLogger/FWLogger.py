import threading
import time
import csv

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


class FWLogger:
    def __init__(self, p):
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

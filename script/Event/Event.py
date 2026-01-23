import threading
import time

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

class Event:
    def __init__(self, p):
        self.p = p
        self.ser = self.p.ser
        self.logging = False
        self.logged_data = []
        self.mask = 0
        self.data = []
        self.event = []
        self.log = []

    def __MainLogger(self, limit=1_000_000):
        try:
            count = 0
            while self.logging:
                with self.p.lock:  # Ensure atomic command and read
                    self.p.get_logged_item()
                    data = self.ser.readline().decode('utf-8', errors='replace').strip().split(" ")
                    
                    if data[0] == "data":
                        self.data.append(data[1:])

                    elif data[0] == "event":
                        self.event.append(data[1:])
                    
                    elif data[0] == "log":
                        self.log.append(data[1:])

                count += 1
                time.sleep(0.001)
        except Exception as e:
            print(f"Logger error: {e}")

    def run(self, time_sampling=10, mask=15, limit=1_000_000):
        try:
            self.data.clear()
            self.event.clear()
            self.log.clear()
            time.sleep(0.1)
            self.logging = True
            self.logger_thread = SThread(self.__MainLogger, True, limit)
        except Exception as e:
            print(f"Failed to start logger: {e}")

    def stop(self):
        self.logging = False

    def print_logged_data(self):
        print("\nLogged data:")
        print("Data", self.data)
        print("Event", self.event)
        print("Log", self.log)
    

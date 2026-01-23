import serial
import yaml
from functools import partial
import serial.tools.list_ports
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

class Pico:
    def __init__(self, yaml_path="", com=None):
        self.ser = None
        self.newinput = None
        self.defaultCOM = None
        self.baud_rate = 115200
        self.lock = threading.RLock()  # Use RLock for reentrant locking
        self.event_queue = []
        self.error_queue = []
        self.is_listener_on = False
        self.connect(com)
        self._load_commands_from_yaml(yaml_path)  # Load command definitions
        self.run()
        # self.get_motor_pos(0)

    def _load_commands_from_yaml(self, yaml_path):
        with open(yaml_path, 'r') as f:
            commands = yaml.safe_load(f)
        
        for cmd in commands:
            try:
                name = cmd['name']
                op = cmd['op']
                args = cmd.get('args', [])
                ret = cmd.get('ret', [])
            except:
                continue
            
            # Create method with appropriate signature
            def command_method(self, *values, op_code=op, params=args, ret=ret):
                if len(values) != len(params):
                    return f"Expected {params} arguments, got {len(values)}"
                
                # Build command string
                cmd_str = op_code
                for val in values:
                    cmd_str += f" {val}"
                
                # Determine if we should print echo (default True except for logger commands)
                print_echo = True if len(ret)>0 else False
                status = self.send_cmd(cmd_str.encode('UTF-8'), print_echo)  # Return the response
                retry_count = 10

                if status == None:
                    return None
                
                elif status[0] == "data":
                    value = status[1:]
                    response_lines = []
                    response = {}

                    for i in range(0, len(value)):
                        for x in value:
                            try:
                                new_value = float(x.strip())
                                response_lines.append(new_value)
                            except:
                                continue
                        response[ret[i]] = response_lines[i]
                    return response
                else:
                    return None
            
            # Set method name and parameters
            command_method.__name__ = name
            command_method.__qualname__ = f"Device.{name}"
            command_method.__doc__ = f"Generated method for {name} command"
            
            # Create partial method with locked execution
            locked_method = partial(self._execute_with_lock, command_method)
            setattr(self, name, locked_method)

    def _execute_with_lock(self, func, *args, **kwargs):
        with self.lock:
            return func(self, *args, **kwargs)
    
    def connect(self, port):
        if port is None:
            acm_ports = [
                 port.device for port in serial.tools.list_ports.comports() 
                 if port.description.startswith("USB-serial logger")]
            
            if not acm_ports:
                raise OSError("No /dev/ttyACM* ports found. Connect a device.")

            # Auto-select the first port (modify logic if multiple devices)
            port = acm_ports[0]
            print(f"Selected port: {port}")
        
        else:
            port = "/dev/ttyACM" + port
        
        stat = True

        try:
            with self.lock:
                self.ser = serial.Serial(port, self.baud_rate, timeout=1)
                time.sleep(1)
                print(f"Connected to {port}")
                return True
        except:
            self.newinput = None
            stat = False
        finally:
            if self.newinput == "":
                return False
            if not stat:
                self.defaultCOM = self.newinput
                self.connect(self.newinput)
    
    def send_cmd(self, cmd, print_echo=True):
        try:
            with self.lock:  # Use the RLock to synchronize
                self.ser.write(cmd)
                time.sleep(0.01)

                if print_echo:
                    while self.ser.in_waiting:
                        data = self.ser.readline()

                    message = data.decode('utf-8', errors='replace').strip().split(" ")
                    return message
                
            time.sleep(0.1)
            return None
        except serial.SerialException as e:
            return f"Serial error: {e}"
        except Exception as e:
            return f"Unexpected error: {e}"
    
    def __EventListener(self, limit=1_000_000):
        try:
            count = 0
            print("Event Listener is Ready")

            while self.is_listener_on: # and count < limit:
                if self.ser.in_waiting:
                    with self.lock:  # Ensure atomic command and read
                        message = self.ser.readline().decode('utf-8', errors='replace').strip().split(" ")
                        
                        if message[0] == "data" or message[0] == "log":
                            continue
                        elif message[0] == "event":
                            self.event_queue.append(message[1:])
                            # TODO: Implement Event Handler
                        else:
                            if len(message) > 1:
                                self.error_queue.append(message)
                            # TODO: Implement Error Handler

                time.sleep(0.1)
                count += 1
        except Exception as e:
            print(f"Logger error: {e}")

    def run(self, limit=1_000_000):
        try:
            self.is_listener_on = True
            self.logger_thread = SThread(self.__EventListener, True, limit)
        except Exception as e:
            print(f"Failed to start listener: {e}")

    def print_queue(self):
        print("Event:", self.event_queue)
        print("Error:", self.error_queue)
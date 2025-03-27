import serial
import yaml
from functools import partial
import serial.tools.list_ports
import threading
import time

class Device:
    def __init__(self, yaml_path="", com="0"):
        self.ser = None
        self.newinput = None
        self.defaultCOM = "0"
        self.baud_rate = 115200
        self.lock = threading.RLock()  # Use RLock for reentrant locking
        self.connect(com)
        self._load_commands_from_yaml(yaml_path)  # Load command definitions

    def _load_commands_from_yaml(self, yaml_path):
        with open(yaml_path, 'r') as f:
            commands = yaml.safe_load(f)
        
        for cmd in commands:
            name = cmd['name']
            op = cmd['op']
            args = cmd.get('args', [])
            
            # Create method with appropriate signature
            def command_method(self, *values, op_code=op, params=args):
                if len(values) != len(params):
                    raise ValueError(f"Expected {len(params)} arguments, got {len(values)}")
                
                # Build command string
                cmd_str = op_code
                for val in values:
                    cmd_str += f" {val}"
                
                # Determine if we should print echo (default True except for logger commands)
                print_echo = not op_code in ["1", "2", "10", "11"]
                self.send_cmd(cmd_str.encode('UTF-8'), print_echo)
            
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

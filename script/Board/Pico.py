import serial
import yaml
from functools import partial
import serial.tools.list_ports
import threading
import time

class Pico:
    def __init__(self, yaml_path="", com=None):
        self.ser = None
        self.newinput = None
        self.defaultCOM = None
        self.baud_rate = 115200
        self.lock = threading.RLock()  # Use RLock for reentrant locking
        self.connect(com)
        self._load_commands_from_yaml(yaml_path)  # Load command definitions

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
                value = self.send_cmd(cmd_str.encode('UTF-8'), print_echo)  # Return the response
                if print_echo :
                    response = {}
                    for i in range(0, len(value)):
                        response[ret[i]] = value[i]
                    return response
            
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
        with self.lock:  # Use the RLock to synchronize
            try:
                self.ser.write(cmd)
                time.sleep(0.1)

                if print_echo:
                    response_lines = []
                    raw_lines = []
                    while self.ser.in_waiting:
                        data = self.ser.readline()
                        raw_lines.append(data)

                    for line in raw_lines:
                        value = line.decode('utf-8', errors='replace').strip()
                        for x in value.split(' '):
                            try:
                                new_value = float(x.strip())
                                response_lines.append(new_value)
                            except:
                                continue
                    return response_lines
                return None  # Return None if print_echo is False
            except serial.SerialException as e:
                return f"Serial error: {e}"
            except Exception as e:
                return f"Unexpected error: {e}"
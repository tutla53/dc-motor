import serial
import yaml
from functools import partial
import serial.tools.list_ports
import threading
import time
import struct

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
                op = int(cmd['op'])
                args = cmd.get('args', [])
                ret = cmd.get('ret', [])
            except:
                continue
            
            # Create method with appropriate signature
            def command_method(self, *values, op_code=op, params=args, ret=ret):
                if len(values) != len(params):
                    return f"Expected {params} arguments, got {len(values)}"
                
                fmt = '<B' 
                packed_vals = [op_code]

                for p_name, val in zip(params, values):
                    p_name = p_name.lower()
                    
                    if "id" in p_name: 
                        fmt += 'B' # u8 (1 byte)
                        packed_vals.append(int(val))
                    elif "time_sampling" in p_name:
                        fmt += 'Q' # u64 (8 bytes)
                        packed_vals.append(int(val))
                    elif "log_mask" in p_name or "logmask" in p_name:
                        fmt += 'I' # u32 (4 bytes)
                        packed_vals.append(int(val))
                    elif isinstance(val, float) or any(x in p_name for x in ["kp", "ki", "kd", "target", "velocity", "acceleration"]):
                        fmt += 'f' # f32 (4 bytes)
                        packed_vals.append(float(val))
                    else:
                        fmt += 'i' # i32 (4 bytes) - used for speed, pos, pwm
                        packed_vals.append(int(val))
                
                binary_payload = struct.pack(fmt, *packed_vals)

                expect_reply = True if len(ret) > 0 else False
                status = self.send_cmd(binary_payload, expect_reply)
                                
                return status
            
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
                 if port.description.startswith("USB-serial example")]
            
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
    
    def send_cmd(self, cmd_bytes, print_echo=True):
        try:
            with self.lock:  # Use the RLock to synchronize
                self.ser.write(cmd_bytes)
                self.ser.flush()

                if print_echo:
                    byte = self.ser.read(1)
                    if not byte:
                        print("Timeout: No data received")
                        return None
                        
                    if byte == b'\x07':
                        payload = self.ser.read(24) # 6 * f32 (4 bytes)
                        if len(payload) < 24: return None
                        unpacked = struct.unpack('<ffffff', payload)
                        return {
                            "kp": unpacked[0], "ki": unpacked[1], "kd": unpacked[2],
                            "kp_speed": unpacked[3], "ki_speed": unpacked[4], "kd_speed": unpacked[5]
                        }

                    # --- Case 0x09: Get Motor Speed PID (3 floats) ---
                    elif byte == b'\x09':
                        payload = self.ser.read(12) # 3 * f32 (4 bytes)
                        if len(payload) < 12: return None
                        unpacked = struct.unpack('<fff', payload)
                        return {"kp": unpacked[0], "ki": unpacked[1], "kd": unpacked[2]}

                    # --- Case 0x13: Get Motor Position (1 i32 or f32) ---
                    elif byte == b'\x13':
                        payload = self.ser.read(4) 
                        if len(payload) < 4: return None
                        # Use 'i' if your Rust motor_pos is i32, 'f' if it is f32
                        unpacked = struct.unpack('<i', payload) 
                        return {"pos_count": unpacked[0]}

                    # --- Case 0x14: Get Motor Speed (1 f32) ---
                    elif byte == b'\x14':
                        payload = self.ser.read(4)
                        if len(payload) < 4: return None
                        unpacked = struct.unpack('<f', payload)
                        return {"speed_cps": unpacked[0]}
                    
                    # --- Fallback: ASCII Response ---
                    else:
                        # If it wasn't a binary header, read the rest of the line
                        _ = byte + self.ser.readline()
                        return None
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
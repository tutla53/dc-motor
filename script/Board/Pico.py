import serial
import yaml
from functools import partial
import serial.tools.list_ports
import threading
import queue
import time
import struct
from Tool.visualize import *

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
    __TYPE_MAP = {
        'u8': ('B', 1),
        'u16': ('H', 2),
        'u32': ('I', 4),
        'i32': ('i', 4),
        'f32': ('f', 4),
        'u64': ('Q', 8),
    }

    def __init__(self, yaml_path="", com=None):
        # Public
        self.ser = None
        self.lock = threading.RLock()
        self.event_queue = []
        self.error_queue = []
        self.log_queue = queue.Queue(maxsize=0)

        # Private
        self.__baud_rate = 115200
        
        self.__is_listener_on = False
        self.__headers = {}
        self.__enums = {}
        self.__cmd_meta = {}         
        self.__shared_responses = {}
        self.__response = False
        self.yaml_version = ""
        
        printy("Running Raspberry Pico RP2040 script, please wait...")
        status = self.__connect(com)
        if status:
            self.__load_commands_from_yaml(yaml_path)
            self.__run()
            print_log("INFO", "Connected to: ", end="")
            printdg(self.ser.port)
            print_log("INFO",  "YAML Version: ", end="")
            printdb(self.yaml_version)
        else:
            self.__load_commands_from_yaml(yaml_path)
            print_log("INFO", "Mode: ", end="")
            printy("Simulation Mode")
            print_log("INFO",  "YAML Version: ", end="")
            printdb(self.yaml_version)

    def _generate_method(self, cmd):
        name = cmd['command']
        op = int(cmd['op'])
        args_dict = cmd.get('args', {})
        ret_dict = cmd.get('ret', {})

        # 1. Build Packing Format (Outgoing)
        send_fmt = '<B' # Header: OpCode
        for t in args_dict.values():
            send_fmt += self.__TYPE_MAP[t][0]

        # 2. Build Unpacking Format (Incoming Response)
        ret_fmt = '<'
        ret_size = 0
        for t in ret_dict.values():
            char, size = self.__TYPE_MAP[t]
            ret_fmt += char
            ret_size += size
        
        # Save metadata for the EventListener to use
        self.__cmd_meta[op] = {'fmt': ret_fmt, 'size': ret_size, 'keys': list(ret_dict.keys())}

        def method(self, *values, _op=op, _send_fmt=send_fmt, _has_ret=(ret_size > 0)):
            header_byte = self.__headers.get('COMMAND')
            
            if header_byte is None:
                raise ValueError("COMMAND header not found in YAML config!")
            
            op_and_args = struct.pack(_send_fmt, _op, *values)
            payload = header_byte + op_and_args  
            return self.send_cmd(payload, _op, _has_ret)

        setattr(self, name, partial(method, self))

    def __load_commands_from_yaml(self, yaml_path):
        with open(yaml_path, 'r') as f:
            Config = yaml.safe_load(f)
        
        for item in Config:
            if 'Headers' in item:
                self.__headers = {k: v.to_bytes(1, 'little') for k, v in item['Headers'].items()}
            
            elif 'Enums' in item:
                # The YAML parser creates a list for the dashes. 
                # We convert that list into a single flat dictionary.
                raw_enums = item['Enums']
                self.__enums = {}
                
                if isinstance(raw_enums, list):
                    for entry in raw_enums:
                        if isinstance(entry, dict):
                            # This handles the nested 'EventCodes' and 'ErrorCodes'
                            self.__enums.update(entry)
                else:
                    self.__enums = raw_enums

            elif 'command' in item:
                self._generate_method(item)
            elif "Version" in item:
                self.yaml_version = item["Version"]

    def _execute_with_lock(self, func, *args, **kwargs):
        with self.lock:
            return func(self, *args, **kwargs)
        
    def _process_log(self, payload):      
        try:
            # '<' = Little Endian
            # 'B' = u8 (Seq)
            # 'I' = u32 (DT)
            # '5i' = 5 x i32 (Values)
            unpacked = struct.unpack('<BI5i', payload)
            
            log_entry = [unpacked[1], unpacked[2], unpacked[3], unpacked[4], unpacked[5], unpacked[6]]
            self.log_queue.put(log_entry)
            
        except Exception as e:
            print(f"Log Unpack Error: {e}")
    
    def __connect(self, port):
        if port is None:
            acm_ports = [
                port.device for port in serial.tools.list_ports.comports() 
                if port.serial_number.startswith("12345678")]
            
            if not acm_ports:
                printr("No USB ports found. Connect a device.")
                return False

            port = acm_ports[0]
        
        else:
            port = "/dev/ttyACM" + port

        try:
            with self.lock:
                self.ser = serial.Serial(port, self.__baud_rate, timeout=0.1, write_timeout=0.5)
                time.sleep(1)
                self.ser.reset_input_buffer()
                return True
        except Exception as e:
            printr("Serial Error:", e)
            return False
    
    def send_cmd(self, payload, op, expect_ret, timeout=0.5):
        self.__response = False

        if expect_ret:
            self.__shared_responses[op] = None
            
        with self.lock:
            self.ser.write(payload)
            self.ser.flush()

        start = time.time()
        while self.__response == False:
            if time.time() - start > timeout:
                return False
            time.sleep(0.001)

        if not expect_ret: return True

        start = time.time()
        while self.__shared_responses.get(op) is None:
            if time.time() - start > timeout:
                print(f"Timeout: OP {op} took too long.")
                return None
            time.sleep(0.001)

        # Retrieve and clean up
        raw_payload = self.__shared_responses.pop(op)
        meta = self.__cmd_meta[op]
        unpacked = struct.unpack(meta['fmt'], raw_payload)
        return dict(zip(meta['keys'], unpacked))
        
    def __EventListener(self, limit=1_000_000):
        local_buffer = bytearray()
        
        while self.__is_listener_on:
            if self.ser.in_waiting > 0:
                with self.lock:
                    if self.ser.in_waiting > 0:
                        local_buffer.extend(self.ser.read(self.ser.in_waiting))

            while len(local_buffer) > 0:
                header = local_buffer[0:1]
                
                if header == self.__headers.get('LOGGER'):
                    if len(local_buffer) >= 26:
                        payload = local_buffer[1:26]
                        self._process_log(payload)
                        del local_buffer[:26]
                    else:
                        break

                elif header == self.__headers.get('EVENT'):
                    if len(local_buffer) >= 3:
                        ev_code, m_id = struct.unpack('<BB', local_buffer[1:3])
                        name = self.__enums['EventCodes'].get(ev_code, "UNKNOWN")
                        self.event_queue.append({name: m_id})
                        printg(f"[EVENT] Motor {m_id}: {name}")
                        del local_buffer[:3]
                    else:
                        break

                elif header == self.__headers.get('COMMAND'):
                    if len(local_buffer) >= 2:
                        error_code = int(local_buffer[1])
                        decoded_error = self.__enums['ErrorCodes'].get(error_code, "UNKLNONW")

                        if decoded_error == "NoError":
                            if len(local_buffer) >=3 :
                                op_code = int(local_buffer[2])
                                meta = self.__cmd_meta.get(op_code)
                                payload_size = meta['size'] if meta else 0
                                total_packet_size = 3 + payload_size
                                
                                if len(local_buffer) >= total_packet_size:
                                    with self.lock:
                                        self.__response = True
                                        if payload_size > 0:
                                            self.__shared_responses[op_code] = local_buffer[3:total_packet_size]
                                    del local_buffer[:total_packet_size]
                                else:
                                    break
                            else:
                                break
                        else:
                            if decoded_error == "InvalidHeaderCode":
                                printr(f"[ERROR]: {decoded_error}")
                                del local_buffer[:2]
                            elif len(local_buffer) >= 3 :
                                op_code = int(local_buffer[2])
                                
                                with self.lock:
                                    self.__response = True
                                    self.__shared_responses[op_code] = None
                                printr(f"[ERROR]: {decoded_error} on OP {op_code}")
                                del local_buffer[:3]
                            else:
                                break
                    else:
                        break
                else:
                    del local_buffer[0:1]
                    printr(f"Alignment may be lost!")
            time.sleep(0.001)

    def __run(self, limit=1_000_000):
        try:
            self.__is_listener_on = True
            self.logger_thread = SThread(self.__EventListener, True, limit)
        except Exception as e:
            print(f"Failed to start listener: {e}")
    
    def clear_log_queue(self):
        while not self.log_queue.empty():
            try:
                self.log_queue.get_nowait()
            except queue.Empty:
                break
    
    def print_queue(self):
        print("Event:", self.event_queue)
        print("Error:", self.error_queue)
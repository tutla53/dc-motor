import serial
import yaml
from functools import partial
import serial.tools.list_ports
import threading
import queue
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
    TYPE_MAP = {
        'u8': ('B', 1),
        'u16': ('H', 2),
        'u32': ('I', 4),
        'i32': ('i', 4),
        'f32': ('f', 4),
        'u64': ('Q', 8),
    }

    def __init__(self, yaml_path="", com=None):
        self.ser = None
        self.newinput = None
        self.defaultCOM = None
        self.baud_rate = 115200
        self.lock = threading.RLock()
        self.event_queue = []
        self.error_queue = []
        self.log_queue = queue.Queue(maxsize=0)
        self.is_listener_on = False
        self.headers = {}
        self.enums = {}
        self.cmd_meta = {}         
        self.shared_responses = {}
        self.connect(com)
        self._load_commands_from_yaml(yaml_path)
        self.run()

    def _generate_method(self, cmd):
        name = cmd['command']
        op = int(cmd['op'])
        args_dict = cmd.get('args', {})
        ret_dict = cmd.get('ret', {})

        # 1. Build Packing Format (Outgoing)
        send_fmt = '<B' # Header: OpCode
        for t in args_dict.values():
            send_fmt += self.TYPE_MAP[t][0]

        # 2. Build Unpacking Format (Incoming Response)
        ret_fmt = '<'
        ret_size = 0
        for t in ret_dict.values():
            char, size = self.TYPE_MAP[t]
            ret_fmt += char
            ret_size += size
        
        # Save metadata for the EventListener to use
        self.cmd_meta[op] = {'fmt': ret_fmt, 'size': ret_size, 'keys': list(ret_dict.keys())}

        def method(self, *values, _op=op, _send_fmt=send_fmt, _has_ret=(ret_size > 0)):
            header_byte = self.headers.get('COMMAND')
            
            if header_byte is None:
                raise ValueError("COMMAND header not found in YAML config!")
            
            op_and_args = struct.pack(_send_fmt, _op, *values)
            payload = header_byte + op_and_args  
            return self.send_cmd(payload, _op, _has_ret)

        setattr(self, name, partial(method, self))

    def _load_commands_from_yaml(self, yaml_path):
        with open(yaml_path, 'r') as f:
            Config = yaml.safe_load(f)
        
        for item in Config:
            if 'Headers' in item:
                self.headers = {k: v.to_bytes(1, 'little') for k, v in item['Headers'].items()}
            
            elif 'Enums' in item:
                # The YAML parser creates a list for the dashes. 
                # We convert that list into a single flat dictionary.
                raw_enums = item['Enums']
                self.enums = {}
                
                if isinstance(raw_enums, list):
                    for entry in raw_enums:
                        if isinstance(entry, dict):
                            # This handles the nested 'EventCodes' and 'ErrorCodes'
                            self.enums.update(entry)
                else:
                    self.enums = raw_enums

            elif 'command' in item:
                self._generate_method(item)

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
                self.ser = serial.Serial(port, self.baud_rate, timeout=0.1, write_timeout=0.5)
                time.sleep(1)
                self.ser.reset_input_buffer()
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
    
    def send_cmd(self, payload, op, expect_ret, timeout=2):
        if expect_ret:
            self.shared_responses[op] = None
            
        with self.lock:
            self.ser.write(payload)
            self.ser.flush()

        if not expect_ret: return True

        start = time.time()
        while self.shared_responses.get(op) is None:
            if time.time() - start > timeout:
                print(f"Timeout: OP {op} took too long.")
                return None
            time.sleep(0.001)

        # Retrieve and clean up
        raw_payload = self.shared_responses.pop(op)
        meta = self.cmd_meta[op]
        unpacked = struct.unpack(meta['fmt'], raw_payload)
        return dict(zip(meta['keys'], unpacked))
        
    def __EventListener(self, limit=1_000_000):
        while self.is_listener_on:
            if self.ser.in_waiting > 0:
                with self.lock:
                    if self.ser.in_waiting > 0:
                        header = self.ser.read(1)
                    
                        if header == self.headers.get('LOGGER'):
                            data = self.ser.read(25)
                            self._process_log(data)

                        elif header == self.headers.get('EVENT'):
                            ev_code, m_id = struct.unpack('<BB', self.ser.read(2))
                            name = self.enums['EventCodes'].get(ev_code, "UNKNOWN")
                            self.event_queue.append({name: m_id})
                            print(f"[EVENT] Motor {m_id}: {name}")

                        elif header == self.headers.get('COMMAND'):
                            op_byte = self.ser.read(1)
                            op = int.from_bytes(op_byte, 'little')
                            
                            if op in self.cmd_meta:
                                size = self.cmd_meta[op]['size']
                                self.shared_responses[op] = self.ser.read(size)
                        else:
                            print(f"Unexpected Byte: {header.hex()} - Alignment may be lost!")
            
            time.sleep(0.0001)

    def run(self, limit=1_000_000):
        try:
            self.is_listener_on = True
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
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt # type: ignore
import threading
import time
import queue  # Import the queue module

class SerialLogger:
    def __init__(self):
        self.logging = False
        self.log_data = []
        self.lock = threading.Lock()

    def start_logging(self):
        with self.lock:
            self.log_data.clear()
            self.logging = True
            print("Type 'print_logged_data' to see latest logged data")

    def stop_logging(self):
        with self.lock:
            self.logging = False
    
    def get_log_data(self):
        return self.log_data
    
    def print_logged_data(self):
        print("\nLogged data:")
        for line in self.log_data:
            value = line.decode('utf-8', errors='replace').strip()
            integer_list = [int(x.strip()) for x in value.split(' ')]
            print("{},{},{}".format(integer_list[0], integer_list[1], integer_list[2]))
            
    def log_line(self, line):
        with self.lock:
            if self.logging:
                self.log_data.append(line)
                return True
            return False

def serial_reader(ser, logger):
    buffer = bytearray()
    try:
        while True:
            data = ser.read(ser.in_waiting or 1)
            if data:
                buffer += data
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    if logger.log_line(line):
                        continue
                    try:
                        print(line.decode('utf-8', errors='replace').strip())
                    except UnicodeDecodeError:
                        print(f"Binary data: {line.hex()}")
            time.sleep(0.001)

    except serial.SerialException as e:
        print(f"Serial reader error: {e}")
    except Exception as e:
        print(f"Serial reader unexpected error: {e}")

def print_menu():
    print("Available command:")
    print("- log start <time_sampling_ms>")
    print("- log stop")
    print("- motor_test <time_sampling_ms> <speed>")
    print("- print_logged_data")
    print("- exit")

def serial_writer(ser, logger, plot_queue):  # Accept plot_queue as an argument
    try:
        while True:
            user_input = input(">> ")

            if user_input.startswith("log start"):
                try:
                    time_sampling = int(user_input.split(" ")[2])
                    ser.write(user_input.encode("utf-8"))
                    logger.start_logging()
                    print(f"Logging started with time sampling: {time_sampling} ms")
                    print("Type 'log stop' to stop the logger")
                except (ValueError, IndexError):
                    print("Invalid log start command. Use: log start <time_sampling_ms>")

            elif user_input == "log stop":
                ser.write(user_input.encode("utf-8"))
                logger.stop_logging()

            elif user_input.startswith("motor_test"):
                try:
                    time_sampling = int(user_input.split(" ")[1])
                    speed = int(user_input.split(" ")[2])
                    plot_data_to_show = motor_test(ser, logger, time_sampling, speed)
                    if plot_data_to_show:
                        plot_queue.put(plot_data_to_show)  # Put data into the queue
                except (ValueError, IndexError):
                    print("Invalid motor_test command. Use: motor_test <time_sampling_ms>")

            elif user_input == "help":
                print_menu()
            
            elif user_input == "print_logged_data":
                logger.print_logged_data()
            
            elif user_input == "exit":
                break

            else:
                print("Unknown command.")
                
    except serial.SerialException as e:
        print(f"Serial writer error: {e}")
    except Exception as e:
        print(f"Serial writer unexpected error: {e}")
    finally:
        print("Writer thread exiting.")

def motor_test(ser, logger, time_sampling, speed_rpm):
    log_start_cmd = "log start " + str(time_sampling)
    speed = int ((speed_rpm * 48.4)/60)
    
    ser.write(log_start_cmd.encode("utf-8"))
    logger.start_logging()
    print(f"Logging started with time sampling of: {time_sampling} ms")
    time.sleep(0.5)
    
    ser.write(("motor start " + str(speed)).encode("utf-8"))
    time.sleep(2)
    
    logger.stop_logging()
    ser.write("log stop".encode("utf-8"))
    time.sleep(0.1)
    
    ser.write("motor stop".encode("utf-8"))

    result = logger.get_log_data()
    time_ms = []
    motor_speed = []
    commanded = []
    
    for line in result:
        try:
            value = line.decode('utf-8', errors='replace').strip()
            integer_list = [int(x.strip()) for x in value.split(' ')]
            time_ms.append(integer_list[0])
            motor_speed.append((integer_list[1]*60/48.4))
            commanded.append((integer_list[2]*60/48.4))
        except ValueError:
            print(f"Warning: Could not parse line for plotting: {line}")
        except UnicodeDecodeError:
            print(f"Warning: Could not decode line for plotting: {line.hex()}")

    if time_ms and motor_speed and commanded:
        return (time_ms, motor_speed, commanded)
    else:
        print("No valid data to plot from motor test.")
        return None

def plot_graph(time_ms, motor_speed, commanded):
    plt.plot(time_ms, motor_speed, marker=".", label = "Encoder")
    plt.plot(time_ms, commanded, label = "Commanded")
    plt.xlabel("Time (ms)")
    plt.ylabel("Motor Speed (RPM)")
    plt.title("Motor Test Plot")
    plt.legend()
    plt.grid()
    plt.show()
    
def serial_monitor(port, baud_rate):
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        logger = SerialLogger()

        print(f"Connected to {port} at {baud_rate} baud.")
        print("Type 'exit' to quit.\nType 'help' to show available command")
        print_menu()

        plot_queue = queue.Queue()  # Create a plot queue

        reader_thread = threading.Thread(target=serial_reader, args=(ser, logger))
        writer_thread = threading.Thread(target=serial_writer, args=(ser, logger, plot_queue))  # Pass the queue

        reader_thread.daemon = True
        writer_thread.daemon = True

        reader_thread.start()
        writer_thread.start()

        # Main thread loop to handle plotting
        while True:
            if not writer_thread.is_alive():
                break
            try:
                data = plot_queue.get_nowait()
                plot_graph(*data)  # Plot on the main thread
            except queue.Empty:
                pass
            time.sleep(0.1)

    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.write("log stop".encode())
            ser.close()
            print("Serial port closed.")

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return None
    else:
        print("Available serial ports:")
        for i, (port, desc, hwid) in enumerate(sorted(ports)):
            print(f"{i+1}: {port}")
        return ports

if __name__ == "__main__":
    available_ports = list_serial_ports()
    if available_ports:
        selected_port = available_ports[0].device
        baud_rate = 115200
        serial_monitor(selected_port, baud_rate)
import serial
import threading
import time
import re

import time
import matplotlib.pyplot as plt

class DataLogger:
    def __init__(self):
        self.start_time = time.time()
        self.speed_history = []  # [(time, signed_speed)]
        self.func_history = {}   # {"F0": [(time, state)], ...}
        self.sensor_events = []  # [(time, sensor_id, state)]
        self.plot_func_labels = {}  # Format: {"F0": "Headlight", "F3": "Horn"}
        self.last_speed = None
        self.last_funcs = {}
        self.plot_acc_labels = {}  # Format: {"0001": "Turnout 1"}
        self.acc_history = {}
        self.last_accs = {}

    # Add new methods to DataLogger
    def set_acc_label(self, address, label):
        """Registers an accessory address to be plotted with a specific label."""
        self.plot_acc_labels[address] = label

    def log_accessory(self, address, state):
        """Logs accessory state changes (0 = clear/False, 1 = set/True)."""
        t = time.time() - self.start_time
        if address not in self.acc_history:
            self.acc_history[address] = [(0.0, state)]
            self.last_accs[address] = state
        
        if self.last_accs[address] != state:
            self.acc_history[address].append((t, state))
            self.last_accs[address] = state

    # Update DataLogger.reset_log
    def reset_log(self):
        # ... existing reset logic ...
        self.acc_history = {}
        
        # Re-seed accessory initial states at t=0.0
        for a_addr, a_state in self.last_accs.items():
            self.acc_history[a_addr] = [(0.0, a_state)]
        
    def set_func_label(self, func_name, label):
        """Registers a function to be plotted with a specific label."""
        self.plot_func_labels[func_name] = label

    def reset_log(self):
        """Clears all historical data and resets time zero to right now."""
        self.start_time = time.time()
        self.sensor_events = []
        self.speed_history = []
        self.func_history = {}
        
        # Re-seed the initial state at the new t=0.0
        if self.last_speed is not None:
            self.speed_history.append((0.0, self.last_speed))
            
        for f_name, f_state in self.last_funcs.items():
            self.func_history[f_name] = [(0.0, f_state)]
    def log_speed(self, direction, speed):
        t = time.time() - self.start_time
        # Convert to signed speed (Forward = positive, Reverse = negative)
        signed_speed = speed if direction == 'F' else -speed
        
        if self.last_speed != signed_speed:
            self.speed_history.append((t, signed_speed))
            self.last_speed = signed_speed
                
    def log_functions(self, functions_dict):
        t = time.time() - self.start_time
        for f_name, f_state in functions_dict.items():
            # Initialize history for this function if we haven't seen it yet
            if f_name not in self.func_history:
                self.func_history[f_name] = [(0.0, f_state)]
                self.last_funcs[f_name] = f_state
            
            # Log only if the state changed
            if self.last_funcs[f_name] != f_state:
                self.func_history[f_name].append((t, f_state))
                self.last_funcs[f_name] = f_state

    def log_sensor(self, sensor_id, state):
        t = time.time() - self.start_time
        self.sensor_events.append((t, sensor_id, state))
        
    def end_test(self):
        self.end_time = time.time() - self.start_time            

def generate_time_graph(logger, filename="test_result.png"):
    # 1. Gather Functions
    if logger.plot_func_labels:
        funcs_to_plot = {f_name: logger.func_history.get(f_name, [(0.0, False)]) 
                         for f_name in logger.plot_func_labels.keys()}
        func_labels_map = logger.plot_func_labels
    else:
        funcs_to_plot = {name: history for name, history in logger.func_history.items() if len(history) > 1}
        func_labels_map = {name: name for name in funcs_to_plot.keys()}

    # 2. Gather Accessories
    accs_to_plot = {}
    if logger.plot_acc_labels:
        accs_to_plot = {addr: logger.acc_history.get(addr, [(0.0, False)]) 
                        for addr in logger.plot_acc_labels.keys()}
        acc_labels_map = logger.plot_acc_labels

    # --- DYNAMIC WIDTH CALCULATION ---
    # Scale width based on total time (e.g., 1 inch per 5 seconds)
    # Ensure it never gets smaller than the default 10 inches
    scale_factor = 5.0 
    dynamic_width = max(10, logger.end_time / scale_factor)

    # Determine required subplots
    num_plots = 1 + (1 if funcs_to_plot else 0) + (1 if accs_to_plot else 0)
    
    # Use dynamic_width instead of the hardcoded 10
    fig, axes = plt.subplots(num_plots, 1, sharex=True, figsize=(dynamic_width, 3 + 2.5 * (num_plots - 1)))
    
    if num_plots == 1: axes = [axes]
    
    current_ax_idx = 0
    
    # ... (The rest of the plotting logic remains exactly the same) ...
    
    # --- Plot Speed (Always axes[0]) ---
    ax_speed = axes[current_ax_idx]
    if logger.speed_history:
        times, speeds = zip(*logger.speed_history)
        times, speeds = list(times) + [logger.end_time], list(speeds) + [speeds[-1]]
        ax_speed.step(times, speeds, where='post', color='#1f77b4', linewidth=2)
        

def _plot_logic_analyzer(ax, data_dict, labels_map, end_time, ylabel, sort_key=lambda x: int(x[0][1:])):
    """Helper to draw the stacked digital states for Functions or Accessories."""
    y_ticks, y_labels = [], []
    sorted_items = sorted(data_dict.items(), key=sort_key) if 'F' in list(data_dict.keys())[0] else sorted(data_dict.items())
    
    for i, (key, history) in enumerate(sorted_items):
        offset = i * 1.5 
        times, states = zip(*history)
        times, states = list(times) + [end_time], list(states) + [states[-1]]
        
        states_offset = [offset + (0.8 if s else 0.0) for s in states]
        ax.step(times, states_offset, where='post', linewidth=1.5)
        ax.axhline(offset, color='gray', linestyle=':', alpha=0.3)
        
        y_ticks.append(offset + 0.4)
        display_label = f"{key}: {labels_map[key]}" if key != labels_map[key] else key
        y_labels.append(display_label)

    ax.set_yticks(y_ticks)
    ax.set_yticklabels(y_labels)
    ax.set_ylabel(ylabel)
    ax.grid(True, axis='x', alpha=0.3)
    ax.set_ylim(-0.5, (len(data_dict) - 1) * 1.5 + 1.2)

class MotormanHIL:
    """Handles bidirectional serial communication with the test harness."""
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self._running = False
        self._thread = None
        self.lock = threading.Lock()
        self.loco_states = {}
        self.logger = DataLogger()

        # Map sensor names to their Active/Inactive characters
        self.sensor_map = {
            'R': ('R', 'r'),
            'L': ('L', 'l'),
            'A': ('A', 'a'),
            'B': ('B', 'b'),
            'P': ('P', 'p'),

        }

    def start(self):
        self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
        if self.serial_conn:
            self.serial_conn.close()

    # --- INPUT: Sending Commands ---

    def send_sensor_command(self, sensor_id, is_active):
        """Sends the appropriate character followed by a carriage return."""
        if sensor_id not in self.sensor_map:
            raise ValueError(f"Unknown sensor: {sensor_id}")
        
        char = self.sensor_map[sensor_id][0] if is_active else self.sensor_map[sensor_id][1]
        self._send_raw(char)
        self.logger.log_sensor(sensor_id, is_active) # <--- NEW LOGGING HOOK

    def send_power_command(self, is_active):
#        self._send_raw('p')
        self.send_sensor_command('P', is_active)

    def reset_sensors(self):
        """Sends the 'x' command to reset all sensors."""
        self._send_raw('x')

    def _send_raw(self, char):
        if self.serial_conn:
            self.serial_conn.write(f"{char}\n".encode('ascii'))
            self.serial_conn.flush()

    # --- OUTPUT: Reading and Parsing (From previous step) ---

    def _read_loop(self):
        while self._running:
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('ascii', errors='ignore').strip()
                # Update condition to accept accessory packets
                if line.startswith("M:") or line.startswith("A:"):
                    self._parse_packet(line)

    def _parse_packet(self, line):
        parts = line.split()
        if len(parts) < 2: return
        
        if parts[0].startswith("M:"):
            # ... (Existing locomotive parsing logic) ...
            address = parts[0].split(":")[1]
            direction = parts[1][0]
            speed = int(parts[1][1:])
            functions = {}
            for part in parts[2:]:
                if part.startswith("F00="): self._parse_func_block(part, 0, functions)
                elif part.startswith("F10="): self._parse_func_block(part, 10, functions)
                elif part.startswith("F20="): self._parse_func_block(part, 20, functions)
            
            with self.lock:
                self.loco_states[address] = {"direction": direction, "speed": speed, "functions": functions}
            self.logger.log_speed(direction, speed)
            self.logger.log_functions(functions)

        elif parts[0].startswith("A:"):
            # Example: A:0001 0 1
            address = parts[0].split(":")[1]
            state_val = parts[1]
            is_set = (state_val == '1')  # 1 = Set, 0 = Clear
           
            with self.lock:
                if not hasattr(self, 'acc_states'):
                    self.acc_states = {}
                self.acc_states[address] = is_set
            
            self.logger.log_accessory(address, is_set)


    def _parse_func_block(self, block_str, base_num, func_dict):
        match = re.search(r'\[(.*?)\]', block_str)
        if match:
            chars = match.group(1).replace(':', '')
            for i, char in enumerate(chars):
                func_dict[f"F{base_num + i}"] = (char != '-')

    def wait_for_accessory(self, address, expected_state, timeout):
        """Blocks until the accessory address reaches the expected state (True/False)."""
        start = time.time()
        while time.time() - start < timeout:
            with self.lock:
                state = getattr(self, 'acc_states', {}).get(address)
            if state == expected_state:
                return True
            time.sleep(0.05)
        return False

    def wait_for_motion(self, address, expected_dir, expected_speed, timeout):
        start = time.time()
        while time.time() - start < timeout:
            with self.lock:
                state = self.loco_states.get(address)
            if state and state["direction"] == expected_dir and state["speed"] == expected_speed:
                return True
            time.sleep(0.05)
        return False

    def wait_for_function(self, address, func_name, expected_state, timeout):
        start = time.time()
        while time.time() - start < timeout:
            with self.lock:
                state = self.loco_states.get(address)
            if state and state["functions"].get(func_name) == expected_state:
                return True
            time.sleep(0.05)
        return False

    def wait_for_motion_min(self, address, expected_dir, expected_speed, min_time, timeout):
        """Blocks until speed is reached, but fails if reached before min_time."""
        start = time.time()
        while time.time() - start < timeout:
            with self.lock:
                state = self.loco_states.get(address)
            
            if state and state["direction"] == expected_dir and state["speed"] == expected_speed:
                elapsed = time.time() - start
                if elapsed < min_time:
                    # Reached the speed too fast!
                    return False, f"Reached {expected_dir}{expected_speed} too quickly: {elapsed:.2f}s (minimum was {min_time}s)"
                else:
                    # Reached within the valid window
                    return True, ""
                    
            time.sleep(0.05)
            
        return False, f"Timeout waiting for {expected_dir}{expected_speed} after {timeout}s"

class ScriptRunner:
    """Reads a text file and executes the commands against the HIL interface."""
    def __init__(self, hil_interface):
        self.hil = hil_interface

    def run_script(self, filepath):
        print(f"--- Running Test Script: {filepath} ---")
        with open(filepath, 'r') as file:
            for line_num, line in enumerate(file, 1):
                line = line.strip()
                # Ignore empty lines and comments
                if not line or line.startswith('#'):
                    continue
                
                try:
                    self._execute_command(line)
                except Exception as e:
                    print(f"FAIL at line {line_num} ['{line}']: {e}")
                    return False
        
        print("--- Test Script Completed Successfully ---")
        return True

    def _execute_command(self, line):
            parts = line.split()
            cmd = parts[0].upper()

            if cmd == "RESET_SENSORS":
                self.hil.reset_sensors()

            # (Inside ScriptRunner._execute_command...)
            elif cmd == "PLOT_ACC":
                # PLOT_ACC 0001 Left Turnout
                addr = parts[1]
                label = " ".join(parts[2:]) 
                self.hil.logger.set_acc_label(addr, label)

            elif cmd == "EXPECT_ACC":
                # EXPECT_ACC 0001 1 2.0
                addr = parts[1]
                expected_state = (parts[2] == '1' or parts[2].upper() == 'SET')
                timeout = float(parts[3]) if len(parts) > 3 else 2.0
                
                success = self.hil.wait_for_accessory(addr, expected_state, timeout)
                if not success:
                    raise TimeoutError(f"Accessory {addr} did not become {parts[2]} within {timeout}s")


            elif cmd == "PLOT_FUNC":
                # PLOT_FUNC F0 Headlight
                # PLOT_FUNC F4 Dynamic Brakes
                func_name = parts[1].upper()
                
                # Join the remaining parts to allow multi-word descriptions
                label = " ".join(parts[2:]) 
                self.hil.logger.set_func_label(func_name, label)                
            elif cmd == "SET_SENSOR":
                # SET_SENSOR L ON
                sensor = parts[1].upper()
                state = parts[2].upper() == "ON"
                self.hil.send_sensor_command(sensor, state)

            elif cmd == "SET_POWER":
                # SET_POWER ON
                state = parts[1].upper() == "ON"
                self.hil.send_power_command(state)

            elif cmd == "RESET_GRAPH":
                self.hil.logger.reset_log()
                
            elif cmd == "DELAY":
                # DELAY 2.5
                time.sleep(float(parts[1]))
                
            elif cmd == "EXPECT_SPEED":
                # EXPECT_SPEED 0000 F 14 5.0
                addr = parts[1]
                direction = parts[2].upper()
                speed = int(parts[3])
                timeout = float(parts[4]) if len(parts) > 4 else 2.0
                
                success = self.hil.wait_for_motion(addr, direction, speed, timeout)
                if not success:
                    raise TimeoutError(f"Loco {addr} did not reach {direction}{speed} within {timeout}s")
                    
            elif cmd == "EXPECT_FUNC":
                # EXPECT_FUNC 0000 F0 ON 2.0
                addr = parts[1]
                func = parts[2].upper()
                state = parts[3].upper() == "ON"
                timeout = float(parts[4]) if len(parts) > 4 else 2.0
                
                success = self.hil.wait_for_function(addr, func, state, timeout)
                if not success:
                    raise TimeoutError(f"Loco {addr} {func} did not become {state} within {timeout}s")
                    
            # --- NEW COMMANDS ---
            
            elif cmd == "ECHO":
                # Strip out the word "ECHO " and print the rest of the line
                # Using split limit or slicing based on the original line preserves spaces in the message
                message = line[len(parts[0]):].strip()
                print(f"> {message}")
                
            elif cmd == "WAIT_KEY":
                # Wait for the user to hit Enter. 
                # Note: For true "any key" cross-platform support, you would need OS-specific libraries, 
                # but input() requiring Enter is standard and robust for CLI scripts.
                input("[Press Enter to continue...]")
    # (Inside ScriptRunner._execute_command...)
            elif cmd == "SAVE_GRAPH":
                # SAVE_GRAPH stop_test_graph.png
                filename = parts[1] if len(parts) > 1 else "hil_graph.png"
                self.hil.logger.end_test()
                generate_time_graph(self.hil.logger, filename)

            elif cmd == "EXPECT_SPEED_MIN":
                # EXPECT_SPEED_MIN 0000 F 0 3.5 6.0
                addr = parts[1]
                direction = parts[2].upper()
                speed = int(parts[3])
                min_time = float(parts[4])
                
                # Default timeout to 2 seconds past the min_time if not provided
                timeout = float(parts[5]) if len(parts) > 5 else (min_time + 2.0)
                
                success, error_msg = self.hil.wait_for_motion_min(addr, direction, speed, min_time, timeout)
                if not success:
                    raise TimeoutError(f"Loco {addr} EXPECT_SPEED_MIN Failed: {error_msg}")
                
            else:
                raise ValueError(f"Unknown command: {cmd}")
            
import sys
import time

if __name__ == "__main__":
    # Ensure we have at least a port and one script
    if len(sys.argv) < 3:
        print("Usage: python test_runner.py <SERIAL_PORT> <script1.txt> [script2.txt ...]")
        print("Example: python test_runner.py COM3 station_stop_test.txt reversing_test.txt")
        sys.exit(1)

    serial_port = sys.argv[1]
    test_scripts = sys.argv[2:]

    # Initialize the HIL connection
    hil = MotormanHIL(port=serial_port, baudrate=115200)
    hil.start()
    
    try:
        # Give the hardware a moment to initialize
        time.sleep(1)
        
        runner = ScriptRunner(hil)
        
        # Run your regression scripts
        all_passed = True
        for script_path in test_scripts:
            success = runner.run_script(script_path)
            if not success:
                all_passed = False
                print(f"--- Stopping execution due to failure in: {script_path} ---")
                break  # Stop running further scripts if one fails
                
        if not all_passed:
            sys.exit(1) # Exit with an error code for CI/CD pipelines
            
    finally:
        # Ensure the serial port is closed safely
        hil.stop()

        
        
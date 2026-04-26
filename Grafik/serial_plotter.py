import serial
import serial.tools.list_ports
import threading
import queue
import time
from datetime import datetime
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import numpy as np
from collections import deque
import tkinter.font as tkFont

class SerialPlotter:
    def __init__(self, root):
        self.root = root
        self.root.title("Swerve Drive Real-Time Plotter")
        
        # Set FIXED window size (bukan full screen)
        window_width = 1400
        window_height = 900
        
        # Get screen dimensions untuk centering
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        
        # Calculate position to center the window
        x_position = (screen_width - window_width) // 2
        y_position = (screen_height - window_height) // 2
        
        # Set window geometry with FIXED size
        self.root.geometry(f"{window_width}x{window_height}+{x_position}+{y_position}")
        
        # Prevent window from going full screen
        self.root.resizable(True, True)  # Bisa resize tapi tidak full screen
        self.root.maxsize(window_width + 200, window_height + 200)  # Batasi maksimum size
        
        # Data storage
        self.max_data_points = 2000
        self.timestamps = deque(maxlen=self.max_data_points)
        self.setpoint_angles = deque(maxlen=self.max_data_points)
        self.actual_angles = deque(maxlen=self.max_data_points)
        self.pwm_values = deque(maxlen=self.max_data_points)
        self.raw_errors = deque(maxlen=self.max_data_points)
        
        # Control variables
        self.serial_connection = None
        self.serial_thread = None
        self.is_running = False
        self.is_paused = False
        self.data_queue = queue.Queue()
        self.start_time = None
        
        # Plot variables
        self.zoom_level = 1.0
        self.display_range = 200
        
        # Setup GUI
        self.setup_gui()
        
        # Find and connect to Arduino/STM32
        self.find_serial_ports()
        
        # Start update timer
        self.update_plot()
        
    def setup_gui(self):
        # Configure style
        style = ttk.Style()
        style.configure('Title.TLabel', font=('Arial', 12, 'bold'))
        style.configure('Heading.TLabel', font=('Arial', 10, 'bold'))
        
        # Main container with padding
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # ========== TOP PANEL ==========
        top_frame = ttk.LabelFrame(main_frame, text="Control Panel", padding="10")
        top_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Row 1: Serial Connection
        conn_frame = ttk.Frame(top_frame)
        conn_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(conn_frame, text="Port:", style='Heading.TLabel').pack(side=tk.LEFT, padx=(0,5))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        self.port_combo.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(conn_frame, text="Baud:", style='Heading.TLabel').pack(side=tk.LEFT, padx=(10,5))
        self.baud_var = tk.StringVar(value="115200")
        baud_combo = ttk.Combobox(conn_frame, textvariable=self.baud_var, 
                                  values=["9600", "19200", "38400", "57600", "115200", "230400"], width=10)
        baud_combo.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection, width=12)
        self.connect_btn.pack(side=tk.LEFT, padx=20)
        
        # Status indicator
        self.status_var = tk.StringVar(value="⚫ Disconnected")
        status_label = ttk.Label(conn_frame, textvariable=self.status_var, font=('Arial', 10, 'bold'))
        status_label.pack(side=tk.LEFT, padx=20)
        
        # Row 2: Setpoint Control
        setpoint_frame = ttk.LabelFrame(top_frame, text="Setpoint Control", padding="5")
        setpoint_frame.pack(fill=tk.X, pady=5)
        
        # Manual setpoint entry
        manual_frame = ttk.Frame(setpoint_frame)
        manual_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(manual_frame, text="Target Angle (0-360°):", style='Heading.TLabel').pack(side=tk.LEFT, padx=5)
        self.target_angle_var = tk.StringVar(value="90")
        self.target_angle_entry = ttk.Entry(manual_frame, textvariable=self.target_angle_var, width=10)
        self.target_angle_entry.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(manual_frame, text="Speed (0-100%):", style='Heading.TLabel').pack(side=tk.LEFT, padx=(20,5))
        self.target_speed_var = tk.StringVar(value="100")
        self.target_speed_entry = ttk.Entry(manual_frame, textvariable=self.target_speed_var, width=10)
        self.target_speed_entry.pack(side=tk.LEFT, padx=5)
        
        self.send_setpoint_btn = ttk.Button(manual_frame, text="Send Setpoint", command=self.send_setpoint, width=15, state='disabled')
        self.send_setpoint_btn.pack(side=tk.LEFT, padx=20)
        
        # Preset angles
        preset_frame = ttk.Frame(setpoint_frame)
        preset_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(preset_frame, text="Presets:", style='Heading.TLabel').pack(side=tk.LEFT, padx=5)
        
        presets = [("0°", "0"), ("45°", "45"), ("90°", "90"), 
                   ("135°", "135"), ("180°", "180"), ("225°", "225"), 
                   ("270°", "270"), ("315°", "315"), ("360°", "360")]
        
        for text, value in presets:
            btn = ttk.Button(preset_frame, text=text, 
                           command=lambda v=value: self.set_preset_angle(v), width=6, state='disabled')
            btn.pack(side=tk.LEFT, padx=2)
            if not hasattr(self, 'preset_buttons'):
                self.preset_buttons = []
            self.preset_buttons.append(btn)
        
        # Row 3: Plot Control
        plot_control_frame = ttk.Frame(top_frame)
        plot_control_frame.pack(fill=tk.X, pady=5)
        
        # Left side - Buttons
        btn_frame = ttk.Frame(plot_control_frame)
        btn_frame.pack(side=tk.LEFT)
        
        self.start_btn = ttk.Button(btn_frame, text="Start Plot", command=self.start_plotting, state='disabled', width=10)
        self.start_btn.pack(side=tk.LEFT, padx=2)
        
        self.pause_btn = ttk.Button(btn_frame, text="Pause", command=self.toggle_pause, state='disabled', width=8)
        self.pause_btn.pack(side=tk.LEFT, padx=2)
        
        self.stop_btn = ttk.Button(btn_frame, text="Stop", command=self.stop_plotting, state='disabled', width=8)
        self.stop_btn.pack(side=tk.LEFT, padx=2)
        
        self.reset_btn = ttk.Button(btn_frame, text="Reset Data", command=self.reset_data, state='disabled', width=10)
        self.reset_btn.pack(side=tk.LEFT, padx=2)
        
        self.clear_btn = ttk.Button(btn_frame, text="Clear", command=self.clear_data, width=8)
        self.clear_btn.pack(side=tk.LEFT, padx=2)
        
        # Right side - Zoom controls
        zoom_frame = ttk.Frame(plot_control_frame)
        zoom_frame.pack(side=tk.RIGHT)
        
        ttk.Label(zoom_frame, text="Zoom:", style='Heading.TLabel').pack(side=tk.LEFT, padx=5)
        self.zoom_out_btn = ttk.Button(zoom_frame, text="Out", command=self.zoom_out, width=5)
        self.zoom_out_btn.pack(side=tk.LEFT, padx=2)
        
        self.zoom_in_btn = ttk.Button(zoom_frame, text="In", command=self.zoom_in, width=5)
        self.zoom_in_btn.pack(side=tk.LEFT, padx=2)
        
        self.zoom_fit_btn = ttk.Button(zoom_frame, text="Fit", command=self.zoom_fit, width=5)
        self.zoom_fit_btn.pack(side=tk.LEFT, padx=2)
        
        # Range slider
        range_frame = ttk.Frame(plot_control_frame)
        range_frame.pack(side=tk.RIGHT, padx=20)
        
        ttk.Label(range_frame, text="Points:", style='Heading.TLabel').pack(side=tk.LEFT, padx=5)
        self.range_var = tk.IntVar(value=200)
        self.range_slider = ttk.Scale(range_frame, from_=20, to=500, orient='horizontal', 
                                      variable=self.range_var, command=self.update_range, length=150)
        self.range_slider.pack(side=tk.LEFT, padx=5)
        self.range_label = ttk.Label(range_frame, text="200", width=4)
        self.range_label.pack(side=tk.LEFT, padx=2)
        
        # ========== MIDDLE PANEL (GRAPHS) ==========
        # Create matplotlib figure with FIXED size ratio
        self.fig = Figure(figsize=(12, 6), dpi=100)  # Reduced height
        self.fig.subplots_adjust(left=0.06, right=0.98, top=0.95, bottom=0.08, hspace=0.2)
        
        # Angle plot
        self.ax1 = self.fig.add_subplot(211)
        self.ax1.set_title('Angle Tracking', fontsize=11, fontweight='bold')
        self.ax1.set_xlabel('Time (s)', fontsize=9)
        self.ax1.set_ylabel('Angle (degrees)', fontsize=9)
        self.ax1.grid(True, alpha=0.3, linestyle='--')
        self.ax1.set_ylim(-100, 100)
        self.ax1.tick_params(labelsize=8)
        
        # PWM plot
        self.ax2 = self.fig.add_subplot(212)
        self.ax2.set_title('PWM Output', fontsize=11, fontweight='bold')
        self.ax2.set_xlabel('Time (s)', fontsize=9)
        self.ax2.set_ylabel('PWM', fontsize=9)
        self.ax2.grid(True, alpha=0.3, linestyle='--')
        self.ax2.set_ylim(-260, 260)
        self.ax2.tick_params(labelsize=8)
        
        # Create canvas
        graph_frame = ttk.Frame(main_frame)
        graph_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Add matplotlib toolbar (compact)
        toolbar_frame = ttk.Frame(graph_frame)
        toolbar_frame.pack(fill=tk.X)
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()
        
        # ========== BOTTOM PANEL (STATISTICS) ==========
        bottom_frame = ttk.LabelFrame(main_frame, text="Statistics", padding="5")
        bottom_frame.pack(fill=tk.X, pady=5)
        
        # Statistics display
        self.stats_text = tk.Text(bottom_frame, height=3, wrap=tk.WORD, font=('Consolas', 8))
        self.stats_text.pack(fill=tk.X)
        
        # Bind mouse events for panning
        self.canvas.mpl_connect('button_press_event', self.on_mouse_press)
        self.canvas.mpl_connect('button_release_event', self.on_mouse_release)
        self.canvas.mpl_connect('motion_notify_event', self.on_mouse_motion)
        
        self.panning = False
        self.pan_start = None
        
    def on_mouse_press(self, event):
        if event.button == 1 and event.inaxes:
            self.panning = True
            self.pan_start = (event.xdata, event.ydata)
            
    def on_mouse_release(self, event):
        self.panning = False
        self.pan_start = None
        
    def on_mouse_motion(self, event):
        if self.panning and event.xdata and event.ydata and self.pan_start and event.inaxes:
            dx = event.xdata - self.pan_start[0]
            dy = event.ydata - self.pan_start[1]
            
            ax = event.inaxes
            xlim = ax.get_xlim()
            ylim = ax.get_ylim()
            ax.set_xlim(xlim[0] - dx, xlim[1] - dx)
            ax.set_ylim(ylim[0] - dy, ylim[1] - dy)
            
            self.canvas.draw_idle()
            self.pan_start = (event.xdata, event.ydata)
            
    def find_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = []
        
        for port in ports:
            port_list.append(port.device)
            if any(x in port.description.lower() for x in ['arduino', 'stm', 'serial', 'usb']):
                self.port_var.set(port.device)
                
        self.port_combo['values'] = port_list
        if not self.port_var.get() and port_list:
            self.port_var.set(port_list[0])
            
    def toggle_connection(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.disconnect()
        else:
            self.connect()
            
    def connect(self):
        try:
            port = self.port_var.get()
            baud = int(self.baud_var.get())
            
            self.serial_connection = serial.Serial(port, baud, timeout=1)
            time.sleep(2)
            
            self.status_var.set(f"🟢 Connected to {port}")
            self.connect_btn.config(text="Disconnect")
            self.start_btn.config(state='normal')
            self.reset_btn.config(state='normal')
            self.send_setpoint_btn.config(state='normal')
            
            for btn in self.preset_buttons:
                btn.config(state='normal')
            
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {str(e)}")
            self.status_var.set("🔴 Connection failed")
            
    def disconnect(self):
        if self.serial_connection:
            self.is_running = False
            if self.serial_thread and self.serial_thread.is_alive():
                self.serial_thread.join(timeout=1)
                
            self.serial_connection.close()
            self.serial_connection = None
            
        self.status_var.set("⚫ Disconnected")
        self.connect_btn.config(text="Connect")
        self.start_btn.config(state='disabled')
        self.pause_btn.config(state='disabled', text="Pause")
        self.stop_btn.config(state='disabled')
        self.reset_btn.config(state='disabled')
        self.send_setpoint_btn.config(state='disabled')
        
        if hasattr(self, 'preset_buttons'):
            for btn in self.preset_buttons:
                btn.config(state='disabled')
        
    def send_setpoint(self):
        if not self.serial_connection or not self.serial_connection.is_open:
            messagebox.showwarning("Not Connected", "Please connect to serial port first")
            return
            
        try:
            angle = float(self.target_angle_var.get())
            speed = float(self.target_speed_var.get())
            
            if angle < 0 or angle > 360:
                messagebox.showerror("Invalid Angle", "Angle must be between 0 and 360 degrees")
                return
                
            if speed < 0 or speed > 100:
                messagebox.showerror("Invalid Speed", "Speed must be between 0 and 100%")
                return
                
            command = f"S{angle:.1f},{speed:.0f}\n"
            self.serial_connection.write(command.encode())
            self.status_var.set(f"✅ Setpoint sent: {angle}° at {speed}%")
            
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numbers")
            
    def set_preset_angle(self, angle):
        self.target_angle_var.set(angle)
        self.send_setpoint()
        
    def start_plotting(self):
        if not self.is_running and self.serial_connection and self.serial_connection.is_open:
            self.is_running = True
            self.is_paused = False
            self.start_time = time.time()
            self.start_btn.config(state='disabled')
            self.pause_btn.config(state='normal', text="Pause")
            self.stop_btn.config(state='normal')
            self.status_var.set("🟢 Plotting...")
            
            self.clear_data()
            
            self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.serial_thread.start()
            
    def read_serial_data(self):
        while self.is_running:
            try:
                if self.serial_connection and self.serial_connection.is_open:
                    line = self.serial_connection.readline().decode('utf-8').strip()
                    
                    if line and not self.is_paused:
                        data = self.parse_serial_line(line)
                        if data:
                            self.data_queue.put(data)
                            
            except Exception as e:
                print(f"Serial read error: {e}")
                time.sleep(0.1)
                
    def parse_serial_line(self, line):
        try:
            if 'A:' in line and 'T:' in line and 'Out:' in line:
                parts = line.split()
                
                angle = None
                target = None
                pwm = None
                
                for part in parts:
                    if part.startswith('A:'):
                        angle = float(part[2:])
                    elif part.startswith('T:'):
                        target = float(part[2:])
                    elif part.startswith('Out:'):
                        pwm = float(part[4:])
                        
                if angle is not None and target is not None and pwm is not None:
                    current_time = time.time() - self.start_time if self.start_time else 0
                    return {
                        'timestamp': current_time,
                        'angle': angle,
                        'target': target,
                        'pwm': pwm,
                        'error': target - angle
                    }
        except:
            pass
        return None
        
    def toggle_pause(self):
        self.is_paused = not self.is_paused
        if self.is_paused:
            self.pause_btn.config(text="Resume")
            self.status_var.set("⏸ Paused")
        else:
            self.pause_btn.config(text="Pause")
            self.status_var.set("🟢 Plotting...")
            
    def stop_plotting(self):
        self.is_running = False
        self.is_paused = False
        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.join(timeout=1)
            
        self.start_btn.config(state='normal')
        self.pause_btn.config(state='disabled', text="Pause")
        self.stop_btn.config(state='disabled')
        self.status_var.set("⏹ Stopped")
        
    def reset_data(self):
        self.clear_data()
        if self.start_time:
            self.start_time = time.time()
        self.status_var.set("🔄 Data reset")
        
    def clear_data(self):
        self.timestamps.clear()
        self.setpoint_angles.clear()
        self.actual_angles.clear()
        self.pwm_values.clear()
        self.raw_errors.clear()
        
    def zoom_out(self):
        self.zoom_level = min(5.0, self.zoom_level * 1.5)
        self.update_display_range()
        
    def zoom_in(self):
        self.zoom_level = max(0.1, self.zoom_level / 1.5)
        self.update_display_range()
        
    def zoom_fit(self):
        if self.timestamps:
            data_range = max(self.timestamps) - min(self.timestamps) if self.timestamps else 10
            self.display_range = max(20, min(500, int(data_range * 10)))
            self.range_var.set(self.display_range)
            self.zoom_level = 1.0
        else:
            self.display_range = 200
            self.range_var.set(200)
            self.zoom_level = 1.0
            
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.ax2.relim()
        self.ax2.autoscale_view()
            
    def update_range(self, value):
        self.display_range = int(float(value))
        self.range_label.config(text=str(self.display_range))
        
    def update_display_range(self):
        base_range = self.range_var.get()
        new_range = int(base_range * self.zoom_level)
        self.display_range = max(20, min(500, new_range))
        self.range_var.set(self.display_range)
        self.range_label.config(text=str(self.display_range))
        
    def update_plot(self):
        while not self.data_queue.empty():
            data = self.data_queue.get_nowait()
            
            self.timestamps.append(data['timestamp'])
            self.setpoint_angles.append(data['target'])
            self.actual_angles.append(data['angle'])
            self.pwm_values.append(data['pwm'])
            self.raw_errors.append(data['error'])
            
        self.ax1.clear()
        self.ax2.clear()
        
        n_points = len(self.timestamps)
        if n_points > 0:
            start_idx = max(0, n_points - self.display_range)
            
            display_times = list(self.timestamps)[start_idx:]
            display_setpoints = list(self.setpoint_angles)[start_idx:]
            display_actual = list(self.actual_angles)[start_idx:]
            display_pwm = list(self.pwm_values)[start_idx:]
            
            self.ax1.plot(display_times, display_setpoints, 'r-', label='Setpoint', linewidth=1.5, alpha=0.8)
            self.ax1.plot(display_times, display_actual, 'b-', label='Actual', linewidth=1.5)
            self.ax1.fill_between(display_times, display_setpoints, display_actual, alpha=0.2, color='gray')
            
            self.ax2.plot(display_times, display_pwm, 'g-', label='PWM', linewidth=1.5)
            self.ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.5)
            
            self.update_statistics()
            
        self.ax1.set_title('Angle Tracking', fontsize=11, fontweight='bold')
        self.ax1.set_xlabel('Time (s)', fontsize=9)
        self.ax1.set_ylabel('Angle (degrees)', fontsize=9)
        self.ax1.grid(True, alpha=0.3, linestyle='--')
        self.ax1.legend(loc='upper right', fontsize=8)
        self.ax1.set_ylim(-100, 100)
        self.ax1.tick_params(labelsize=8)
        
        self.ax2.set_title('PWM Output', fontsize=11, fontweight='bold')
        self.ax2.set_xlabel('Time (s)', fontsize=9)
        self.ax2.set_ylabel('PWM', fontsize=9)
        self.ax2.grid(True, alpha=0.3, linestyle='--')
        self.ax2.legend(loc='upper right', fontsize=8)
        self.ax2.set_ylim(-260, 260)
        self.ax2.tick_params(labelsize=8)
        
        self.canvas.draw_idle()
        self.root.after(50, self.update_plot)
        
    def update_statistics(self):
        if self.timestamps:
            current_error = self.raw_errors[-1] if self.raw_errors else 0
            avg_error = np.mean(self.raw_errors) if self.raw_errors else 0
            max_error = max(self.raw_errors, key=abs) if self.raw_errors else 0
            min_error = min(self.raw_errors) if self.raw_errors else 0
            std_error = np.std(self.raw_errors) if len(self.raw_errors) > 1 else 0
            
            avg_pwm = np.mean(self.pwm_values) if self.pwm_values else 0
            max_pwm = max(self.pwm_values, key=abs) if self.pwm_values else 0
            min_pwm = min(self.pwm_values) if self.pwm_values else 0
            
            runtime = time.time() - self.start_time if self.start_time else 0
            
            self.stats_text.delete(1.0, tk.END)
            stats = [
                f"Runtime: {runtime:.1f}s | Data: {len(self.timestamps)}/{self.max_data_points}",
                f"Error: Curr={current_error:.1f}° | Avg={avg_error:.1f}° | Max={max_error:.1f}°",
                f"PWM: Curr={self.pwm_values[-1] if self.pwm_values else 0:.0f} | Avg={avg_pwm:.0f} | Max={max_pwm:.0f}"
            ]
            self.stats_text.insert(1.0, '\n'.join(stats))
            
    def on_closing(self):
        self.is_running = False
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        self.root.destroy()

def main():
    root = tk.Tk()
    app = SerialPlotter(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
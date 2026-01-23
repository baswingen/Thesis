"""
Trial GUI Module
=================
Tkinter-based GUI for trial data collection with real-time signal visualization.

Features:
- Real-time EMG and IMU signal plots
- Exercise instructions display
- Trial progress tracking
- Keyboard controls
- State indicators
"""

import tkinter as tk
from tkinter import ttk, font as tkfont
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
from typing import Dict, List, Optional, Tuple, Callable
from enum import Enum
import threading


# =============================================================================
# STATE DEFINITIONS
# =============================================================================

class TrialState(Enum):
    """Trial state machine states."""
    IDLE = "idle"
    CALIBRATING = "calibrating"
    READY = "ready"
    COUNTDOWN = "countdown"
    RECORDING = "recording"
    SAVING = "saving"
    COMPLETED = "completed"


# =============================================================================
# MAIN GUI CLASS
# =============================================================================

class TrialGUI:
    """
    Main GUI window for trial data collection.
    
    Layout:
    - Top: Exercise instruction panel (large, color-coded)
    - Middle: Real-time signal plots (EMG + IMU)
    - Bottom: Status panel with progress and keyboard shortcuts
    """
    
    def __init__(self, 
                 config: Optional[Dict] = None,
                 on_key_press: Optional[Callable[[str], None]] = None,
                 on_test_hardware: Optional[Callable[[], None]] = None):
        """
        Initialize GUI.
        
        Args:
            config: GUI configuration dictionary
            on_key_press: Callback for keyboard events, signature: callback(key: str)
            on_test_hardware: Callback for test hardware button, signature: callback()
        """
        # Configuration
        self.config = config or self._default_config()
        self.on_key_press = on_key_press
        self.on_test_hardware = on_test_hardware
        
        # State
        self.state = TrialState.IDLE
        self.current_exercise = ""
        self.current_instruction = ""
        self.trial_number = 0
        self.total_trials = 0
        self.trial_progress = 0.0
        
        # Data buffers for plotting
        self.emg_buffer_size = int(self.config['emg_plot_window_s'] * 2048)  # Assume 2kHz
        self.imu_buffer_size = int(self.config['imu_plot_window_s'] * 200)   # Assume 200Hz
        
        self.emg_time = deque(maxlen=self.emg_buffer_size)
        self.emg_data = [deque(maxlen=self.emg_buffer_size) 
                        for _ in range(self.config['emg_channels_to_plot'])]
        
        self.imu_time = deque(maxlen=self.imu_buffer_size)
        self.imu_accel = [deque(maxlen=self.imu_buffer_size) for _ in range(3)]
        self.imu_gyro = [deque(maxlen=self.imu_buffer_size) for _ in range(3)]
        
        # Thread safety
        self.update_lock = threading.Lock()
        
        # Create GUI
        self.root = None
        self.canvas = None
        self._create_gui()
    
    def _default_config(self) -> Dict:
        """Get default GUI configuration."""
        return {
            'window_title': 'Trial Data Collection',
            'window_width': 1200,
            'window_height': 700,
            'fullscreen': False,
            'plot_update_interval_ms': 50,
            'status_update_interval_ms': 100,
            'emg_plot_window_s': 5.0,
            'imu_plot_window_s': 5.0,
            'emg_channels_to_plot': 4,
            'color_idle': '#4CAF50',
            'color_ready': '#2196F3',
            'color_recording': '#F44336',
            'color_saving': '#FF9800',
            'color_completed': '#9C27B0',
            'font_instruction': ('Arial', 24, 'bold'),
            'font_status': ('Arial', 14),
            'font_info': ('Arial', 12),
        }
    
    def _create_gui(self):
        """Create the main GUI window."""
        self.root = tk.Tk()
        self.root.title(self.config['window_title'])
        self.root.geometry(f"{self.config['window_width']}x{self.config['window_height']}")
        
        # Bind keyboard events
        self.root.bind('<KeyPress>', self._handle_keypress)
        
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(1, weight=1)  # Plots get most space
        
        # Create panels
        self._create_instruction_panel(main_frame)
        self._create_plot_panel(main_frame)
        self._create_status_panel(main_frame)
        
        # Start update loops
        self._schedule_plot_update()
        self._schedule_status_update()
        
        # Fullscreen if configured
        if self.config['fullscreen']:
            self.root.attributes('-fullscreen', True)
            self.root.bind('<Escape>', lambda e: self.root.attributes('-fullscreen', False))
    
    def _create_instruction_panel(self, parent):
        """Create top instruction panel."""
        panel = ttk.Frame(parent, relief='ridge', borderwidth=2)
        panel.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # State indicator
        self.state_label = tk.Label(
            panel,
            text="IDLE",
            font=('Arial', 16, 'bold'),
            bg=self.config['color_idle'],
            fg='white',
            width=15
        )
        self.state_label.pack(side=tk.TOP, fill=tk.X, pady=5)
        
        # Exercise name
        self.exercise_label = tk.Label(
            panel,
            text="Ready to start",
            font=self.config['font_instruction'],
            bg='white',
            fg='black'
        )
        self.exercise_label.pack(side=tk.TOP, fill=tk.X, pady=10)
        
        # Instruction text
        self.instruction_label = tk.Label(
            panel,
            text="Press SPACE to begin first trial",
            font=self.config['font_status'],
            bg='white',
            fg='gray',
            wraplength=self.config['window_width'] - 50,
            justify=tk.CENTER
        )
        self.instruction_label.pack(side=tk.TOP, fill=tk.X, pady=5)
        
        # Progress bar
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(
            panel,
            variable=self.progress_var,
            maximum=100,
            mode='determinate',
            length=400
        )
        self.progress_bar.pack(side=tk.TOP, pady=10)
        
        # Trial counter
        self.trial_counter_label = tk.Label(
            panel,
            text="Trial: 0 / 0",
            font=self.config['font_info'],
            bg='white'
        )
        self.trial_counter_label.pack(side=tk.TOP)
    
    def _create_plot_panel(self, parent):
        """Create middle panel with real-time plots."""
        panel = ttk.Frame(parent, relief='ridge', borderwidth=2)
        panel.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        
        # Create matplotlib figure (adjusted for smaller window)
        self.fig = Figure(figsize=(11, 5.5), dpi=100)
        self.fig.patch.set_facecolor('white')
        
        # Create subplots: 2 rows (EMG, IMU)
        gs = self.fig.add_gridspec(2, 1, height_ratios=[1.5, 1], hspace=0.3)
        
        # EMG subplot
        self.ax_emg = self.fig.add_subplot(gs[0])
        self.ax_emg.set_title('EMG Signals', fontsize=12, fontweight='bold')
        self.ax_emg.set_xlabel('Time (s)', fontsize=10)
        self.ax_emg.set_ylabel('Amplitude (µV)', fontsize=10)
        self.ax_emg.grid(True, alpha=0.3)
        self.emg_lines = []
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
        for i in range(self.config['emg_channels_to_plot']):
            line, = self.ax_emg.plot([], [], colors[i % len(colors)], 
                                     linewidth=1, label=f'CH{i+1}')
            self.emg_lines.append(line)
        self.ax_emg.legend(loc='upper right', fontsize=8)
        
        # IMU subplot (accel and gyro)
        self.ax_imu = self.fig.add_subplot(gs[1])
        self.ax_imu.set_title('IMU Data (Acceleration)', fontsize=12, fontweight='bold')
        self.ax_imu.set_xlabel('Time (s)', fontsize=10)
        self.ax_imu.set_ylabel('Acceleration (g)', fontsize=10)
        self.ax_imu.grid(True, alpha=0.3)
        self.imu_lines = []
        imu_colors = ['r', 'g', 'b']
        imu_labels = ['X', 'Y', 'Z']
        for i in range(3):
            line, = self.ax_imu.plot([], [], imu_colors[i], 
                                    linewidth=1, label=imu_labels[i])
            self.imu_lines.append(line)
        self.ax_imu.legend(loc='upper right', fontsize=8)
        
        # Embed in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=panel)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    
    def _create_status_panel(self, parent):
        """Create bottom status panel."""
        panel = ttk.Frame(parent, relief='ridge', borderwidth=2)
        panel.grid(row=2, column=0, sticky=(tk.W, tk.E))
        
        # Status text
        self.status_label = tk.Label(
            panel,
            text="Status: Idle | Waiting to start",
            font=self.config['font_info'],
            bg='white',
            anchor=tk.W
        )
        self.status_label.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)
        
        # Button frame for hardware test
        button_frame = ttk.Frame(panel)
        button_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)
        
        # Test Hardware button
        self.test_hardware_button = ttk.Button(
            button_frame,
            text="🔍 Test Hardware (T)",
            command=self._on_test_hardware_clicked,
            width=20
        )
        self.test_hardware_button.pack(side=tk.LEFT, padx=5)
        
        # Button state label
        self.hardware_test_label = tk.Label(
            button_frame,
            text="Launch visualization to verify EMG and IMU signals",
            font=('Arial', 9),
            fg='gray'
        )
        self.hardware_test_label.pack(side=tk.LEFT, padx=10)
        
        # Keyboard shortcuts
        shortcuts_text = (
            "Keyboard: [SPACE] Start/Stop | [N] Next Trial | [R] Repeat | "
            "[C] Calibrate IMU | [T] Test Hardware | [Q] Quit"
        )
        shortcuts_label = tk.Label(
            panel,
            text=shortcuts_text,
            font=('Arial', 10),
            bg='lightgray',
            fg='black'
        )
        shortcuts_label.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)
        
        # Signal quality indicators
        quality_frame = ttk.Frame(panel)
        quality_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)
        
        tk.Label(quality_frame, text="EMG Rate:", font=('Arial', 9)).pack(side=tk.LEFT, padx=5)
        self.emg_rate_label = tk.Label(quality_frame, text="0 Hz", 
                                       font=('Arial', 9, 'bold'), fg='blue')
        self.emg_rate_label.pack(side=tk.LEFT, padx=5)
        
        tk.Label(quality_frame, text="IMU Rate:", font=('Arial', 9)).pack(side=tk.LEFT, padx=15)
        self.imu_rate_label = tk.Label(quality_frame, text="0 Hz", 
                                       font=('Arial', 9, 'bold'), fg='blue')
        self.imu_rate_label.pack(side=tk.LEFT, padx=5)
        
        tk.Label(quality_frame, text="EMG RMS:", font=('Arial', 9)).pack(side=tk.LEFT, padx=15)
        self.emg_rms_label = tk.Label(quality_frame, text="0.0", 
                                      font=('Arial', 9, 'bold'), fg='green')
        self.emg_rms_label.pack(side=tk.LEFT, padx=5)
    
    def _on_test_hardware_clicked(self):
        """Handle test hardware button click."""
        if self.on_test_hardware:
            self.on_test_hardware()
    
    def _handle_keypress(self, event):
        """Handle keyboard events."""
        key = event.char.lower()
        keysym = event.keysym.lower()
        
        # Map keys
        if keysym == 'space':
            key = 'space'
        elif keysym == 'escape':
            key = 'escape'
        
        # Handle 'T' key for test hardware
        if key == 't':
            self._on_test_hardware_clicked()
            return
        
        # Call callback
        if self.on_key_press:
            self.on_key_press(key)
    
    def _schedule_plot_update(self):
        """Schedule periodic plot updates."""
        if not self.root or not self.root.winfo_exists():
            return
        
        self._update_plots()
        
        if self.root and self.root.winfo_exists():
            self.root.after(self.config['plot_update_interval_ms'], self._schedule_plot_update)
    
    def _schedule_status_update(self):
        """Schedule periodic status updates."""
        if not self.root or not self.root.winfo_exists():
            return
        
        self._update_status()
        
        if self.root and self.root.winfo_exists():
            self.root.after(self.config['status_update_interval_ms'], self._schedule_status_update)
    
    def _update_plots(self):
        """Update matplotlib plots with buffered data."""
        # Check if window still exists
        if not self.root or not self.root.winfo_exists():
            return
        
        try:
            with self.update_lock:
                # Update EMG plots
                if len(self.emg_time) > 0:
                    time_array = np.array(self.emg_time)
                    
                    for i, line in enumerate(self.emg_lines):
                        if i < len(self.emg_data) and len(self.emg_data[i]) > 0:
                            data_array = np.array(self.emg_data[i])
                            line.set_data(time_array, data_array)
                    
                    # Auto-scale
                    self.ax_emg.relim()
                    self.ax_emg.autoscale_view()
                    
                    # Set x-axis to show window
                    window = self.config['emg_plot_window_s']
                    if len(time_array) > 0 and time_array[-1] > window:
                        self.ax_emg.set_xlim(time_array[-1] - window, time_array[-1])
                
                # Update IMU plots
                if len(self.imu_time) > 0:
                    time_array = np.array(self.imu_time)
                    
                    for i, line in enumerate(self.imu_lines):
                        if i < len(self.imu_accel) and len(self.imu_accel[i]) > 0:
                            data_array = np.array(self.imu_accel[i])
                            line.set_data(time_array, data_array)
                    
                    # Auto-scale
                    self.ax_imu.relim()
                    self.ax_imu.autoscale_view()
                    
                    # Set x-axis
                    window = self.config['imu_plot_window_s']
                    if len(time_array) > 0 and time_array[-1] > window:
                        self.ax_imu.set_xlim(time_array[-1] - window, time_array[-1])
                
                # Redraw
                if self.canvas:
                    self.canvas.draw_idle()
        
        except Exception as e:
            # Silently handle plotting errors (may occur during shutdown)
            pass
    
    def _update_status(self):
        """Update status displays."""
        # Update trial counter
        self.trial_counter_label.config(
            text=f"Trial: {self.trial_number} / {self.total_trials}"
        )
        
        # Update progress bar
        self.progress_var.set(self.trial_progress * 100)
    
    # =========================================================================
    # PUBLIC API
    # =========================================================================
    
    def set_state(self, state: TrialState):
        """
        Set GUI state.
        
        Args:
            state: New state
        """
        self.state = state
        
        # Update state label
        state_colors = {
            TrialState.IDLE: self.config['color_idle'],
            TrialState.CALIBRATING: self.config['color_saving'],
            TrialState.READY: self.config['color_ready'],
            TrialState.COUNTDOWN: self.config['color_saving'],
            TrialState.RECORDING: self.config['color_recording'],
            TrialState.SAVING: self.config['color_saving'],
            TrialState.COMPLETED: self.config['color_completed'],
        }
        
        self.state_label.config(
            text=state.value.upper(),
            bg=state_colors.get(state, 'gray')
        )
    
    def set_exercise(self, name: str, instruction: str):
        """
        Set current exercise information.
        
        Args:
            name: Exercise name
            instruction: Instruction text
        """
        self.current_exercise = name
        self.current_instruction = instruction
        
        self.exercise_label.config(text=name)
        self.instruction_label.config(text=instruction)
    
    def set_trial_info(self, trial_number: int, total_trials: int):
        """
        Set trial counter information.
        
        Args:
            trial_number: Current trial number (1-based)
            total_trials: Total number of trials
        """
        self.trial_number = trial_number
        self.total_trials = total_trials
    
    def set_progress(self, progress: float):
        """
        Set trial progress (0.0 to 1.0).
        
        Args:
            progress: Progress fraction
        """
        self.trial_progress = max(0.0, min(1.0, progress))
    
    def set_status(self, status: str):
        """
        Set status text.
        
        Args:
            status: Status message
        """
        self.status_label.config(text=status)
    
    def update_emg_data(self, timestamps: np.ndarray, data: np.ndarray):
        """
        Update EMG plot data.
        
        Args:
            timestamps: (N,) array of timestamps
            data: (N, C) array of EMG data
        """
        with self.update_lock:
            n_samples = len(timestamps)
            n_channels = min(data.shape[1], len(self.emg_data))
            
            for i in range(n_samples):
                self.emg_time.append(timestamps[i])
                for ch in range(n_channels):
                    self.emg_data[ch].append(data[i, ch])
    
    def update_imu_data(self, timestamps: np.ndarray, accel: np.ndarray, 
                       gyro: Optional[np.ndarray] = None):
        """
        Update IMU plot data.
        
        Args:
            timestamps: (N,) array of timestamps
            accel: (N, 3) array of acceleration
            gyro: (N, 3) array of gyro (optional)
        """
        with self.update_lock:
            n_samples = len(timestamps)
            
            for i in range(n_samples):
                self.imu_time.append(timestamps[i])
                for axis in range(3):
                    self.imu_accel[axis].append(accel[i, axis])
                    if gyro is not None:
                        self.imu_gyro[axis].append(gyro[i, axis])
    
    def update_signal_quality(self, emg_rate: float, imu_rate: float, emg_rms: float):
        """
        Update signal quality indicators.
        
        Args:
            emg_rate: EMG sample rate (Hz)
            imu_rate: IMU sample rate (Hz)
            emg_rms: EMG RMS value
        """
        self.emg_rate_label.config(text=f"{emg_rate:.0f} Hz")
        self.imu_rate_label.config(text=f"{imu_rate:.0f} Hz")
        self.emg_rms_label.config(text=f"{emg_rms:.2f}")
    
    def clear_plots(self):
        """Clear all plot data buffers."""
        with self.update_lock:
            self.emg_time.clear()
            for buf in self.emg_data:
                buf.clear()
            
            self.imu_time.clear()
            for buf in self.imu_accel:
                buf.clear()
            for buf in self.imu_gyro:
                buf.clear()
    
    def show_message(self, title: str, message: str):
        """
        Show a message dialog.
        
        Args:
            title: Dialog title
            message: Message text
        """
        from tkinter import messagebox
        messagebox.showinfo(title, message)
    
    def show_error(self, title: str, message: str):
        """
        Show an error dialog.
        
        Args:
            title: Dialog title
            message: Error message
        """
        from tkinter import messagebox
        messagebox.showerror(title, message)
    
    def ask_yes_no(self, title: str, message: str) -> bool:
        """
        Ask a yes/no question.
        
        Args:
            title: Dialog title
            message: Question text
        
        Returns:
            True if yes, False if no
        """
        from tkinter import messagebox
        return messagebox.askyesno(title, message)
    
    def run(self):
        """Start the GUI main loop (blocking)."""
        self.root.mainloop()
    
    def close(self):
        """Close the GUI window."""
        if self.root:
            try:
                self.root.quit()
            except:
                pass
            try:
                self.root.destroy()
            except:
                pass
            self.root = None


# =============================================================================
# DEMO
# =============================================================================

if __name__ == '__main__':
    import time
    import random
    
    def on_key(key):
        print(f"Key pressed: {key}")
        if key == 'q':
            gui.close()
    
    # Create GUI
    gui = TrialGUI(on_key_press=on_key)
    
    # Set initial state
    gui.set_state(TrialState.READY)
    gui.set_exercise("Wrist Flexion", "Flex your wrist and hold")
    gui.set_trial_info(1, 10)
    
    # Simulate data updates
    def update_data():
        if gui.root and gui.root.winfo_exists():
            # Generate fake EMG data
            t = time.time() % 1000
            emg_data = np.random.randn(10, 4) * 100
            timestamps = np.linspace(t, t + 0.01, 10)
            gui.update_emg_data(timestamps, emg_data)
            
            # Generate fake IMU data
            accel = np.random.randn(5, 3) * 0.1 + np.array([0, 0, 1])
            gyro = np.random.randn(5, 3) * 0.5
            timestamps_imu = np.linspace(t, t + 0.025, 5)
            gui.update_imu_data(timestamps_imu, accel, gyro)
            
            # Update quality
            gui.update_signal_quality(2000, 200, 45.6)
            
            # Update progress
            progress = (time.time() % 5) / 5.0
            gui.set_progress(progress)
            
            # Schedule next update
            gui.root.after(50, update_data)
    
    # Start data updates
    gui.root.after(100, update_data)
    
    # Run GUI
    print("Trial GUI Demo")
    print("Press Q to quit")
    gui.run()

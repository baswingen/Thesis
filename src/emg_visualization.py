"""
EMG Visualization Module
========================
Real-time visualization utilities for EMG signals.

Features:
- Real-time plotting with matplotlib
- Multi-channel display
- Filtered signal comparison
- Auto-saving plots
"""

import numpy as np
from typing import Optional, List, Tuple
from collections import deque
from datetime import datetime
from pathlib import Path

# Optional matplotlib for visualization
try:
    import matplotlib
    matplotlib.use('TkAgg')  # Use TkAgg backend for better compatibility
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib not available - visualization disabled")


class EMGPlotter:
    """
    Real-time EMG signal plotter.
    
    Displays raw and optionally filtered EMG signals in real-time.
    """
    
    def __init__(self, 
                 sample_rate: float,
                 window_seconds: float = 5.0,
                 show_filtered: bool = False,
                 title: str = "EMG Signal"):
        """
        Initialize EMG plotter.
        
        Args:
            sample_rate: Sample rate in Hz
            window_seconds: Time window to display (seconds)
            show_filtered: Show filtered signal in second subplot
            title: Plot title
        
        Raises:
            RuntimeError: If matplotlib not available
        """
        if not MATPLOTLIB_AVAILABLE:
            raise RuntimeError("matplotlib is required for visualization. Install with: pip install matplotlib")
        
        self.sample_rate = sample_rate
        self.window_seconds = window_seconds
        self.show_filtered = show_filtered
        self.title = title
        
        # Data buffers
        buffer_size = int(window_seconds * sample_rate)
        self.time_buffer = deque(maxlen=buffer_size)
        self.raw_buffer = deque(maxlen=buffer_size)
        
        if show_filtered:
            self.time_filtered_buffer = deque(maxlen=buffer_size)
            self.filtered_buffer = deque(maxlen=buffer_size)
        
        # Plot elements
        self.fig = None
        self.axes = []
        self.lines = {}
        self.time_offset = 0.0
        
        self._setup_plot()
    
    def _setup_plot(self):
        """Setup matplotlib figure and axes."""
        if self.show_filtered:
            self.fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
            self.axes = [ax1, ax2]
            
            # Raw signal subplot
            ax1.set_title(f'Raw EMG Signal - {self.title}', fontsize=12, fontweight='bold')
            ax1.set_ylabel('Amplitude (µV)', fontsize=10)
            ax1.grid(True, alpha=0.3)
            line1, = ax1.plot([], [], 'b-', linewidth=0.8, label='Raw')
            ax1.legend(loc='upper right')
            self.lines['raw'] = line1
            
            # Filtered signal subplot
            ax2.set_title(f'Filtered EMG Signal', fontsize=12, fontweight='bold')
            ax2.set_xlabel('Time (s)', fontsize=10)
            ax2.set_ylabel('Amplitude (µV)', fontsize=10)
            ax2.grid(True, alpha=0.3)
            line2, = ax2.plot([], [], 'r-', linewidth=0.8, label='Filtered')
            ax2.legend(loc='upper right')
            self.lines['filtered'] = line2
        else:
            self.fig, ax1 = plt.subplots(1, 1, figsize=(12, 6))
            self.axes = [ax1]
            
            ax1.set_title(f'EMG Signal - {self.title} @ {self.sample_rate:.0f} Hz',
                         fontsize=12, fontweight='bold')
            ax1.set_xlabel('Time (s)', fontsize=10)
            ax1.set_ylabel('Amplitude (µV)', fontsize=10)
            ax1.grid(True, alpha=0.3)
            line1, = ax1.plot([], [], 'b-', linewidth=0.8, label='EMG')
            ax1.legend(loc='upper right')
            self.lines['raw'] = line1
        
        plt.tight_layout()
        plt.ion()
        plt.show(block=False)
    
    def update(self, 
               raw_data: np.ndarray, 
               filtered_data: Optional[np.ndarray] = None):
        """
        Update plot with new data.
        
        Args:
            raw_data: 1D array of raw EMG samples
            filtered_data: Optional 1D array of filtered EMG samples
        """
        if not MATPLOTLIB_AVAILABLE or self.fig is None:
            return
        
        n_samples = len(raw_data)
        dt = 1.0 / self.sample_rate
        
        # Create time values for this chunk
        chunk_times = self.time_offset + np.arange(n_samples) * dt
        self.time_offset = chunk_times[-1] + dt
        
        # Add to raw buffers (skip NaNs)
        for t, val in zip(chunk_times, raw_data):
            if not np.isnan(val):
                self.time_buffer.append(t)
                self.raw_buffer.append(val)
        
        # Add to filtered buffers if provided
        if filtered_data is not None and self.show_filtered:
            for t, val in zip(chunk_times, filtered_data):
                if not np.isnan(val):
                    self.time_filtered_buffer.append(t)
                    self.filtered_buffer.append(val)
        
        # Update raw plot
        if len(self.time_buffer) > 1:
            time_array = np.array(self.time_buffer)
            raw_array = np.array(self.raw_buffer)
            
            if 'raw' in self.lines:
                self.lines['raw'].set_data(time_array, raw_array)
                self.axes[0].relim()
                self.axes[0].autoscale_view()
                
                # Set x-axis to show window
                if time_array[-1] > self.window_seconds:
                    self.axes[0].set_xlim(time_array[-1] - self.window_seconds, time_array[-1])
        
        # Update filtered plot
        if 'filtered' in self.lines and self.show_filtered and len(self.filtered_buffer) > 1:
            tf = np.array(self.time_filtered_buffer)
            yf = np.array(self.filtered_buffer)
            
            if len(tf) == len(yf) and len(tf) > 1:
                self.lines['filtered'].set_data(tf, yf)
                self.axes[1].relim()
                self.axes[1].autoscale_view()
                
                if tf[-1] > self.window_seconds:
                    self.axes[1].set_xlim(tf[-1] - self.window_seconds, tf[-1])
        
        # Redraw
        try:
            self.fig.canvas.draw_idle()
            plt.pause(0.001)
        except:
            pass
    
    def save(self, filename: Optional[str] = None, dpi: int = 150):
        """
        Save current plot to file.
        
        Args:
            filename: Output filename. If None, generates timestamp-based name
            dpi: Resolution in dots per inch
        """
        if not MATPLOTLIB_AVAILABLE or self.fig is None:
            return
        
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"emg_plot_{timestamp}.png"
        
        try:
            self.fig.savefig(filename, dpi=dpi, bbox_inches='tight')
            print(f"[OK] Plot saved to '{filename}'")
        except Exception as e:
            print(f"[!] Error saving plot: {e}")
    
    def close(self):
        """Close the plot."""
        if MATPLOTLIB_AVAILABLE and self.fig is not None:
            plt.close(self.fig)
            self.fig = None
    
    def show(self, block: bool = True):
        """
        Show the plot.
        
        Args:
            block: If True, block until window is closed
        """
        if MATPLOTLIB_AVAILABLE and self.fig is not None:
            plt.show(block=block)
    
    def clear(self):
        """Clear all data buffers."""
        self.time_buffer.clear()
        self.raw_buffer.clear()
        
        if self.show_filtered:
            self.time_filtered_buffer.clear()
            self.filtered_buffer.clear()
        
        self.time_offset = 0.0
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()


class MultiChannelPlotter:
    """
    Multi-channel EMG plotter for displaying multiple signals simultaneously.
    """
    
    def __init__(self,
                 sample_rate: float,
                 num_channels: int,
                 channel_names: Optional[List[str]] = None,
                 window_seconds: float = 5.0):
        """
        Initialize multi-channel plotter.
        
        Args:
            sample_rate: Sample rate in Hz
            num_channels: Number of channels to display
            channel_names: Optional list of channel names
            window_seconds: Time window to display (seconds)
        
        Raises:
            RuntimeError: If matplotlib not available
        """
        if not MATPLOTLIB_AVAILABLE:
            raise RuntimeError("matplotlib is required for visualization")
        
        self.sample_rate = sample_rate
        self.num_channels = num_channels
        self.window_seconds = window_seconds
        
        if channel_names is None:
            self.channel_names = [f"CH{i+1}" for i in range(num_channels)]
        else:
            self.channel_names = channel_names[:num_channels]
        
        # Data buffers
        buffer_size = int(window_seconds * sample_rate)
        self.time_buffer = deque(maxlen=buffer_size)
        self.data_buffers = [deque(maxlen=buffer_size) for _ in range(num_channels)]
        
        # Plot elements
        self.fig = None
        self.axes = []
        self.lines = []
        self.time_offset = 0.0
        
        self._setup_plot()
    
    def _setup_plot(self):
        """Setup matplotlib figure and axes."""
        self.fig, self.axes = plt.subplots(self.num_channels, 1, 
                                           figsize=(12, 2*self.num_channels),
                                           sharex=True)
        
        if self.num_channels == 1:
            self.axes = [self.axes]
        
        for i, (ax, name) in enumerate(zip(self.axes, self.channel_names)):
            ax.set_title(f'{name}', fontsize=10)
            ax.set_ylabel('µV', fontsize=9)
            ax.grid(True, alpha=0.3)
            line, = ax.plot([], [], 'b-', linewidth=0.8)
            self.lines.append(line)
            
            if i == self.num_channels - 1:
                ax.set_xlabel('Time (s)', fontsize=10)
        
        plt.tight_layout()
        plt.ion()
        plt.show(block=False)
    
    def update(self, data: np.ndarray):
        """
        Update plot with new multi-channel data.
        
        Args:
            data: 2D array of shape (num_samples, num_channels)
        """
        if not MATPLOTLIB_AVAILABLE or self.fig is None:
            return
        
        n_samples = data.shape[0]
        dt = 1.0 / self.sample_rate
        
        # Create time values
        chunk_times = self.time_offset + np.arange(n_samples) * dt
        self.time_offset = chunk_times[-1] + dt
        
        # Update time buffer
        for t in chunk_times:
            self.time_buffer.append(t)
        
        # Update each channel
        for ch_idx in range(min(self.num_channels, data.shape[1])):
            for val in data[:, ch_idx]:
                if not np.isnan(val):
                    self.data_buffers[ch_idx].append(val)
                else:
                    self.data_buffers[ch_idx].append(np.nan)
        
        # Update plots
        if len(self.time_buffer) > 1:
            time_array = np.array(self.time_buffer)
            
            for ch_idx, (line, ax, data_buf) in enumerate(zip(self.lines, self.axes, self.data_buffers)):
                if len(data_buf) > 1:
                    data_array = np.array(data_buf)
                    line.set_data(time_array[-len(data_array):], data_array)
                    ax.relim()
                    ax.autoscale_view()
                    
                    if time_array[-1] > self.window_seconds:
                        ax.set_xlim(time_array[-1] - self.window_seconds, time_array[-1])
        
        # Redraw
        try:
            self.fig.canvas.draw_idle()
            plt.pause(0.001)
        except:
            pass
    
    def save(self, filename: Optional[str] = None, dpi: int = 150):
        """Save current plot to file."""
        if not MATPLOTLIB_AVAILABLE or self.fig is None:
            return
        
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"emg_multichannel_{timestamp}.png"
        
        try:
            self.fig.savefig(filename, dpi=dpi, bbox_inches='tight')
            print(f"[OK] Plot saved to '{filename}'")
        except Exception as e:
            print(f"[!] Error saving plot: {e}")
    
    def close(self):
        """Close the plot."""
        if MATPLOTLIB_AVAILABLE and self.fig is not None:
            plt.close(self.fig)
            self.fig = None
    
    def show(self, block: bool = True):
        """Show the plot."""
        if MATPLOTLIB_AVAILABLE and self.fig is not None:
            plt.show(block=block)
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()


def quick_plot_emg(data: np.ndarray, 
                   sample_rate: float,
                   title: str = "EMG Signal",
                   save_to: Optional[str] = None):
    """
    Quick utility to plot EMG data (non-real-time).
    
    Args:
        data: 1D or 2D array of EMG data
        sample_rate: Sample rate in Hz
        title: Plot title
        save_to: Optional filename to save plot
    """
    if not MATPLOTLIB_AVAILABLE:
        print("matplotlib not available for plotting")
        return
    
    # Handle 1D or 2D data
    if data.ndim == 1:
        data = data.reshape(-1, 1)
    
    num_samples, num_channels = data.shape
    time = np.arange(num_samples) / sample_rate
    
    # Create plot
    if num_channels == 1:
        fig, ax = plt.subplots(figsize=(12, 6))
        ax.plot(time, data[:, 0], 'b-', linewidth=0.8)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Amplitude (µV)')
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
    else:
        fig, axes = plt.subplots(num_channels, 1, figsize=(12, 2*num_channels), sharex=True)
        if num_channels == 1:
            axes = [axes]
        
        for i, ax in enumerate(axes):
            ax.plot(time, data[:, i], 'b-', linewidth=0.8)
            ax.set_ylabel(f'CH{i+1} (µV)')
            ax.grid(True, alpha=0.3)
            if i == 0:
                ax.set_title(title)
            if i == num_channels - 1:
                ax.set_xlabel('Time (s)')
    
    plt.tight_layout()
    
    if save_to:
        fig.savefig(save_to, dpi=150, bbox_inches='tight')
        print(f"[OK] Plot saved to '{save_to}'")
    
    plt.show()

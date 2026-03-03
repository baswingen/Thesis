"""
Trial Configuration
===================

Configuration for the experimental trial, including hardware, participant, 
and trial logic parameters.
"""

from typing import List, Dict

# 1. STM32 Hardware Connections
# Pins as defined in STM32_all_in_python_binary.ino
STM32_CONNECTIONS: Dict[str, str] = {
    "PRBS Output": "PA8 (Digital Out)",
    "IMU 1 (I2C)": "SDA: PA10, SCL: PA9",
    "IMU 2 (I2C)": "SDA: PA3, SCL: PA2",
    "Matrix Rows": "PA0, PA1, PA4",
    "Matrix Cols": "PB0, PB1, PB10, PA5",
}

# 2. EMG Muscle Mapping
# Muscles corresponding to dashboard plots (consistent Title Case)
EMG_CONNECTIONS: List[str] = [
    "Anterior Deltoid",
    "Lateral Deltoid",
    "Posterior Deltoid",
    "Biceps Brachii",
    "Triceps Brachii",
    "Brachioradialis",
    "Flexor Carpi Radialis",
    "Extensor Carpi Radialis",
]

# EMG Plot Labels (mapping to dashboard plots 1-8)
EMG_PLOT_LABELS: List[str] = [
    "CH 1", "CH 2", "CH 3", "CH 4", "CH 5", "CH 6", "CH 7", "CH 8"
]

# 3. Participant Configuration
PARTICIPANT_CONFIG: Dict[str, str] = {
    "ID": "P01",
    "Trial": "05",
    "Session": "1",
    "Age": "24",
    "Handedness": "Right",
}

# 4. Trial Logic Configuration
TRIAL_LOGIC_CONFIG: Dict[str, str] = {
    "Movements": "12",
    "Repetitions": "3",
    "Interval (s)": "5",
    "Rest (s)": "10",
    "Mode": "Active Sync",
}

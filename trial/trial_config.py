"""
Trial Configuration
===================

Configuration for the experimental trial, including hardware, participant, 
and trial logic parameters.
"""

from typing import List, Dict, Any

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
    "Trial": "01",
    "Session": "1",
    "Gender": "Male",
    "Age": "25",
    "Handedness": "Right",
    "Total Arm Length [cm]": "82",
    "Upper Arm Length [cm]": "38",
    "Forearm Length [cm]": "33",
    "Hand Length [cm]": "20",
    "Upper Arm Circumference [cm]": "32",
    "Fore Arm Circumference [cm]": "28",
}

# 4. Trial Logic Configuration
TRIAL_LOGIC_CONFIG: Dict[str, Any] = {
    "Movements": 10,
    "Number of Weights": 7,
}

# 5. Starting Slots (Weights to be moved onto the matrix)
# The 8 starting slots and their initial weights in kg (None if empty)
# Adjust these based on the actual physical predefined setup
STARTING_SLOTS_CONFIG: Dict[str, float | None] = {
    "Slot 1": 0.75,
    "Slot 2": 1.0,
    "Slot 3": 2.0,
    "Slot 4": 2.25,
    "Slot 5": 3.0,
    "Slot 6": 4.25,
    "Slot 7": 6.0,
    "Slot 8": None,
}

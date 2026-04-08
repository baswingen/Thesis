"""
Run Trial
=========

Wrapper script to execute a trial using the parameters defined in `trial_config.py`.
This script automatically loads the configuration and initializes the required components.
"""
import sys
import time
from pathlib import Path

# Ensure the root directory is accessible for imports
sys.path.append(str(Path(__file__).parent.parent))

from trial.setup_trial import TrialManager, _TMSI_AVAILABLE, _QT_AVAILABLE
from trial.trial_config import PARTICIPANT_CONFIG

try:
    from trial.dashboard import launch_dashboard
except ImportError:
    pass

def main():
    if not _TMSI_AVAILABLE:
        print("\n[ERROR] TMSi SDK not available. Cannot run acquisition.")
        sys.exit(1)

    # Extract configuration from trial_config.py
    participant_id = PARTICIPANT_CONFIG.get("ID", "P00")
    try:
        trial_num = int(PARTICIPANT_CONFIG.get("Trial", "0"))
    except ValueError:
        trial_num = 0

    try:
        age = int(PARTICIPANT_CONFIG.get("Age", "25"))
    except ValueError:
        age = 25

    handedness = PARTICIPANT_CONFIG.get("Handedness", "Right")
    session = PARTICIPANT_CONFIG.get("Session", "3")
    
    metadata = {
        'age': age,
        'dominant_arm': handedness,
        'measured_arm': handedness,  # Assume measured arm is the dominant one
        'session': session
    }

    print(f"=== Initializing Trial for {participant_id} - Trial {trial_num} ===")
    print(f"Config: {metadata}")

    # Initialize Trial Manager
    manager = TrialManager(
        participant_id=participant_id,
        trial_num=trial_num,
        metadata=metadata
    )
    
    manager.start()
    
    if _QT_AVAILABLE:
        print("[TRIAL] Launching Dashboard...")
        launch_dashboard(manager=manager, n_emg_channels=8)
    else:
        try:
            print("[TRIAL] Running in headless mode (No PyQt6 found). Press Ctrl+C to stop.")
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[TRIAL] Interrupted by user.")
        finally:
            manager.stop()

if __name__ == '__main__':
    main()

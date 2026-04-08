import os
import sys

# Add project root to path
sys.path.insert(0, os.getcwd())

def debug_imports():
    print(f"Python: {sys.version}")
    print(f"Prefix: {sys.prefix}")
    
    print("\n--- Importing torch ---")
    try:
        import torch
        print(f"Success! Torch version: {torch.__version__}")
    except Exception as e:
        print(f"Torch failed: {e}")

    print("\n--- Importing PyQt6.QtWidgets ---")
    try:
        from PyQt6 import QtWidgets
        print("Success! PyQt6.QtWidgets imported.")
    except Exception as e:
        print(f"PyQt6 failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    debug_imports()

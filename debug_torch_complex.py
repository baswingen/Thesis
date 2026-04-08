import os
import sys

# Add project root to path
sys.path.insert(0, os.getcwd())

def debug_import_complex():
    print(f"Python: {sys.version}")
    print(f"CWD: {os.getcwd()}")
    
    print("\n--- Importing inference.model_loader ---")
    try:
        from inference.model_loader import ModelLoader
        print("Success! ModelLoader imported.")
        
        print("\n--- Importing torch ---")
        import torch
        print(f"Success! Torch version: {torch.__version__}")
        
    except Exception as e:
        print(f"Failed with error type: {type(e)}")
        print(f"Error message: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    debug_import_complex()

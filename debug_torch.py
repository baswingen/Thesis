import os
import sys
import ctypes

def debug_import():
    print(f"Python: {sys.version}")
    print(f"Prefix: {sys.prefix}")
    
    # Identify torch lib
    torch_lib = os.path.join(sys.prefix, 'Lib', 'site-packages', 'torch', 'lib')
    sys32 = os.path.join(os.environ.get("SystemRoot", r"C:\Windows"), "System32")
    
    print(f"Candidate torch lib: {torch_lib} (exists: {os.path.isdir(torch_lib)})")
    print(f"System32: {sys32} (exists: {os.path.isdir(sys32)})")
    
    # Try Import WITHOUT any help first
    print("\n--- Attempt 1: Raw Import ---")
    try:
        import torch
        print(f"Success! Torch version: {torch.__version__}")
        return
    except Exception as e:
        print(f"Failed: {e}")
        
    # Try with add_dll_directory
    print("\n--- Attempt 2: add_dll_directory(System32) ---")
    try:
        os.add_dll_directory(sys32)
        import torch
        print(f"Success! Torch version: {torch.__version__}")
        return
    except Exception as e:
        print(f"Failed: {e}")
        
    # Try with add_dll_directory(torch_lib)
    print("\n--- Attempt 3: add_dll_directory(torch_lib) ---")
    try:
        os.add_dll_directory(torch_lib)
        import torch
        print(f"Success! Torch version: {torch.__version__}")
        return
    except Exception as e:
        print(f"Failed: {e}")

    # Manual dependency check
    print("\n--- Manual Dependency Check ---")
    try:
        k32 = ctypes.WinDLL('kernel32.dll', use_last_error=True)
        res = k32.LoadLibraryExW(os.path.join(torch_lib, 'c10.dll'), None, 0x00001100)
        if not res:
            err = ctypes.get_last_error()
            print(f"c10.dll failed to load via LoadLibraryExW. Error: {err}")
        else:
            print("c10.dll loaded successfully via LoadLibraryExW")
    except Exception as e:
        print(f"LoadLibrary test failed: {e}")

if __name__ == "__main__":
    debug_import()

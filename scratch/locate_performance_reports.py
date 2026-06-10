from pathlib import Path

run_dirs = [
    Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/ST-transformer-par-spec-cross-val"),
    Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/final_run")
]

for r in run_dirs:
    print(f"\nRun: {r.name}")
    for sub in ['all', 'emg_only', 'imu_only']:
        p = r / sub / "performance_report.txt"
        if p.exists():
            with open(p, "r") as f:
                lines = f.readlines()
            # print lines containing "Parameter" or "train_time" or "inference"
            print(f"  --- {sub} ---")
            for line in lines:
                if any(x in line for x in ["Parameter", "train_time", "inference", "Device:"]):
                    print("   ", line.strip())

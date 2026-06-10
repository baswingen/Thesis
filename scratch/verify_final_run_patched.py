import json
from pathlib import Path

p = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/final_run/run_data.json")

print("================ JSON VERIFICATION (final_run) ================")
if p.exists():
    with open(p, "r") as f:
        data = json.load(f)
    for mod in ['all', 'emg_only', 'imu_only']:
        print(f"\nModality: {mod}")
        info = data.get(mod, {}).get("model_info", {})
        print("  total_parameters:", info.get("total_parameters"))
        print("  model_size_mb_fp32:", info.get("model_size_mb_fp32"))
        print("  on_device_latency_estimate_ms:", info.get("on_device_latency_estimate_ms"))
        print("  real_time_cpu_idle_percent:", info.get("real_time_cpu_idle_percent"))
else:
    print("Consolidated JSON not found!")

print("\n================ TEXT REPORT VERIFICATION (final_run) ================")
txt_path = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/final_run/all/performance_report.txt")
if txt_path.exists():
    with open(txt_path, "r") as f:
        content = f.read()
    print("Last 15 lines of performance_report.txt:")
    print("\n".join(content.splitlines()[-12:]))
else:
    print("performance_report.txt not found!")

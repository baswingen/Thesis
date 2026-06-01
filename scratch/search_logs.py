import json
from pathlib import Path

log_path = Path("/Users/baswingen/.gemini/antigravity-ide/brain/87dd1108-dc96-4ab7-9c12-2454be97c5d2/.system_generated/logs/transcript.jsonl")

print("Searching logs...")
with open(log_path, "r") as f:
    for line in f:
        obj = json.loads(line)
        content = obj.get("content", "")
        # Search for commands or system outputs or specific keywords
        if "Laurens" in content or "explainability" in content or "retrospective" in content or "mount" in content:
            print(f"[{obj.get('type')}] status: {obj.get('status')}")
            print(content[:500])
            print("-" * 50)

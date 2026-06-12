import json
import pandas as pd
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
json_path = os.path.join(script_dir, "drive_log.json")

with open(json_path) as f:
    data = json.load(f)

rows = []

for s in data:
    rows.append({
        "time": s["time"],
        "fl_pos": s["FrontLeft"]["position"],
        "fl_current": s["FrontLeft"]["current"],
        "fr_pos": s["FrontRight"]["position"],
        "fr_current": s["FrontRight"]["current"]
        })
df = pd.DataFrame(rows)

print(df.head())
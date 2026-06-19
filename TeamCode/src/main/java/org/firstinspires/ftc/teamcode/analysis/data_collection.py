import json
import pandas as pd
import os
import plotly.express as px

script_dir = os.path.dirname(os.path.abspath(__file__))
json_path = os.path.join(script_dir, "BLACKJACK.json")

with open(json_path) as f:
    text = f.read()

chunks = text.replace("]\n[", "]|[").replace("][", "]|[").split("|")

data = []
for chunk in chunks:
    data.extend(json.loads(chunk))

rows = []

for s in data:
    rows.append({
        "timestamp": s.get("timestamp"),
        "run time": s.get("run time"),
        #"fl_pos": s["FrontLeft"]["position"],
        #"fr_pos": s["FrontRight"]["position"],
        #"bl_pos": s["BackLeft"]["position"],
        #"br_pos": s["BackRight"]["position"],
        "fl_power": s["FrontLeft"]["power"],
        "fr_power": s["FrontRight"]["power"],
        "bl_power": s["BackLeft"]["power"],
        "br_power": s["BackRight"]["power"],
        "fl_current": s["FrontLeft"]["current"],
        "fr_current": s["FrontRight"]["current"],
        "bl_current": s["BackLeft"]["current"],
        "br_current": s["BackRight"]["current"]
        })
df = pd.DataFrame(rows)

print(df.head())

fig = px.line(
    df,
    x="run time",
    y=["fl_current", "fr_current", "bl_current", "br_current", "fl_power", "fr_power", "bl_power", "br_power"],
    title="Current and Power vs Time"
)

fig.show()
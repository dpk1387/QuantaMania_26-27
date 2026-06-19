import json
import pandas as pd
import os
import plotly.express as px # plotly is the tool for graphing the data

# find the folder containing this python script
# allows the program to locate the json data even if the project is moved
script_dir = os.path.dirname(os.path.abspath(__file__))
json_path = os.path.join(script_dir, "BLACKJACK.json") # create the full path to the JSON file

with open(json_path) as f: # open the json file and read its contents into a string
    text = f.read()

chunks = text.replace("]\n[", "]|[").replace("][", "]|[").split("|")
# since each run creates a new array into the same file, patterns like "][", which is invalid, can be created
# this replaces the boundaries with a separator, split the file into individual arrays, and process each array separately

# convert each json array into python objects and combine all samples into a single list
data = []
for chunk in chunks:
    data.extend(json.loads(chunk))

# create a list that holds one row of data for each recorded sample
rows = []

# extract the given data from each sample
# nested json data is flattened into columns so it can be analyzed using pandas
for s in data:
    rows.append({
        "timestamp": s.get("timestamp"), # current real world time
        "run time": s.get("run time"), # time since program started running
        #"fl_pos": s["FrontLeft"]["position"],
        #"fr_pos": s["FrontRight"]["position"],
        #"bl_pos": s["BackLeft"]["position"],
        #"br_pos": s["BackRight"]["position"],
        # power and current of all four wheels
        "fl_power": s["FrontLeft"]["power"],
        "fr_power": s["FrontRight"]["power"],
        "bl_power": s["BackLeft"]["power"],
        "br_power": s["BackRight"]["power"],
        "fl_current": s["FrontLeft"]["current"],
        "fr_current": s["FrontRight"]["current"],
        "bl_current": s["BackLeft"]["current"],
        "br_current": s["BackRight"]["current"]
        })
# convert list of rows into a dataframe, similar to a spreadsheet
df = pd.DataFrame(rows)

print(df.head()) # display first few rows to verify data was loaded correctly

# create a line graph plotting the wheel currents and powers on the same graph for comparison
fig = px.line(
    df,
    x="run time",
    y=["fl_current", "fr_current", "bl_current", "br_current", "fl_power", "fr_power", "bl_power", "br_power"],
    title="Current and Power vs Time"
)

fig.show() # graph the data using Plotly
#!/home/grasp/anaconda3/envs/arha_vla/bin/python
import json
import time
import matplotlib.pyplot as plt
from pathlib import Path

# must match log_path in train_stage1.yaml
LOG_PATH = Path("/home/grasp/Documents/ARHA/arha_vla/logs/train_stage1.json")

def plot_now():
    if not LOG_PATH.exists():
        print(f"Waiting for log file at: {LOG_PATH}", end='\r')
        return

    try:
        with open(LOG_PATH, "r") as f:
            data = json.load(f)
        
        steps = data.get("steps", [])
        loss = data.get("train_loss", [])

        if len(steps) < 2:
            return

        plt.clf()
        plt.plot(steps, loss, color='dodgerblue', linewidth=2, label='Train Loss')
        plt.yscale('log')
        plt.xlabel('Global Steps')
        plt.ylabel('MSE Loss (Log Scale)')
        plt.title('ARHA VLA Training Progress')
        plt.legend()
        plt.grid(True, which="both", ls="-", alpha=0.3)
        plt.draw()
        plt.pause(0.1)
    except Exception as e:
        print(f"\n[Error] Reading log: {e}")

plt.ion()
fig = plt.figure(figsize=(9, 5))

print(f"Starting live plot tracking: {LOG_PATH}")
print("Press Ctrl+C in this terminal to stop.")

try:
    while True:
        plot_now()
        time.sleep(15)
except KeyboardInterrupt:
    print("\nPlotter stopped by user.")
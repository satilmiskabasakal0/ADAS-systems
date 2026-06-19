import os
import csv
import matplotlib.pyplot as plt


def _use_compatible_backend():
    """Select an interactive Matplotlib backend that's actually available on
    this machine (macOS often ships without Tkinter), falling back to the
    non-interactive 'Agg' backend so the script still runs headless / in CI.
    Set the MPLBACKEND environment variable to force a specific backend."""
    import sys
    if os.environ.get("MPLBACKEND"):
        return  # respect an explicit user choice
    candidates = (["MacOSX"] if sys.platform == "darwin" else []) + ["QtAgg", "TkAgg", "Agg"]
    for backend in candidates:
        try:
            plt.switch_backend(backend)
            return
        except Exception:
            continue


_use_compatible_backend()
def plot_csv_log(file_path, title=None):
    iterations = []
    total_costs = []
    jerk_costs = []
    time_costs = []
    pos_costs = []

    with open(file_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            iterations.append(int(row["iteration"]))
            total_costs.append(float(row["total_cost"]))
            jerk_costs.append(float(row["jerk_cost"]))
            time_costs.append(float(row["time_cost"]))
            pos_costs.append(float(row["position_cost"]))

    plt.figure(figsize=(12, 6))
    plt.plot(iterations, total_costs, label='Total Cost', linewidth=2)
    plt.plot(iterations, jerk_costs, label='Jerk Cost', linestyle='--')
    plt.plot(iterations, time_costs, label='Time Cost', linestyle='--')
    plt.plot(iterations, pos_costs, label='Position Cost', linestyle='--')
    plt.xlabel('Iteration')
    plt.ylabel('Cost')
    plt.title(title or os.path.basename(file_path))
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_all_logs(folder="opt_logs"):
    for filename in os.listdir(folder):
        if filename.endswith(".csv"):
            file_path = os.path.join(folder, filename)
            plot_csv_log(file_path)

if __name__ == "__main__":
    plot_all_logs()

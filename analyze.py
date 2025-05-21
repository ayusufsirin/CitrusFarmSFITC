import pandas as pd
import matplotlib.pyplot as plt
import os

log_dir = os.path.expanduser("./logs")

# Get a list of all CSV files
log_files = [f for f in os.listdir(log_dir) if f.startswith('fusion_performance_') and f.endswith('.csv')]
log_files.sort() # Sort by name (which includes timestamp) to process in order

all_data = []

for filename in log_files:
    filepath = os.path.join(log_dir, filename)
    df = pd.read_csv(filepath)

    # Extract parameters from the filename or assume they are constant for this file
    # If parameters were part of the filename, you could parse them here.
    # Otherwise, they are directly in the df columns.

    # Calculate average metrics for this log file
    avg_fusion_time = df['FusionTime_ms'].mean()
    avg_total_callback_time = df['TotalCallbackTime_ms'].mean()
    ncutoff_val = df['ncutoff'].iloc[0] # Get the first value (should be constant for the file)
    threshold_val = df['threshold'].iloc[0] # Get the first value

    all_data.append({
        'ncutoff': ncutoff_val,
        'threshold': threshold_val,
        'AvgFusionTime_ms': avg_fusion_time,
        'AvgTotalCallbackTime_ms': avg_total_callback_time,
        'LogFilename': filename
    })

summary_df = pd.DataFrame(all_data)
print("Summary of Performance across different parameters:")
print(summary_df.round(2))

# Example Plot: Avg Fusion Time vs. ncutoff for different thresholds
plt.figure(figsize=(10, 6))
for threshold in summary_df['threshold'].unique():
    subset = summary_df[summary_df['threshold'] == threshold]
    plt.plot(subset['ncutoff'], subset['AvgFusionTime_ms'], marker='o', label=f'Threshold={threshold}')

plt.title('Average Fusion Time vs. ncutoff for Different Thresholds')
plt.xlabel('ncutoff')
plt.ylabel('Average Fusion Time (ms)')
plt.legend()
plt.grid(True)
plt.xticks(summary_df['ncutoff'].unique()) # Ensure x-axis ticks match your tested ncutoff values
plt.show()

# You can create many more plots:
# - Avg Fusion Time vs. threshold for different ncutoffs
# - Histograms of FusionTime_ms for a specific run
# - Visualize specific depth maps / point clouds from good/bad runs in RViz
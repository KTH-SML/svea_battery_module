import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np

# ================================
# Configuration
# ================================

# Path to the CSV file
CSV_FILE_PATH = "battery_readings.csv"

# Total battery capacity in milliampere-hours (mAh)
BATTERY_CAPACITY_MAH = 8000  # Updated from 2000 mAh to 8000 mAh

# Column names in the CSV
TIMESTAMP_COL = "Timestamp"
CURRENT_COL = "Current"  # in Amperes (A)
BUS_VOLTAGE_COL = "BusVoltage"  # in Volts (V)
SOC_SIGMOIDAL_COL = "SoC_Sigmoidal"  # Existing SoC (%)

# Voltage range for linear SoC estimation
VOLTAGE_FULL = 12.6  # Volts corresponding to 100% SoC
VOLTAGE_EMPTY = 9.0  # Volts corresponding to 0% SoC

# Opacity level for plot elements (0.0 to 1.0)
OPACITY_LEVEL = 0.6  # Adjust as needed (e.g., 0.3 for more transparency)

# Number of recent data points to consider for average current calculation
RECENT_POINTS = 100000  # Adjust based on data frequency and desired responsiveness

# ================================
# Data Loading and Preprocessing
# ================================

# Load the CSV data into a pandas DataFrame
df = pd.read_csv(CSV_FILE_PATH)

# Convert the Timestamp column to datetime objects
df[TIMESTAMP_COL] = pd.to_datetime(df[TIMESTAMP_COL])

# Sort the DataFrame by Timestamp in case it's unordered
df = df.sort_values(by=TIMESTAMP_COL).reset_index(drop=True)

# Calculate the time difference between consecutive readings in hours
df["Delta_Time_Hours"] = df[TIMESTAMP_COL].diff().dt.total_seconds() / 3600
df.at[0, "Delta_Time_Hours"] = 0  # First entry has no previous timestamp

# ================================
# SoC Calculation via Coulomb Counting
# ================================

# Convert current from Amperes to milliampere (mA) for consistency with mAh
df["Current_mA"] = df[CURRENT_COL] * 1000  # mA

# Calculate the charge consumed in each interval (mAh)
df["Charge_Consumed_mAh"] = df["Current_mA"] * df["Delta_Time_Hours"]

# Initialize Remaining Capacity based on initial SoC
initial_soc_percent = df[SOC_SIGMOIDAL_COL].iloc[0]  # e.g., 98.0%
initial_capacity_mAh = (initial_soc_percent / 100) * BATTERY_CAPACITY_MAH
df["Remaining_mAh"] = initial_capacity_mAh - df["Charge_Consumed_mAh"].cumsum()

# Ensure that Remaining Capacity does not exceed Battery Capacity or drop below 0
df["Remaining_mAh"] = df["Remaining_mAh"].clip(upper=BATTERY_CAPACITY_MAH, lower=0)

# Calculate SoC Percentage via Coulomb Counting
df["SoC_Coulomb_Counting"] = (df["Remaining_mAh"] / BATTERY_CAPACITY_MAH) * 100

# ================================
# Linear SoC Estimation Based on BusVoltage
# ================================

# Calculate SoC based on BusVoltage using linear estimation
df["SoC_Linear"] = (
    (df[BUS_VOLTAGE_COL] - VOLTAGE_EMPTY) / (VOLTAGE_FULL - VOLTAGE_EMPTY)
) * 100

# Clip the SoC_Linear to be within 0% and 100%
df["SoC_Linear"] = df["SoC_Linear"].clip(lower=0, upper=100)

# ================================
# Time Conversion for Plotting
# ================================

# Calculate elapsed time in minutes from the first timestamp
df["Elapsed_Time_Minutes"] = (
    df[TIMESTAMP_COL] - df[TIMESTAMP_COL].iloc[0]
).dt.total_seconds() / 60

# ================================
# Runtime Prediction Function
# ================================


def predict_runtime(df, recent_points=10):
    """
    Predicts the remaining runtime of the battery based on the average current
    over the most recent data points and calculates the total runtime until depletion.

    Parameters:
    - df (pd.DataFrame): The DataFrame containing battery data.
    - recent_points (int): Number of recent data points to consider.

    Returns:
    - tuple: (remaining_time_str, total_runtime_str)
    """
    # Ensure there are enough data points
    if len(df) < recent_points:
        recent_points = len(df)

    # Get the most recent data points
    recent_df = df.tail(recent_points)

    # Calculate the average current (mA)
    avg_current_mA = recent_df["Current_mA"].mean()

    # Determine the direction of current
    if avg_current_mA > 0:
        direction = "discharging"
    elif avg_current_mA < 0:
        direction = "charging"
    else:
        return (
            "Battery is neither charging nor discharging.",
            "Total runtime prediction is not available.",
        )

    # Avoid division by zero
    if avg_current_mA == 0:
        return (
            "Battery is neither charging nor discharging.",
            "Total runtime prediction is not available.",
        )

    # Get the latest SoC from Coulomb Counting
    latest_soc = df["SoC_Coulomb_Counting"].iloc[-1]

    if direction == "discharging":
        # Calculate remaining mAh
        remaining_mAh = (latest_soc / 100) * BATTERY_CAPACITY_MAH
        # Estimate runtime in hours
        runtime_hours = remaining_mAh / avg_current_mA
        runtime_hours = max(runtime_hours, 0)  # Ensure non-negative
        runtime_minutes = runtime_hours * 60
        remaining_time_str = f"Estimated runtime until fully discharged: {runtime_hours:.2f} hours ({runtime_minutes:.1f} minutes)."
    else:
        # Charging
        # Calculate mAh needed to reach 100%
        needed_mAh = BATTERY_CAPACITY_MAH - (latest_soc / 100) * BATTERY_CAPACITY_MAH
        # Average charging current (absolute value)
        avg_charge_mA = abs(avg_current_mA)
        # Estimate time to full charge in hours
        runtime_hours = needed_mAh / avg_charge_mA
        runtime_hours = max(runtime_hours, 0)  # Ensure non-negative
        runtime_minutes = runtime_hours * 60
        remaining_time_str = f"Estimated runtime until fully charged: {runtime_hours:.2f} hours ({runtime_minutes:.1f} minutes)."

    # Calculate total runtime until depletion or full charge
    # Total runtime = elapsed time + remaining runtime
    # Convert elapsed time to hours
    elapsed_time_hours = df["Elapsed_Time_Minutes"].iloc[-1] / 60
    total_runtime_hours = elapsed_time_hours + runtime_hours
    total_runtime_minutes = total_runtime_hours * 60
    total_runtime_str = f"Total runtime until predicted depletion: {total_runtime_hours:.2f} hours ({total_runtime_minutes:.1f} minutes)."

    return (remaining_time_str, total_runtime_str)


# ================================
# Plotting
# ================================

# Create a figure with two subplots: one for mAh, one for SoC Percentage
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

# -------------------------------
# First Plot: Remaining Capacity (mAh) over Time
# -------------------------------
ax1.plot(
    df["Elapsed_Time_Minutes"],
    df["Remaining_mAh"],
    marker="o",
    linestyle="-",
    markersize=2,  # Smaller markers
    markerfacecolor="blue",
    markeredgecolor="blue",
    linewidth=0.6,  # Thinner lines
    alpha=OPACITY_LEVEL,  # Lower opacity
    label="Remaining Capacity (mAh)",
)
ax1.set_title("Battery Remaining Capacity Over Time")
ax1.set_ylabel("Remaining Capacity (mAh)")
ax1.grid(True, which="both", linestyle="--", linewidth=0.5)
ax1.legend(loc="upper right", fontsize=8)

# -------------------------------
# Second Plot: SoC Percentage Comparison
# -------------------------------
# Plot Calculated SoC via Coulomb Counting
ax2.plot(
    df["Elapsed_Time_Minutes"],
    df["SoC_Coulomb_Counting"],
    marker="s",
    linestyle="-",
    markersize=2,  # Smaller markers
    markerfacecolor="green",
    markeredgecolor="green",
    linewidth=0.6,  # Thinner lines
    alpha=OPACITY_LEVEL,  # Lower opacity
    label="Calculated SoC (%)",
)

# Plot SoC_Sigmoidal from CSV
ax2.plot(
    df["Elapsed_Time_Minutes"],
    df[SOC_SIGMOIDAL_COL],
    marker="^",
    linestyle="--",
    markersize=2,  # Smaller markers
    markerfacecolor="red",
    markeredgecolor="red",
    linewidth=0.6,  # Thinner lines
    alpha=OPACITY_LEVEL,  # Lower opacity
    label="SoC_Sigmoidal (%)",
)

# Plot Linear SoC Estimation based on BusVoltage
ax2.plot(
    df["Elapsed_Time_Minutes"],
    df["SoC_Linear"],
    marker="d",
    linestyle=":",
    markersize=2,  # Smaller markers
    markerfacecolor="purple",
    markeredgecolor="purple",
    linewidth=0.6,  # Thinner lines
    alpha=OPACITY_LEVEL,  # Lower opacity
    label="SoC_Linear (%)",
)

ax2.set_title("State of Charge (SoC) Comparison Over Time")
ax2.set_xlabel("Time Elapsed (minutes)")
ax2.set_ylabel("State of Charge (%)")
ax2.grid(True, which="both", linestyle="--", linewidth=0.5)
ax2.legend(loc="upper right", fontsize=8)


# ================================
# Runtime Prediction
# ================================

remaining_time, total_runtime = predict_runtime(df, RECENT_POINTS)
print(remaining_time)
print(total_runtime)


plt.tight_layout()
plt.show()

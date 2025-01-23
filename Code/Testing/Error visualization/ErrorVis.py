import pandas as pd
import matplotlib.pyplot as plt

# Example worst-case errors (adjust as needed)
worst_bus_err = 0.004  # +0.4%
worst_curr_err = 0.005  # +0.5%

# Starting battery capacity (mAh)
initial_capacity_mAh = 8000

df = pd.read_csv("battery_readings.csv", parse_dates=["Timestamp"])
df.sort_values(by="Timestamp", inplace=True)

# Nominal power (voltage * current)
df["power_nominal"] = df["BusVoltage"] * df["Current"]

# Upper/Lower (voltage ± worst_bus_err, current ± worst_curr_err)
df["voltage_upper"] = df["BusVoltage"] * (1 + worst_bus_err)
df["current_upper"] = df["Current"] * (1 + worst_curr_err)
df["power_upper"] = df["voltage_upper"] * df["current_upper"]

df["voltage_lower"] = df["BusVoltage"] * (1 - worst_bus_err)
df["current_lower"] = df["Current"] * (1 - worst_curr_err)
df["power_lower"] = df["voltage_lower"] * df["current_lower"]

# Time difference in hours
df["delta_t_hours"] = df["Timestamp"].diff().dt.total_seconds().fillna(0) / 3600.0

# Integrate current to find capacity in mAh

# Nominal
df["mAh_used_nominal_inc"] = df["Current"] * 1000 * df["delta_t_hours"]
df["mAh_used_nominal"] = df["mAh_used_nominal_inc"].cumsum()
df["capacity_nominal"] = initial_capacity_mAh - df["mAh_used_nominal"]

# Upper
df["mAh_used_upper_inc"] = df["current_upper"] * 1000 * df["delta_t_hours"]
df["mAh_used_upper"] = df["mAh_used_upper_inc"].cumsum()
df["capacity_upper"] = initial_capacity_mAh - df["mAh_used_upper"]

# Lower
df["mAh_used_lower_inc"] = df["current_lower"] * 1000 * df["delta_t_hours"]
df["mAh_used_lower"] = df["mAh_used_lower_inc"].cumsum()
df["capacity_lower"] = initial_capacity_mAh - df["mAh_used_lower"]

# Plot instantaneous power
plt.figure(figsize=(10, 4))
plt.subplot(1, 2, 1)
plt.plot(df["Timestamp"], df["power_nominal"], label="Power Nominal")
plt.plot(df["Timestamp"], df["power_upper"], label="Power Upper Bound")
plt.plot(df["Timestamp"], df["power_lower"], label="Power Lower Bound")
plt.title("Instantaneous Power")
plt.xlabel("Time")
plt.ylabel("Power (W)")
plt.legend()

# Plot remaining capacity (mAh)
plt.subplot(1, 2, 2)
plt.plot(df["Timestamp"], df["capacity_nominal"], label="Capacity Nominal")
plt.plot(df["Timestamp"], df["capacity_upper"], label="Capacity Upper Bound")
plt.plot(df["Timestamp"], df["capacity_lower"], label="Capacity Lower Bound")
plt.title("Remaining Capacity (mAh)")
plt.xlabel("Time")
plt.ylabel("Capacity (mAh)")
plt.legend()

plt.tight_layout()
plt.show()

# --- Print final deviation from nominal ---
final_nominal = df["capacity_nominal"].iloc[-1]
final_upper = df["capacity_upper"].iloc[-1]
final_lower = df["capacity_lower"].iloc[-1]

# Positive and negative deviations from nominal
pos_deviation = final_upper - final_nominal
neg_deviation = final_nominal - final_lower

print(f"Final nominal capacity: {final_nominal:.2f} mAh")
print(f"Deviation from nominal: +{pos_deviation:.2f} / -{neg_deviation:.2f} mAh")
print(
    f"Or approximately ±{max(abs(pos_deviation), abs(neg_deviation)):.2f} mAh total range."
)

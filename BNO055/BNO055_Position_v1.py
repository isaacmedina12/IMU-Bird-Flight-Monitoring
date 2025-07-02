# BNO055 position tracking script v1.0
# Tufts University - School of Engineering 
# Department of Electrical and Computer Engineering 
# Sonkusale Research Lab
# Made by: Isaac Medina
# Date: 26 June 2025
# Purpose: This script analyses data collected from the BNO055 9-DoF IMU. 
#          Linear acceleation and quaternion data is collected at ~100 Hz from 
#          the BNO055 via ESP32. Data is exported to CSV for analysis.
# Algorithm: Simple high pass filter.

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# === 1. Load CSV ===
df = pd.read_csv("/Users/isaacmedina12/Downloads/BIRD PROJECT/IMU/BNO055_Initial_Motion_test_1.csv")  
df.columns = df.columns.str.strip()

# === 2. Time conversion ===
df["timestamp_s"] = (df["timestamp_us"] - df["timestamp_us"].iloc[0]) / 1_000_000
dt = np.diff(df["timestamp_s"], prepend=df["timestamp_s"].iloc[0])  # seconds

# === 3. Rotate acceleration to world frame ===
acc_local = df[["acc_x", "acc_y", "acc_z"]].to_numpy()
quats = df[["quat_w", "quat_x", "quat_y", "quat_z"]].to_numpy()
acc_world = []

for i in range(len(acc_local)):
    r = R.from_quat([quats[i, 1], quats[i, 2], quats[i, 3], quats[i, 0]])  # x, y, z, w
    acc_world.append(r.apply(acc_local[i]))

acc_world = np.array(acc_world)

# === 4. High-pass filter to remove constant bias (optional but helpful) ===
acc_world -= acc_world.mean(axis=0)

# === 5. Integrate to get velocity and position ===
vel_world = np.cumsum(acc_world * dt[:, None], axis=0)
pos_world = np.cumsum(vel_world * dt[:, None], axis=0)


# === 6. Plot 3D trajectory ===
fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(pos_world[:,0], pos_world[:,1], pos_world[:,2], label="Estimated Path")
ax.scatter([0], [0], [0], color='red', label="Start")
ax.set_title("3D Position Estimated from BNO055 IMU | Test 1")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.legend()
plt.tight_layout()
plt.show()

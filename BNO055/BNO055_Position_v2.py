# BNO055 position tracking script v1.0
# Tufts University - School of Engineering 
# Department of Electrical and Computer Engineering 
# Sonkusale Research Lab
# Made by: Isaac Medina
# Date: 27 June 2025
# Purpose: This script analyses data collected from the BNO055 9-DoF IMU. 
#          Linear acceleation and quaternion data is collected at ~100 Hz from 
#          the BNO055 via ESP32. Data is exported to CSV for analysis.
# Algorithm: This script is an extension of my first implementation. v2 adds 
#            zero-velocity updates (ZUPT) and Kalman filtering.

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.signal import butter, filtfilt

# --- 1. High-pass filter helper ---
def highpass(data, fs, cutoff=0.2, order=1):
    """
    Apply a Butterworth high-pass filter along axis=0 to remove low-frequency bias.
    data: np.ndarray, shape (N, 3)
    fs: sampling rate (Hz)
    cutoff: cutoff frequency (Hz)
    """
    b, a = butter(order, cutoff / (fs / 2), btype='high')
    return filtfilt(b, a, data, axis=0)

# --- 2. Load CSV & timestamps ---
df = pd.read_csv(
    "/Users/isaacmedina12/Downloads/BIRD PROJECT/IMU/BNO055_Initial_Motion_test_1.csv"
)
df.columns = df.columns.str.strip()
# convert microseconds to seconds
df["timestamp_s"] = (df["timestamp_us"] - df["timestamp_us"].iloc[0]) / 1e6
# compute time step array
dt = np.diff(df["timestamp_s"], prepend=df["timestamp_s"].iloc[0])

# --- 3. Rotate accelerations to world frame ---
acc_local = df[["acc_x", "acc_y", "acc_z"]].to_numpy()
quats = df[["quat_w", "quat_x", "quat_y", "quat_z"]].to_numpy()
acc_world = np.zeros_like(acc_local)
for i in range(len(acc_local)):
    # scipy expects [x, y, z, w]
    r = R.from_quat([quats[i,1], quats[i,2], quats[i,3], quats[i,0]])
    acc_world[i] = r.apply(acc_local[i])

# --- 4. High-pass filter ---
fs = 1.0 / np.mean(dt[1:])  # approximate sampling rate
acc_world_hp = highpass(acc_world, fs, cutoff=0.2, order=1)

# --- 5. Inspect magnitudes & threshold choice ---
acc_mag = np.linalg.norm(acc_world_hp, axis=1)
print(f"acc_mag range: {acc_mag.min():.3f} to {acc_mag.max():.3f} m/s²")
# plot histogram for threshold selection
plt.figure()
plt.hist(acc_mag, bins=100)
plt.xlabel("High-pass acc magnitude (m/s²)")
plt.ylabel("Count")
plt.title("Histogram of high-pass filtered acc magnitude")
plt.show()


# choose threshold (e.g., 0.2 m/s²)
threshold = 2.0 # some value in m/s²
zupt_mask = acc_mag < threshold
print(f"ZUPT frames: {zupt_mask.sum()} / {len(zupt_mask)} using threshold {threshold}")

# --- 6. Kalman filter + ZUPT integration ---
# state X = [position; velocity] per axis
dim = acc_world.shape[1]
X = [np.zeros(2) for _ in range(dim)] # State vector, the quantities we are estimating = velocity and position
P = [np.eye(2) for _ in range(dim)]   # Process covariance matrix, this is the uncertainty of the estimate of X, the state vector
Q = np.eye(2) * 0.01  # process noise covariance, larger Q = trust model less and respond quicker to sensorn changes
R_kalman = np.array([[0.05]])         # measurement noise covariance, how uncertain the measurement is, can be tweaked
H = np.array([[0, 1]])                # we observe velocity only during ZUPT

vel_world = np.zeros_like(acc_world)
pos_world = np.zeros_like(acc_world)

for i in range(len(acc_world_hp)):
    dti = dt[i]
    A = np.array([[1, dti], [0, 1]])        # state transition vector how position and velocity evolve over time
    B = np.array([0.5 * dti**2, dti])       # control input vector (acceleration) how acceleration affects state

    for axis in range(dim):
        u = acc_world_hp[i, axis] # control input, in this case acceleration data from the IMU

        # === Predict step ===
        X[axis] = A @ X[axis] + B * u   # predict the new state of the system based on what we know about acceleration input physics
        P[axis] = A @ P[axis] @ A.T + Q # predict the new covariance matrix

        # === Update step (if ZUPT condition met) ===
        if zupt_mask[i]:
            z = np.array([[0.0]])                   # we expect velocity = 0, what the sensor tells us based on the threshold v = 0 during ZUPT
            y = z - H @ X[axis]                     # innovation (measurement residual) = the difference between what we measured (z) and what we expect H @ X
            S = H @ P[axis] @ H.T + R_kalman        # innovation covariance: total uncertainty in the innovation y
            K = P[axis] @ H.T @ np.linalg.inv(S)    # Kalman gain, says how much we should trust the measurement vs the model
            X[axis] = X[axis] + (K @ y).flatten()   # current predicted state = current predicted state + correction amount to apply to the state
                                                    # Adjust the position and velocity estimates to bring them closer to the measurment
            P[axis] = (np.eye(2) - K @ H) @ P[axis] # Update the uncertainty P in the state after incorperating the measurement

        # Store results
        pos_world[i, axis] = X[axis][0]
        vel_world[i, axis] = X[axis][1]

print(f"Final ZUPT count: {zupt_mask.sum()} out of {len(zupt_mask)}")

# --- 7. 3D trajectory plot ---
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(pos_world[:,0], pos_world[:,1], pos_world[:,2], label='Estimated Path')
ax.scatter(0, 0, 0, color='red', label='Start')
end_x, end_y, end_z = pos_world[-1]
ax.scatter(end_x, end_y, end_z, color='green', label='End')
ax.set_title('Improved Tracking from BNO055 IMU with Kalman Filter and ZUPT')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.legend()
plt.tight_layout()
plt.show() 
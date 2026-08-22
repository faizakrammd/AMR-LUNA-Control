import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load your CSV
df = pd.read_csv("sqrewpid.csv")

# Extract columns
time = df['Time'].values
yaw = np.deg2rad(df['Yaw'].values)  # degrees → radians

enc_left = df['Encoder Left'].values
enc_right = df['Encoder Right'].values

# ----------------------------
# ⚙️ SET THESE PROPERLY
# ----------------------------
ticks_per_rev = 360      # change if needed
wheel_radius = 0.03      # meters
wheel_base = 0.15        # distance between wheels
# ----------------------------

def ticks_to_dist(ticks):
    return 2 * np.pi * wheel_radius * (ticks / ticks_per_rev)

# Change in encoder values
dL = np.diff(enc_left)
dR = np.diff(enc_right)

# Convert to distance
dL = ticks_to_dist(dL)
dR = ticks_to_dist(dR)

# Average forward movement
d = (dL + dR) / 2

# Initialize trajectory
x = [0]
y = [0]

for i in range(len(d)):
    theta = yaw[i]  # using IMU yaw directly
    x.append(x[-1] + d[i] * np.cos(theta))
    y.append(y[-1] + d[i] * np.sin(theta))

# Plot
plt.figure()
plt.plot(x, y)
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Robot Trajectory")
plt.axis('equal')
plt.grid()
plt.show()
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon

# ----------------------------
# LOAD DATA
# ----------------------------
df = pd.read_csv("crpid1.csv")

time = df['Time'].values
yaw = np.deg2rad(df['Yaw'].values)

enc_left = df['Encoder Left'].values
enc_right = df['Encoder Right'].values

# ----------------------------
# PARAMETERS
# ----------------------------
ticks_per_rev = 360
wheel_radius = 0.03
wheel_base = 0.15

def ticks_to_dist(ticks):
    return 2 * np.pi * wheel_radius * (ticks / ticks_per_rev)

# Encoder differences
dL = np.diff(enc_left)
dR = np.diff(enc_right)

dL = ticks_to_dist(dL)
dR = ticks_to_dist(dR)

d = (dL + dR) / 2

# ----------------------------
# TRAJECTORY
# ----------------------------
x = [0]
y = [0]

for i in range(len(d)):
    theta = yaw[i]
    x.append(x[-1] + d[i] * np.cos(theta))
    y.append(y[-1] + d[i] * np.sin(theta))

# ----------------------------
# FIGURE
# ----------------------------
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.grid()

line, = ax.plot([], [], lw=2)

# Robot body (rectangle)
robot = Polygon([[0,0]], closed=True, color='red')

# Heading arrow
arrow = ax.quiver(0, 0, 0, 0, scale=5)

ax.add_patch(robot)

# Limits
margin = 0.1
ax.set_xlim(min(x)-margin, max(x)+margin)
ax.set_ylim(min(y)-margin, max(y)+margin)

# ----------------------------
# ROBOT SHAPE FUNCTION
# ----------------------------
def get_robot_shape(xc, yc, theta):
    L = 0.08   # length
    W = 0.05   # width

    # rectangle corners (centered at origin)
    corners = np.array([
        [ L/2,  W/2],
        [ L/2, -W/2],
        [-L/2, -W/2],
        [-L/2,  W/2]
    ])

    # rotation matrix
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

    rotated = corners @ R.T
    translated = rotated + np.array([xc, yc])

    return translated

# ----------------------------
# UPDATE FUNCTION
# ----------------------------
def update(frame):
    # Stop at last frame (no repeat)
    if frame >= len(x):
        ani.event_source.stop()
        return

    # Path
    line.set_data(x[:frame], y[:frame])

    # Robot body
    shape = get_robot_shape(x[frame], y[frame], yaw[frame])
    robot.set_xy(shape)

    # Heading arrow
    arrow_length = 0.4   # try 0.05 → small, 0.2 → big
    dx = arrow_length * np.cos(yaw[frame])
    dy = arrow_length * np.sin(yaw[frame])
    arrow.set_offsets([x[frame], y[frame]])
    arrow.set_UVC(dx, dy)

    return line, robot, arrow

# ----------------------------
# ANIMATION
# ----------------------------
ani = FuncAnimation(
    fig,
    update,
    frames=len(x)+1,
    interval=0,
    repeat=False   # 👈 IMPORTANT
)

plt.title("LUNA Robot Trajectory")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.show()
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle

# ==========================================
# ENVIRONMENT
# ==========================================

goal = np.array([8.0, 0.0])

obstacle = np.array([4.0, 0.0])
obs_radius = 0.8

# ==========================================
# ROBOT PARAMETERS
# ==========================================

x = 0.0
y = 0.0
theta = 0.0

wheel_base = 0.25

dt = 0.05

trail_x = []
trail_y = []

state = "GO_TO_GOAL"

# Simulated ToF
sensor_range = 2.0

# ==========================================
# HELPER FUNCTIONS
# ==========================================

def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

# ==========================================
# PLOT
# ==========================================

fig, ax = plt.subplots(figsize=(10, 6))

ax.set_xlim(-1, 10)
ax.set_ylim(-4, 4)
ax.set_aspect('equal')
ax.grid(True)

ax.set_title("LUNA: Differential Drive Robot Obstacle Avoidance")

# Obstacle
ax.add_patch(
    Circle(
        obstacle,
        obs_radius,
        color='red',
        alpha=0.5,
        label="Obstacle"
    )
)

# Goal
ax.plot(
    goal[0],
    goal[1],
    'g*',
    markersize=18,
    label="Goal"
)

# Robot
robot_plot, = ax.plot([], [], 'bo', ms=12)

# Heading direction
heading_line, = ax.plot([], [], lw=3)

# ToF Beam
sensor_line, = ax.plot([], [], '--')

# Path
path_plot, = ax.plot([], [], lw=2)

# Status text
text_dashboard = ax.text(
    0.02,
    0.98,
    "",
    transform=ax.transAxes,
    verticalalignment='top',
    fontsize=10,
    bbox=dict(facecolor='white', alpha=0.8)
)

# ==========================================
# ANIMATION UPDATE
# ==========================================

def update(frame):

    global x, y, theta, state

    pos = np.array([x, y])

    # --------------------------------------
    # GOAL CHECK
    # --------------------------------------

    goal_distance = np.linalg.norm(goal - pos)

    if goal_distance < 0.20:

        robot_plot.set_data([x], [y])

        hx = x + 0.4*np.cos(theta)
        hy = y + 0.4*np.sin(theta)

        heading_line.set_data(
            [x, hx],
            [y, hy]
        )

        path_plot.set_data(
            trail_x,
            trail_y
        )

        text_dashboard.set_text(
            "STATE : GOAL REACHED\n"
            f"X : {x:.2f} m\n"
            f"Y : {y:.2f} m\n"
            f"Heading : {np.degrees(theta):.1f}°"
        )

        return (
            robot_plot,
            heading_line,
            sensor_line,
            path_plot,
            text_dashboard
        )

    # --------------------------------------
    # TOF DISTANCE
    # --------------------------------------

    center_dist = np.linalg.norm(pos - obstacle)

    tof_distance = center_dist - obs_radius

    if tof_distance < sensor_range:
        measured_tof = max(tof_distance, 0)
    else:
        measured_tof = np.nan

    # --------------------------------------
    # STATE MACHINE
    # --------------------------------------

    if center_dist < 1.5:
        state = "AVOID_OBSTACLE"

    elif x > obstacle[0] + 1.2:
        state = "GO_TO_GOAL"

    # --------------------------------------
    # CONTROLLER
    # --------------------------------------

    if state == "GO_TO_GOAL":

        target_angle = np.arctan2(
            goal[1] - y,
            goal[0] - x
        )

        error = wrap(target_angle - theta)

        v = 0.5
        w = 2.0 * error

    else:

        dx = x - obstacle[0]
        dy = y - obstacle[1]

        tangent_angle = (
            np.arctan2(dy, dx)
            + np.pi/2
        )

        error = wrap(
            tangent_angle - theta
        )

        v = 0.4
        w = 3.0 * error

    # --------------------------------------
    # DIFFERENTIAL DRIVE KINEMATICS
    # --------------------------------------

    theta += w * dt

    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt

    trail_x.append(x)
    trail_y.append(y)

    # --------------------------------------
    # ROBOT VISUALIZATION
    # --------------------------------------

    robot_plot.set_data([x], [y])

    hx = x + 0.4*np.cos(theta)
    hy = y + 0.4*np.sin(theta)

    heading_line.set_data(
        [x, hx],
        [y, hy]
    )

    # --------------------------------------
    # TOF BEAM
    # --------------------------------------

    beam_length = (
        measured_tof
        if not np.isnan(measured_tof)
        else sensor_range
    )

    sx = x + beam_length*np.cos(theta)
    sy = y + beam_length*np.sin(theta)

    sensor_line.set_data(
        [x, sx],
        [y, sy]
    )

    # --------------------------------------
    # PATH
    # --------------------------------------

    path_plot.set_data(
        trail_x,
        trail_y
    )

    # --------------------------------------
    # DASHBOARD
    # --------------------------------------

    if np.isnan(measured_tof):
        tof_text = "No Target"
    else:
        tof_text = f"{measured_tof:.2f} m"

    text_dashboard.set_text(
        f"STATE : {state}\n"
        f"ToF : {tof_text}\n"
        f"Goal Dist : {goal_distance:.2f} m\n"
        f"X : {x:.2f} m\n"
        f"Y : {y:.2f} m\n"
        f"Heading : {np.degrees(theta):.1f}°"
    )

    return (
        robot_plot,
        heading_line,
        sensor_line,
        path_plot,
        text_dashboard
    )

# ==========================================
# RUN ANIMATION
# ==========================================

ani = FuncAnimation(
    fig,
    update,
    interval=30,
    blit=True
)

plt.legend()
plt.show()
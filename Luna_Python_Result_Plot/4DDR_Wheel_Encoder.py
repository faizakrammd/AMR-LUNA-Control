import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ==========================================
# DIFFERENTIAL DRIVE ODOMETRY SIMULATION
# Straight Line -> Circle
# Ground Truth vs Encoder Odometry
# ==========================================

# Robot Parameters
wheel_radius = 0.05      # m
wheel_base = 0.25        # m
ticks_per_rev = 360

dt = 0.05

# Encoder Scale Errors
left_scale = 0.998
right_scale = 1.002

# ==========================================
# GROUND TRUTH
# ==========================================

x_gt = 0
y_gt = 0
theta_gt = 0

# ==========================================
# ODOMETRY
# ==========================================

x_odom = 0
y_odom = 0
theta_odom = 0

# ==========================================
# ENCODERS
# ==========================================

left_ticks_total = 0
right_ticks_total = 0

# ==========================================
# LOGS
# ==========================================

gt_x_log = []
gt_y_log = []

odom_x_log = []
odom_y_log = []

error_log = []
time_log = []

# ==========================================
# FIGURE
# ==========================================

fig = plt.figure(figsize=(16,7))

ax_world = plt.subplot(1,2,1)
ax_error = plt.subplot(1,2,2)

ax_world.set_xlim(-2,8)
ax_world.set_ylim(-4,4)
ax_world.set_aspect('equal')
ax_world.grid(True)

ax_world.set_title(
    "Differential Drive Odometry"
)

ax_error.set_title(
    "Position Error vs Time"
)

ax_error.grid(True)

# Ground Truth
gt_path, = ax_world.plot(
    [],
    [],
    'g',
    linewidth=3,
    label='Ground Truth'
)

# Odometry
odom_path, = ax_world.plot(
    [],
    [],
    'r--',
    linewidth=3,
    label='Odometry'
)

gt_robot, = ax_world.plot(
    [],
    [],
    'go',
    markersize=10
)

odom_robot, = ax_world.plot(
    [],
    [],
    'ro',
    markersize=10
)

gt_heading, = ax_world.plot([],[],'g')
odom_heading, = ax_world.plot([],[],'r')

error_plot, = ax_error.plot(
    [],
    [],
    linewidth=2
)

dashboard = ax_world.text(
    0.02,
    0.98,
    "",
    transform=ax_world.transAxes,
    verticalalignment='top',
    bbox=dict(facecolor='white')
)

ax_world.legend()

# ==========================================
# UPDATE
# ==========================================

def update(frame):

    global x_gt,y_gt,theta_gt
    global x_odom,y_odom,theta_odom
    global left_ticks_total,right_ticks_total

    # =====================================
    # MOTION PROFILE
    # =====================================

    if frame < 250:

        # Straight Line

        v = 0.4
        w = 0

        phase = "STRAIGHT"

    else:

        # Circle

        v = 0.4
        w = 0.45

        phase = "CIRCLE"

    # =====================================
    # GROUND TRUTH
    # =====================================

    theta_gt += w*dt

    x_gt += v*np.cos(theta_gt)*dt
    y_gt += v*np.sin(theta_gt)*dt

    # =====================================
    # WHEEL VELOCITIES
    # =====================================

    v_left = v - (wheel_base/2)*w
    v_right = v + (wheel_base/2)*w

    d_left = v_left*dt
    d_right = v_right*dt

    # =====================================
    # ENCODER TICKS
    # =====================================

    ticks_left = (
        d_left /
        (2*np.pi*wheel_radius)
    ) * ticks_per_rev

    ticks_right = (
        d_right /
        (2*np.pi*wheel_radius)
    ) * ticks_per_rev

    # Realistic wheel mismatch

    ticks_left *= left_scale
    ticks_right *= right_scale

    # Small noise

    ticks_left += np.random.normal(0,0.02)
    ticks_right += np.random.normal(0,0.02)

    left_ticks_total += ticks_left
    right_ticks_total += ticks_right

    # =====================================
    # ODOMETRY
    # =====================================

    d_left_est = (
        ticks_left /
        ticks_per_rev
    ) * 2*np.pi*wheel_radius

    d_right_est = (
        ticks_right /
        ticks_per_rev
    ) * 2*np.pi*wheel_radius

    d_center = (
        d_left_est +
        d_right_est
    ) / 2

    d_theta = (
        d_right_est -
        d_left_est
    ) / wheel_base

    theta_odom += d_theta

    x_odom += d_center*np.cos(theta_odom)
    y_odom += d_center*np.sin(theta_odom)

    # =====================================
    # LOGGING
    # =====================================

    gt_x_log.append(x_gt)
    gt_y_log.append(y_gt)

    odom_x_log.append(x_odom)
    odom_y_log.append(y_odom)

    error = np.sqrt(
        (x_gt-x_odom)**2 +
        (y_gt-y_odom)**2
    )

    error_log.append(error)
    time_log.append(frame*dt)

    # =====================================
    # DRAW PATHS
    # =====================================

    gt_path.set_data(
        gt_x_log,
        gt_y_log
    )

    odom_path.set_data(
        odom_x_log,
        odom_y_log
    )

    # =====================================
    # DRAW ROBOTS
    # =====================================

    gt_robot.set_data([x_gt],[y_gt])
    odom_robot.set_data([x_odom],[y_odom])

    gt_hx = x_gt + 0.3*np.cos(theta_gt)
    gt_hy = y_gt + 0.3*np.sin(theta_gt)

    od_hx = x_odom + 0.3*np.cos(theta_odom)
    od_hy = y_odom + 0.3*np.sin(theta_odom)

    gt_heading.set_data(
        [x_gt,gt_hx],
        [y_gt,gt_hy]
    )

    odom_heading.set_data(
        [x_odom,od_hx],
        [y_odom,od_hy]
    )

    # =====================================
    # ERROR PLOT
    # =====================================

    error_plot.set_data(
        time_log,
        error_log
    )

    ax_error.set_xlim(
        0,
        max(20,time_log[-1]+1)
    )

    ax_error.set_ylim(
        0,
        max(0.5,max(error_log)+0.1)
    )

    # =====================================
    # DASHBOARD
    # =====================================

    dashboard.set_text(
        f"PHASE: {phase}\n\n"
        f"LEFT TICKS : {int(left_ticks_total)}\n"
        f"RIGHT TICKS: {int(right_ticks_total)}\n\n"
        f"GT X : {x_gt:.2f}\n"
        f"GT Y : {y_gt:.2f}\n"
        f"GT θ : {np.degrees(theta_gt):.1f}°\n\n"
        f"ODOM X : {x_odom:.2f}\n"
        f"ODOM Y : {y_odom:.2f}\n"
        f"ODOM θ : {np.degrees(theta_odom):.1f}°\n\n"
        f"ERROR : {error:.3f} m"
    )

    return (
        gt_path,
        odom_path,
        gt_robot,
        odom_robot,
        gt_heading,
        odom_heading,
        error_plot,
        dashboard
    )

# ==========================================
# RUN
# ==========================================

ani = FuncAnimation(
    fig,
    update,
    interval=20,
    blit=True,
    cache_frame_data=False
)

plt.tight_layout()
plt.show()
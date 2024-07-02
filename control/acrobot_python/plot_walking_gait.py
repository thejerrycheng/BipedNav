import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parameters
T = 2  # Duration of the gait cycle in seconds
thigh_length = 0.5
calf_length = 0.5

# Constraints for hip and knee joints
theta_initial = 0
theta_final = 0.5
velocity_initial = 0
velocity_final = 1
acceleration_initial = 0
acceleration_final = 0

# Time array
t = np.linspace(0, T, 1000)

# Set up the system of equations to solve for the coefficients
A = np.array([
    [0**3, 0**2, 0, 1],
    [T**3, T**2, T, 1],
    [3*0**2, 2*0, 1, 0],
    [3*T**2, 2*T, 1, 0]
])

b_hip = np.array([theta_initial, theta_final, velocity_initial, velocity_final])
b_knee = np.array([theta_initial, theta_final, velocity_initial, velocity_final])

# Solve for the coefficients
coeff_hip = np.linalg.solve(A, b_hip)
coeff_knee = np.linalg.solve(A, b_knee)

# Compute the joint angles using the polynomial coefficients
theta_hip = np.polyval(coeff_hip, t)
theta_knee = np.polyval(coeff_knee, t)

# Forward kinematics to compute foot positions
def forward_kinematics(theta_hip, theta_knee):
    x_hip = np.zeros_like(theta_hip)
    y_hip = thigh_length * np.cos(theta_hip)
    x_knee = x_hip + thigh_length * np.sin(theta_hip)
    y_knee = y_hip - calf_length * np.cos(theta_knee)
    return x_knee, y_knee

x_swing_foot, y_swing_foot = forward_kinematics(theta_hip, theta_knee)

# Stance foot is assumed to be at the origin
x_stance_foot = np.zeros_like(t)
y_stance_foot = np.zeros_like(t)

# Create the figure and axis
fig, ax = plt.subplots(figsize=(8, 6))

# Initialize the lines for the stance and swing foot
line_stance, = ax.plot([], [], 'ro', label='Stance Foot')
line_swing, = ax.plot([], [], 'b-', label='Swing Foot')

# Set up the plot limits and labels
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_title('Foot Trajectories')
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.legend()
ax.grid(True)

# Initialize the plot
def init():
    line_stance.set_data([], [])
    line_swing.set_data([], [])
    return line_stance, line_swing

# Update function for the animation
def update(frame):
    line_stance.set_data(x_stance_foot[frame], y_stance_foot[frame])
    line_swing.set_data(x_swing_foot[:frame], y_swing_foot[:frame])
    return line_stance, line_swing

# Create the animation
ani = FuncAnimation(fig, update, frames=len(t), init_func=init, blit=True, interval=20)

# Display the animation
plt.show()

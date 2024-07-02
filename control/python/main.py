import numpy as np
import sympy as sp
from scipy.integrate import solve_ivp
import pybullet as p
import pybullet_data
import time

# Initialize symbolic variables
l, lc, Iz, m, g = sp.symbols('l lc Iz m g', real=True)
t, q1, q2, x1, x2, q1dot, q2dot, x1dot, x2dot, tau = sp.symbols('t q1 q2 x1 x2 q1dot q2dot x1dot x2dot tau', real=True)

q = sp.Matrix([q1, q2])
x = sp.Matrix([x1, x2])
qbar = sp.Matrix([q1, q2, x1, x2])
qdot = sp.Matrix([q1dot, q2dot])
xdot = sp.Matrix([x1dot, x2dot])
qbardot = sp.Matrix([q1dot, q2dot, x1dot, x2dot])

# Define centers of mass of two links
rc1 = x + lc * sp.Matrix([sp.cos(q1), sp.sin(q1)])
rc2 = x + l * sp.Matrix([sp.cos(q1), sp.sin(q1)]) + lc * sp.Matrix([sp.cos(q1 + q2), sp.sin(q1 + q2)])

# Compute time derivatives of centers of mass
rc1dot = rc1.jacobian(qbar) * qbardot
rc2dot = rc2.jacobian(qbar) * qbardot

# Define the total kinetic energy of the robot
K = 1/2 * m * (rc1dot.T * rc1dot + rc2dot.T * rc2dot)[0] + 1/2 * Iz * (q1dot**2 + (q1dot + q2dot)**2)
K = sp.simplify(K)

# Extract the square symmetric matrix of the kinetic energy
Dbar = sp.Matrix(sp.hessian(K, qbardot))

# Extract the matrix of the kinetic energy of the pinned robot
D = Dbar[:2, :2]

# Define the potential energy of the pinned robot
P = m * g * (lc * sp.sin(q1) + l * sp.sin(q1) + lc * sp.sin(q1 + q2))

# Input matrix of pinned robot
B = sp.Matrix([0, 1])

# Computation of matrix C(q,qdot) of pinned robot
C = sp.zeros(2, 2)
for i in range(2):
    for j in range(2):
        for k in range(2):
            C[k, j] += 1/2 * (sp.diff(D[k, j], q[i]) + sp.diff(D[k, i], q[j]) - sp.diff(D[i, j], q[k])) * qdot[i]

# Computation of gradient of the potential for the pinned robot
G = P.jacobian(q).T

# Computation of qddot (pinned robot)
qddot = sp.simplify(D.inv() * (-C * qdot - G + B * tau))

# Convert to numerical functions
l_val, lc_val, m_val, Iz_val, g_val, tau_val = 1, 0.5, 1, 1/12, 9.81, 0
params = {l: l_val, lc: lc_val, m: m_val, Iz: Iz_val, g: g_val, tau: tau_val}

qddot_num = sp.lambdify([q, qdot], qddot.subs(params), 'numpy')

# Define ODE system
def acrobot_ode(t, y):
    q = y[:2]
    qdot = y[2:]
    qddot = qddot_num(q, qdot).flatten()
    return np.hstack((qdot, qddot))

# Simulation parameters
q0 = np.array([np.pi/2 - np.pi/6, -np.pi/6])
qdot0 = np.array([2, -10])
y0 = np.hstack((q0, qdot0))
t_span = [0, 10]
dt = 1/60

# Perform numerical integration
sol = solve_ivp(acrobot_ode, t_span, y0, method='RK45', t_eval=np.arange(t_span[0], t_span[1], dt))

# Extract the results
t_vals = sol.t
q_vals = sol.y[:2].T

# Initialize PyBullet
print("Initializing PyBullet...")
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load ground plane and acrobot URDF
print("Loading URDF for plane...")
p.loadURDF("plane.urdf")
print("Loading URDF for acrobot...")
acrobot_id = p.loadURDF("acrobot.urdf", [0, 0, 0], useFixedBase=True)
dt = 10

time.sleep(dt)

# # Set initial joint states
# p.resetJointState(acrobot_id, 0, q0[0], qdot0[0])
# p.resetJointState(acrobot_id, 1, q0[1], qdot0[1])

# Run simulation
# for i in range(len(t_vals)):
#     q1, q2 = q_vals[i]
#     p.resetJointState(acrobot_id, 0, q1, 0)
#     p.resetJointState(acrobot_id, 1, q2, 0)
#     p.stepSimulation()
#     time.sleep(dt)

# Disconnect from PyBullet
# p.disconnect()

import pybullet as p
import pybullet_data
import time
import math
import numpy as np
from sympy import symbols, diff, Poly, lambdify
from numpy.linalg import inv

import sympy as sp

################################################################################################################################################
################################################ Modeling ################################################
################################################################################################################################################
def calculate_torque(x, data):
    q1, q2, q1dot, q2dot = x
    q1_plus = data['q1_plus']
    q_tilde_1 = data['q_tilde_1']
    
    theta = (q1_plus - q1) / q_tilde_1
    thetadot = -q1dot / q_tilde_1

    phi = data['phi_fun'](theta)
    phiprime = data['phiprime_fun'](theta)
    phipprime = data['phipprime_fun'](theta)

    # Define the output y and its derivatives
    y = q2 - phi
    ydot = q2dot - phiprime * thetadot

    # Compute Jacobian H and its derivative dH_dt
    H = np.array(data['Hfun'](q1, q2, q1_plus, q_tilde_1)).astype(np.float64).reshape(1, -1)  # H evaluated at the current state
    dH_dt = np.array(data['dH_dt_fun'](q1, q2, q1dot, q2dot, q1_plus, q_tilde_1)).astype(np.float64)  # dH_dt evaluated at the current state

    # Control inputs
    D_mat = np.array(data['D'](q1, q2, data['l'], data['lc'], data['m'], data['Iz'])).astype(np.float64)
    C_mat = np.array(data['C'](q1, q2, q1dot, q2dot, data['l'], data['lc'], data['m'], data['Iz'])).astype(np.float64)
    G_vec = np.array(data['G'](q1, q2, data['l'], data['lc'], data['m'], data['g'])).astype(np.float64).reshape(-1, 1)
    B_mat = np.array(data['B']).astype(np.float64).reshape(-1, 1)
    qdot = np.array([q1dot, q2dot]).reshape(-1, 1)
    y = np.array([y]).reshape(-1, 1)

    # Debug print statements to check dimensions
    # print("H shape:", H.shape)
    # print("dH_dt shape:", dH_dt.shape)
    # print("D_mat shape:", D_mat.shape)
    # print("C_mat shape:", C_mat.shape)
    # print("G_vec shape:", G_vec.shape)
    # print("B_mat shape:", B_mat.shape)
    # print("qdot shape:", qdot.shape)
    # print("y shape:", y.shape)

    tau = (1/(H @ inv(D_mat) @ B_mat)) * (H @ inv(D_mat) @ (C_mat@qdot + G_vec) - Kp*np.sin(y) - Kd*(H@qdot) - Kdd*(dH_dt@qdot))  # Solve for control input tau
    # print("The calculated torque is ", tau)
    return tau

    
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
G = sp.Matrix([P]).jacobian(q).T

# Computation of qddot (pinned robot)
qddot = sp.simplify(D.inv() * (-C * qdot - G + B * tau))

# Convert to numerical functions
l_val, lc_val, m_val, Iz_val, g_val, tau_val = 1, 0.5, 1, 1/12, 9.81, 0
params = {l: l_val, lc: lc_val, m: m_val, Iz: Iz_val, g: g_val, tau: tau_val}

D_func = sp.lambdify((q1, q2, l, lc, m, Iz), D)
C_func = sp.lambdify((q1, q2, q1dot, q2dot, l, lc, m, Iz), C)
G_func = sp.lambdify((q1, q2, l, lc, m, g), G)
B_func = sp.lambdify((), B)
qddot_func = sp.lambdify((q1, q2, q1dot, q2dot, tau, l, lc, m, Iz, g), qddot)

# Example usage of numerical functions
q1_val, q2_val, q1dot_val, q2dot_val = 0.1, 0.2, 0.3, 0.4
D_num = D_func(q1_val, q2_val, l_val, lc_val, m_val, Iz_val)
C_num = C_func(q1_val, q2_val, q1dot_val, q2dot_val, l_val, lc_val, m_val, Iz_val)
G_num = G_func(q1_val, q2_val, l_val, lc_val, m_val, g_val)
B_num = B_func()
qddot_num = qddot_func(q1_val, q2_val, q1dot_val, q2dot_val, tau_val, l_val, lc_val, m_val, Iz_val, g_val)

print("D matrix:", D_num)
print("C matrix:", C_num)
print("G vector:", G_num)
print("B matrix:", B_num)
print("qddot:", qddot_num)

# Define variables
beta = np.radians(30)  # Example value for beta in radians
psi = np.radians(10)   # Example value for psi in radians

# Define q_minus and q_plus
q_minus = np.array([(np.pi - beta) / 2 + psi, np.pi + beta])
q_plus = np.array([(np.pi + beta) / 2 + psi, np.pi - beta])

q1_plus = q_plus[0]
q2_plus = q_plus[1]
q1_minus = q_minus[0]
q2_minus = q_minus[1]

# Define q_tilde
q_tilde_1 = q1_plus - q1_minus
q_tilde_2 = q2_plus - q2_minus

# Print results
print(f"q1_plus: {q1_plus}")
print(f"q2_plus: {q2_plus}")
print(f"q1_minus: {q1_minus}")
print(f"q2_minus: {q2_minus}")
print(f"q_tilde_1: {q_tilde_1}")
print(f"q_tilde_2: {q_tilde_2}")

################################################################################################################################################
################################################ VHC Curve ################################################################################################
################################################################################################################################################
import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import sympy as sp
from numpy.linalg import inv

# Define the VHC polynomial coefficients
a = [13.3225, -35.9442, 30.1847, -7.3543, 0.4246, 2.8250]

# Define the function phi and its derivatives using numpy's polyval
phi = lambda theta: np.polyval(a, theta)
phiprime = lambda theta: np.polyval(np.polyder(a), theta)
phipprime = lambda theta: np.polyval(np.polyder(np.polyder(a)), theta)

# Define sigma and its derivatives
sigma = lambda theta: [q1_plus - theta * q_tilde_1, phi(theta)]
sigmaprime = lambda theta: [-q_tilde_1, phiprime(theta)]
sigmapprime = lambda theta: [0, phipprime(theta)]

# Symbolic conversion
theta, thetadot = sp.symbols('theta thetadot')
q1_plus_sym, q_tilde_1_sym = sp.symbols('q1_plus q_tilde_1')
q1, q2 = sp.symbols('q1 q2')
q = [q1, q2]
qdot = sp.symbols('q1dot q2dot')
a_sym = sp.Poly(a[::-1], theta)  # Convert to sympy polynomial
phi_sym = a_sym.as_expr()  # Convert to symbolic expression
phiprime_sym = sp.diff(phi_sym, theta)  # First derivative
phipprime_sym = sp.diff(phiprime_sym, theta)  # Second derivative

# Define sigma and its derivatives symbolically
sigma_sym = [q1_plus_sym - theta * q_tilde_1_sym, phi_sym]
sigmaprime_sym = [-q_tilde_1_sym, phiprime_sym]
sigmapprime_sym = [0, phipprime_sym]

# Control input calculation
# Error term: Substitute theta in with an expression with q1
y = q2 - phi_sym.subs(theta, (q1_plus_sym - q1) / q_tilde_1_sym)

# Gradient (Jacobian of a scalar function)
H = [sp.diff(y, q_i) for q_i in q]

# Define the time derivative of Jacobian
dHdt = [[sp.diff(H[i], q[j]) * qdot[j] for j in range(len(q))] for i in range(len(H))]

# Convert to functions
H_fun = sp.lambdify([q1, q2, q1_plus_sym, q_tilde_1_sym], H)
dH_dt_fun = sp.lambdify([q1, q2, q1dot, q2dot, q1_plus_sym, q_tilde_1_sym], dHdt)

# Controller gains and system parameters
Kp = 100
Kd = 20
Kdd = 10

def main():
    # Connect to PyBullet simulator (GUI mode)
    physicsClient = p.connect(p.GUI)

    # Optionally add search path for PyBullet to load URDF files
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load the ground plane
    plane_id = p.loadURDF("plane.urdf")

    # Load the URDF file
    urdf_path = "acrobot.urdf"  # Replace with your URDF file path
    robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 1.0], useFixedBase=False)  # Adjust the base position to be above the ground

    # Optionally, set gravity (for simulation purposes)
    p.setGravity(0, 0, -9.81)

    # Set collision filters to prevent link collisions
    # We will assume that the URDF links are indexed in the order they are defined
    link1_index = 0  # First link
    link2_index = 1  # Second link

    # Joint indices based on URDF structure
    joint1_index = 1  # Joint1
    joint2_index = 0  # Joint2 (floating joint, not controlled)

    # Set initial joint angles
    initial_angle_link1 = math.pi / 2 + math.radians(30) / 2  # pi/2 + beta/2
    initial_angle_joint1 = math.radians(30)  # 30 degrees

    # Set the initial joint states
    p.resetJointState(robot_id, joint2_index, initial_angle_link1)  # This joint is floating
    p.resetJointState(robot_id, joint1_index, initial_angle_joint1)

    # Control loop
    start_time = time.time()
    while True:
        # Calculate the elapsed time
        elapsed_time = time.time() - start_time

        # Get joint states
        joint1_states = p.getJointState(robot_id, joint1_index)
        joint2_states = p.getJointState(robot_id, joint2_index)
        q1, q1dot = joint1_states[0], joint1_states[1]
        q2, q2dot = joint2_states[0], joint2_states[1]

        # Define state x
        x = [q1, q2, q1dot, q2dot]

        # Define data dictionary
        data = {
            'q1_plus': q1_plus,
            'q_tilde_1': q_tilde_1,
            'phi_fun': phi,
            'phiprime_fun': phiprime,
            'phipprime_fun': phipprime,
            'Hfun': H_fun,
            'dH_dt_fun': dH_dt_fun,
            'D': D_func,
            'C': C_func,
            'G': G_func,
            'B': B_num,
            'l': l_val,
            'lc': lc_val,
            'm': m_val,
            'Iz': Iz_val,
            'g': g_val
        }

        # Calculate torque
        torque = calculate_torque(x, data)

        # Control the joint using torque control
        p.setJointMotorControl2(bodyIndex=robot_id, jointIndex=joint1_index, controlMode=p.TORQUE_CONTROL, force=torque[0])

        # Step simulation
        p.stepSimulation()

        # Sleep to maintain real-time simulation
        time.sleep(1./240.)

    # Disconnect from the simulator
    p.disconnect()



if __name__ == "__main__":
    main()

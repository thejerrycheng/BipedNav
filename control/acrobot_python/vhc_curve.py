import numpy as np
from sympy import symbols, cos, sin, pi, diff, Matrix, Function
from sympy.simplify import simplify
from sympy.utilities.lambdify import lambdify

# Physical parameters
l = 1
lc = 0.5
m = 1
Iz = 1/12 * m * l**2
g = 9.81
psi = np.deg2rad(0)
incline_degrees = 0

# Control parameters
Kp = 40**2
Kd = 20*2
Kdd = 1

# Initialize the variables
beta = 0.316637
v1 = -0.894373
v2 = 1.9

# Symbolic computations
t, q1, q2, x1, x2, q1dot, q2dot, x1dot, x2dot, tau = symbols('t q1 q2 x1 x2 q1dot q2dot x1dot x2dot tau', real=True)
q = Matrix([q1, q2])
qdot = Matrix([q1dot, q2dot])
q_minus = Matrix([(pi-beta)/2 + psi, pi+beta])
q_plus = Matrix([(pi+beta)/2 + psi, pi-beta])
q1_plus = q_plus[0]
q2_plus = q_plus[1]
q1_minus = q_minus[0]
q2_minus = q_minus[1]
q_tilde_1 = q1_plus - q1_minus
q_tilde_2 = q2_plus - q2_minus

# VHC Design precedure
data = {}
data['q1_plus'] = q1_plus
data['q2_plus'] = q2_plus
data['q1_minus'] = q1_minus
data['q2_minus'] = q2_minus
data['q_tilde_1'] = q_tilde_1
data['q_tilde_2'] = q_tilde_2

import numpy as np
from sympy import Matrix

# Define a random 4x4 matrix for Delta
np.random.seed(42)  # Seed for reproducibility
Delta_random = np.random.rand(4, 4)

# Convert the random numpy matrix to a sympy matrix
Delta = Matrix(Delta_random)

print("Random Delta matrix:")
print(Delta)

# Placeholder for Deltaqdot and Delta function
Delta = ...  # This should be provided or computed earlier in the code
Deltaqdot = Delta.jacobian(qdot)
Deltadotfun = lambdify((q1, q2, q1dot, q2dot), Deltaqdot, 'numpy')
data['Deltadotfun'] = Deltadotfun

Deltaqdot_evaluated = Deltaqdot.subs({q1: q_minus[0], q2: q_minus[1]}).evalf()
I_evaluated = Deltaqdot_evaluated[2:4, :]
data['I'] = I_evaluated

# Function f for hybrid invariance condition
I_11 = I_evaluated[0, 0]
I_12 = I_evaluated[0, 1]
I_21 = I_evaluated[1, 0]
I_22 = I_evaluated[1, 1]

f = lambda v1: (-q_tilde_1 * (-I_21 * q_tilde_1 + I_22 * v1)) / (-I_11 * q_tilde_1 + I_12 * v1)
data['f'] = f

# Linear Optimization Approach Finding a
k = 6
theta_vals = np.array([0, 1, 0.5, 0, 1, 0.5])
powers = np.arange(k)

V = np.zeros((6, k))
for i in range(6):
    V[i, :] = theta_vals[i] ** powers

V[4, 1:] = np.arange(1, k) * theta_vals[4] ** (np.arange(1, k) - 1)
V[5, 1:] = np.arange(1, k) * theta_vals[5] ** (np.arange(1, k) - 1)
V[6, 1:] = np.arange(1, k) * theta_vals[6] ** (np.arange(1, k) - 1)

b = np.array([q2_plus.evalf(), q2_minus.evalf(), pi, f(v1).evalf(), v1, v2])
a = np.linalg.solve(V, b)
data['a'] = a

phi = lambda theta: np.polyval(a[::-1], theta)
phiprime = lambda theta: np.polyval(np.polyder(a[::-1]), theta)
phipprime = lambda theta: np.polyval(np.polyder(np.polyder(a[::-1])), theta)
data['phi_fun'] = phi
data['phiprime_fun'] = phiprime
data['phipprime_fun'] = phipprime

sigma = lambda theta: np.array([q1_plus - theta * q_tilde_1, phi(theta)])
sigmaprime = lambda theta: np.array([-q_tilde_1, phiprime(theta)])
sigmapprime = lambda theta: np.array([0, phipprime(theta)])
data['sigma_fun'] = sigma
data['sigmaprime_fun'] = sigmaprime
data['sigmapprime_fun'] = sigmapprime

theta_range = np.linspace(0, 1, 100)
sigma_values = np.array([sigma(th) for th in theta_range])
sigma_prime_values = np.array([sigmaprime(th) for th in theta_range])
sigma_double_prime_values = np.array([sigmapprime(th) for th in theta_range])

q1_values = sigma_values[:, 0]
q2_values = sigma_values[:, 1]
q1_prime_values = sigma_prime_values[:, 0]
q2_prime_values = sigma_prime_values[:, 1]
q1_pprime_values = sigma_double_prime_values[:, 0]
q2_pprime_values = sigma_double_prime_values[:, 1]

import matplotlib.pyplot as plt

plt.figure(figsize=(10, 8))
plt.subplot(3, 1, 1)
plt.plot(theta_range, q1_values, 'b-', theta_range, q2_values, 'r-')
plt.title('Sigma Function - Joint Angles')
plt.xlabel('Theta')
plt.ylabel('Joint Angles')
plt.legend(['q1(theta)', 'q2(theta)'])

plt.subplot(3, 1, 2)
plt.plot(theta_range, q1_prime_values, 'b--', theta_range, q2_prime_values, 'r--')
plt.title('Sigma Prime Function - Derivatives of Joint Angles')
plt.xlabel('Theta')
plt.ylabel('Joint Velocity')
plt.legend(['q1\'(theta)', 'q2\'(theta)'])

plt.subplot(3, 1, 3)
plt.plot(theta_range, q1_pprime_values, 'b--', theta_range, q2_pprime_values, 'r--')
plt.title('Sigma Double Prime Function - Second Derivatives of Joint Angles')
plt.xlabel('Theta')
plt.ylabel('Joint Acceleration')
plt.legend(['q1\'\'(theta)', 'q2\'\'(theta)'])
plt.tight_layout()
plt.show()

# Convert the function to symbolic expression
theta, thetadot = symbols('theta thetadot')
a_sym = np.array(a, dtype=float)  # Ensure numeric coefficients are in float
phi_sym = sum(a_sym[i] * theta**i for i in range(len(a_sym)))
phiprime_sym = diff(phi_sym, theta)
phipprime_sym = diff(phiprime_sym, theta)

sigma_sym = Matrix([q1_plus - theta * q_tilde_1, phi_sym])
sigmaprime_sym = Matrix([-q_tilde_1, phiprime_sym])
sigmapprime_sym = Matrix([0, phipprime_sym])

# Error term
y = q2 - phi_sym.subs(theta, (q1_plus - q1) / q_tilde_1)
H = y.jacobian(q)
Hfun = lambdify((q1, q2), H, 'numpy')
data['Hfun'] = Hfun

dHdt = Matrix.zeros(*H.shape)
for i in range(H.shape[0]):
    for j in range(H.shape[1]):
        for k in range(len(q)):
            dH_ij_dqk = diff(H[i, j], q[k])
            dHdt[i, j] += dH_ij_dqk * qdot[k]
dH_dt_fun = lambdify((q1, q2, q1dot, q2dot), dHdt, 'numpy')
data['dH_dt_fun'] = dH_dt_fun

# Save the data structure using pickle
import pickle
with open('vhc_curve_data.pkl', 'wb') as f:
    pickle.dump(data, f)

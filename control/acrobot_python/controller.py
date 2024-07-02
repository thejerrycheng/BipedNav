import numpy as np
import matplotlib.pyplot as plt
from sympy import symbols, diff, Poly, lambdify

# Given VHC polynomial coefficients
a = [13.3225, -35.9442, 30.1847, -7.3543, 0.4246, 2.8250]

# Define the function phi and its derivatives using numpy's polyval
phi = lambda theta: np.polyval(a, theta)
phiprime = lambda theta: np.polyval(np.polyder(a), theta)
phipprime = lambda theta: np.polyval(np.polyder(np.polyder(a)), theta)

# Placeholder values for q1_plus and q_tilde_1
q1_plus = 1.0
q_tilde_1 = 0.5

# Define sigma and its derivatives
sigma = lambda theta: [q1_plus - theta * q_tilde_1, phi(theta)]
sigmaprime = lambda theta: [-q_tilde_1, phiprime(theta)]
sigmapprime = lambda theta: [0, phipprime(theta)]

# Define the theta range for plotting
theta_range = np.linspace(0, 1, 100)

# Evaluate sigma and its derivatives
sigma_values = np.array([sigma(th) for th in theta_range])
sigma_prime_values = np.array([sigmaprime(th) for th in theta_range])
sigma_double_prime_values = np.array([sigmapprime(th) for th in theta_range])

# Extract components for plotting
q1_values = sigma_values[:, 0]
q2_values = sigma_values[:, 1]
q1_prime_values = sigma_prime_values[:, 0]
q2_prime_values = sigma_prime_values[:, 1]
q1_pprime_values = sigma_double_prime_values[:, 0]
q2_pprime_values = sigma_double_prime_values[:, 1]

# Plotting
plt.figure(figsize=(10, 8))

plt.subplot(3, 1, 1)
plt.plot(theta_range, q1_values, 'b-', label='q1(theta)')
plt.plot(theta_range, q2_values, 'r-', label='q2(theta)')
plt.title('Sigma Function - Joint Angles')
plt.xlabel('Theta')
plt.ylabel('Joint Angles')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(theta_range, q1_prime_values, 'b--', label='q1\'(theta)')
plt.plot(theta_range, q2_prime_values, 'r--', label='q2\'(theta)')
plt.title('Sigma Prime Function - Derivatives of Joint Angles')
plt.xlabel('Theta')
plt.ylabel('Joint Velocity')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(theta_range, q1_pprime_values, 'b--', label='q1\'\'(theta)')
plt.plot(theta_range, q2_pprime_values, 'r--', label='q2\'\'(theta)')
plt.title('Sigma Double Prime Function - Second Derivatives of Joint Angles')
plt.xlabel('Theta')
plt.ylabel('Joint Acceleration')
plt.legend()

plt.tight_layout()
plt.show()

# Symbolic conversion
theta, thetadot = symbols('theta thetadot')
q1_plus, q_tilde_1 = symbols('q1_plus q_tilde_1')
q1, q2 = symbols('q1 q2')
q = [q1, q2]
qdot = symbols('q1dot q2dot')
a_sym = Poly(a[::-1], theta)  # Convert to sympy polynomial
phi_sym = a_sym.as_expr()  # Convert to symbolic expression
phiprime_sym = diff(phi_sym, theta)  # First derivative
phipprime_sym = diff(phiprime_sym, theta)  # Second derivative

# Define sigma and its derivatives symbolically
sigma_sym = [q1_plus - theta * q_tilde_1, phi_sym]
sigmaprime_sym = [-q_tilde_1, phiprime_sym]
sigmapprime_sym = [0, phipprime_sym]

# Control input calculation
# Error term: Substitute theta in with an expression with q1
y = q2 - phi_sym.subs(theta, (q1_plus - q1)/q_tilde_1)

# Gradient (Jacobian of a scalar function)
H = [diff(y, q_i) for q_i in q]

# Define the time derivative of Jacobian
dHdt = np.zeros((len(H), len(q)), dtype=object)

for i in range(len(H)):
    for j in range(len(q)):
        dH_ij_dqj = diff(H[i], q[j])
        dHdt[i, j] = dH_ij_dqj * qdot[j]

# Convert to functions
H_fun = lambdify([q1, q2], H)
dH_dt_fun = lambdify([q1, q2, qdot[0], qdot[1]], dHdt)

# Example usage
data = {
    'phi_fun': phi,
    'phiprime_fun': phiprime,
    'phipprime_fun': phipprime,
    'sigma_fun': sigma,
    'sigmaprime_fun': sigmaprime,
    'sigmapprime_fun': sigmapprime,
    'Hfun': H_fun,
    'dH_dt_fun': dH_dt_fun
}

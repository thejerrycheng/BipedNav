import numpy as np
from sympy import symbols, cos, sin, Matrix, simplify
from sympy import lambdify
from sympy import hessian

# Physical parameters
l = 1
lc = 0.5
m = 1
Iz = 1/12 * m * l**2
g = 9.81
psi = np.deg2rad(0)
incline_degrees = 0

# Control parameters
Kp = 40**2  # Default Kp = 20**2; Kd = 20*2
Kd = 20*2
Kdd = 1

# Initialize the variables
beta = 0.316637
v1 = -0.894373
v2 = 1.9
animationFileName = 'acrobot_animation_beta_pi_4.mp4'

# Symbolic computations
symbolic_computations = True  # Set this to True or False as needed

if symbolic_computations:
    print("Start calculating the modeling of the robot ...")
    
    # Define symbolic variables
    t, q1, q2, x1, x2, q1dot, q2dot, x1dot, x2dot, tau = symbols('t q1 q2 x1 x2 q1dot q2dot x1dot x2dot tau', real=True)
    q = Matrix([q1, q2])
    x = Matrix([x1, x2])
    qbar = Matrix([q1, q2, x1, x2])
    qdot = Matrix([q1dot, q2dot])
    xdot = Matrix([x1dot, x2dot])
    qbardot = Matrix([q1dot, q2dot, x1dot, x2dot])
    
    # Define centres of mass of two links
    rc1 = x + lc * Matrix([cos(q1), sin(q1)])
    rc2 = x + l * Matrix([cos(q1), sin(q1)]) + lc * Matrix([cos(q1 + q2), sin(q1 + q2)])
    
    # Compute time derivatives of centres of mass
    rc1dot = rc1.jacobian(qbar) * qbardot
    rc2dot = rc2.jacobian(qbar) * qbardot
    
    # Define the total kinetic energy of the robot
    K = 1/2 * m * (rc1dot.T * rc1dot + rc2dot.T * rc2dot)[0] + 1/2 * Iz * (q1dot**2 + (q1dot + q2dot)**2)
    K = simplify(K)
    
    # Extract the square symmetric matrix of the kinetic energy
    Dbar = simplify(hessian(K, qbardot))
    print("D bar looks like: ", Dbar)
    
    # Extract the matrix of the kinetic energy of the pinned robot
    D = Dbar[:2, :2]
    print("Done computing D matrix... ")
    print("D matrix: ", D)
    
    # Define the potential energy of the pinned robot
    P = m * g * (lc * sin(q1) + l * sin(q1) + lc * sin(q1 + q2))
    psi = np.deg2rad(incline_degrees)
    gvector = Matrix([[cos(psi), -sin(psi)], [sin(psi), cos(psi)]]) * Matrix([0, -1])
    P = - (m * g * lc * Matrix([cos(q1), sin(q1)]) + m * g * Matrix([l * cos(q1) + lc * cos(q1 + q2), l * sin(q1) + lc * sin(q1 + q2)])).T * gvector
    P = simplify(P[0])
    print("Done computing P matrix... ")
    print("P matrix: ", P)
    
    # Input matrix of pinned robot
    B = Matrix([0, 1])
    B_perp = Matrix([1, 0])
    
    # Computation of matrix C(q,qdot) of pinned robot
    C = Matrix.zeros(2, 2)
    for i in range(2):
        for j in range(2):
            for k in range(2):
                C[k, j] += 1/2 * (D.diff(q[i])[k, j] + D.diff(q[j])[k, i] - D.diff(q[k])[i, j]) * qdot[i]
    print("Done computing C matrix... ")
    print("C matrix: ", C)
    
    # Computation of gradient of the potential for the pinned robot
    P_matrix = Matrix([P])
    G = P_matrix.jacobian(q).T
    print("Done computing G matrix... ")
    print("G matrix: ", G)
    
    # Convert symbolic expressions to numerical functions using lambdify
    Dfun = lambdify((q1, q2), D, 'numpy')
    Gfun = lambdify((q1, q2), G, 'numpy')
    Cfun = lambdify((q1, q2, q1dot, q2dot), C, 'numpy')
    
    # Impact map computation
    T = -Matrix.eye(2) * q + Matrix([np.pi, 2 * np.pi])
    Delta2 = Matrix.vstack(T, -Matrix.eye(2) * qdot)
    print("Done computing the relabeling map... ")
    
    # First impact map
    px = l * Matrix([cos(q1) + cos(q1 + q2), sin(q1) + sin(q1 + q2)])
    xo = Matrix.zeros(2, 1)
    
    E = Matrix.hstack(px.jacobian(q), Matrix.eye(2))
    print("Done computing E matrix... ")
    
    # Use the provided inv_block_matrix results directly
    inv_block_matrix = Matrix([
        [-(12*(3*cos(q1 + q2)**2 + 3*sin(q1 + q2)**2 + 1))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         (6*(3*cos(q2) + 6*cos(q1 + q2)**2 + 6*sin(q1 + q2)**2 + 2))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         -(6*(2*sin(q1) + 6*cos(q1 + q2)**2*sin(q1) + 6*sin(q1 + q2)**2*sin(q1) - 3*sin(q1 + q2)*cos(q2)))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         (6*(2*cos(q1) + 6*cos(q1 + q2)**2*cos(q1) + 6*sin(q1 + q2)**2*cos(q1) - 3*cos(q1 + q2)*cos(q2)))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16)],
        [(6*(3*cos(q2) + 6*cos(q1 + q2)**2 + 6*sin(q1 + q2)**2 + 2))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         -(12*(3*cos(q2) + 3*cos(q1 + q2)**2 + 3*sin(q1 + q2)**2 - 3*cos(q1)**2 - 3*sin(q1)**2 + 5))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         (6*(2*sin(q1) - 8*sin(q1 + q2) + 3*cos(q2)*sin(q1) + 6*sin(q1 + q2)*cos(q1)**2 + 6*cos(q1 + q2)**2*sin(q1) + 6*sin(q1 + q2)*sin(q1)**2 + 6*sin(q1 + q2)**2*sin(q1) - 3*sin(q1 + q2)*cos(q2)))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         -(6*(2*cos(q1) - 8*cos(q1 + q2) + 3*cos(q1)*cos(q2) + 6*cos(q1 + q2)*cos(q1)**2 + 6*cos(q1 + q2)**2*cos(q1) + 6*cos(q1 + q2)*sin(q1)**2 + 6*sin(q1 + q2)**2*cos(q1) - 3*cos(q1 + q2)*cos(q2)))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16)],
        [-(6*(2*sin(q1) + 6*cos(q1 + q2)**2*sin(q1) + 6*sin(q1 + q2)**2*sin(q1) - 3*sin(q1 + q2)*cos(q2)))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         (6*(2*sin(q1) - 8*sin(q1 + q2) + 3*cos(q2)*sin(q1) + 6*sin(q1 + q2)*cos(q1)**2 + 6*cos(q1 + q2)**2*sin(q1) + 6*sin(q1 + q2)*sin(q1)**2 + 6*sin(q1 + q2)**2*sin(q1) - 3*sin(q1 + q2)*cos(q2)))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         -(12*(4*sin(q1 + q2)**2 + sin(q1)**2 + 3*cos(q1 + q2)**2*sin(q1)**2 - 3*sin(q1 + q2)**2*cos(q1)**2 - 3*sin(q1 + q2)*cos(q2)*sin(q1)))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         (6*(2*cos(q1)*sin(q1) + 8*cos(q1 + q2)*sin(q1 + q2) + 6*cos(q1 + q2)**2*cos(q1)*sin(q1) + 6*sin(q1 + q2)**2*cos(q1)*sin(q1) - 3*cos(q1 + q2)*cos(q2)*sin(q1) - 3*sin(q1 + q2)*cos(q1)*cos(q2) - 6*cos(q1 + q2)*sin(q1 + q2)*cos(q1)**2 - 6*cos(q1 + q2)*sin(q1 + q2)*sin(q1)**2))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16)],
        [(6*(2*cos(q1) + 6*cos(q1 + q2)**2*cos(q1) + 6*sin(q1 + q2)**2*cos(q1) - 3*cos(q1 + q2)*cos(q2)))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         -(6*(2*cos(q1) - 8*cos(q1 + q2) + 3*cos(q1)*cos(q2) + 6*cos(q1 + q2)*cos(q1)**2 + 6*cos(q1 + q2)**2*cos(q1) + 6*cos(q1 + q2)*sin(q1)**2 + 6*sin(q1 + q2)**2*cos(q1) - 3*cos(q1 + q2)*cos(q2)))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         (6*(2*cos(q1)*sin(q1) + 8*cos(q1 + q2)*sin(q1 + q2) + 6*cos(q1 + q2)**2*cos(q1)*sin(q1) + 6*sin(q1 + q2)**2*cos(q1)*sin(q1) - 3*cos(q1 + q2)*cos(q2)*sin(q1) - 3*sin(q1 + q2)*cos(q1)*cos(q2) - 6*cos(q1 + q2)*sin(q1 + q2)*cos(q1)**2 - 6*cos(q1 + q2)*sin(q1 + q2)*sin(q1)**2))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16)],
        [-(3*(2*sin(q1) + 6*cos(q1 + q2)**2*sin(q1) + 6*sin(q1 + q2)**2*sin(q1) - 9*sin(q1 + q2)*cos(q2)))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         (3*(2*sin(q1) - 24*sin(q1 + q2) + 3*cos(q2)*sin(q1) + 18*sin(q1 + q2)*cos(q1)**2 + 6*cos(q1 + q2)**2*sin(q1) + 18*sin(q1 + q2)*sin(q1)**2 + 6*sin(q1 + q2)**2*sin(q1) - 9*sin(q1 + q2)*cos(q2)))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         -(24*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 12*cos(q1)**2 + 9*cos(q2)**2 + 18*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 54*cos(q1 + q2)**2*sin(q1)**2 - 18*sin(q1 + q2)**2*cos(q1)**2 - 36*sin(q1 + q2)*cos(q2)*sin(q1) - 16)/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         (3*(2*cos(q1)*sin(q1) + 24*cos(q1 + q2)*sin(q1 + q2) + 6*cos(q1 + q2)**2*cos(q1)*sin(q1) + 6*sin(q1 + q2)**2*cos(q1)*sin(q1) - 3*cos(q1 + q2)*cos(q2)*sin(q1) - 9*sin(q1 + q2)*cos(q1)*cos(q2) - 18*cos(q1 + q2)*sin(q1 + q2)*cos(q1)**2 - 18*cos(q1 + q2)*sin(q1 + q2)*sin(q1)**2))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16)],
        [ (3*(2*cos(q1) + 6*cos(q1 + q2)**2*cos(q1) + 6*sin(q1 + q2)**2*cos(q1) - 9*cos(q1 + q2)*cos(q2)))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         -(3*(2*cos(q1) - 24*cos(q1 + q2) + 3*cos(q1)*cos(q2) + 18*cos(q1 + q2)*cos(q1)**2 + 6*cos(q1 + q2)**2*cos(q1) + 18*cos(q1 + q2)*sin(q1)**2 + 6*sin(q1 + q2)**2*cos(q1) - 9*cos(q1 + q2)*cos(q2)))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         (3*(2*cos(q1)*sin(q1) + 24*cos(q1 + q2)*sin(q1 + q2) + 6*cos(q1 + q2)**2*cos(q1)*sin(q1) + 6*sin(q1 + q2)**2*cos(q1)*sin(q1) - 9*cos(q1 + q2)*cos(q2)*sin(q1) - 3*sin(q1 + q2)*cos(q1)*cos(q2) - 18*cos(q1 + q2)*sin(q1 + q2)*cos(q1)**2 - 18*cos(q1 + q2)*sin(q1 + q2)*sin(q1)**2))/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16),
         -(24*cos(q1 + q2)**2 - 48*sin(q1 + q2)**2 + 18*cos(q1)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 - 18*cos(q1 + q2)**2*sin(q1)**2 + 54*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 36*cos(q1 + q2)*cos(q1)*cos(q2) - 16)/(12*cos(q1)**2 - 48*sin(q1 + q2)**2 - 48*cos(q1 + q2)**2 + 9*cos(q2)**2 + 12*sin(q1)**2 + 36*cos(q1 + q2)**2*cos(q1)**2 + 36*cos(q1 + q2)**2*sin(q1)**2 + 36*sin(q1 + q2)**2*cos(q1)**2 + 36*sin(q1 + q2)**2*sin(q1)**2 - 16)]
    ])

    Deltaqdot = Matrix.hstack(eye(2), Matrix.zeros(2, 4)) * inv_block_matrix * Matrix.vstack(
        Dbar * Matrix.vstack(eye(2), xo.jacobian(q)),
        Matrix.zeros(2, 2)
    )

    print("Done computing the impact map... ")
    print("Deltaqdot: ", Deltaqdot)
    
    # Composition of two maps
    Delta1 = simplify(Matrix.vstack(q, Deltaqdot * qdot))
    Delta = simplify(Delta2.subs(list(zip(Matrix([q, qdot]), Delta1))))
    Deltafun = lambdify((q1, q2, q1dot, q2dot), Delta, 'numpy')
    print("Done computing the impact map function ... ")
    print("Delta: ", Delta)
    
    # Cache objects in data structure
    data = {
        'D': Dfun,
        'C': Cfun,
        'G': Gfun,
        'B': B,
        'B_perp': B_perp,
        'tau': tau,
        'Delta': Delta,
        'Deltafun': Deltafun
    }
    print("Exporting ...  ")

    # Save the data (use pickle or another method to save the data structure in Python)
    import pickle
    with open('walking_acrobot_model.pkl', 'wb') as f:
        pickle.dump(data, f)
        print("SUCCESS! ")
else:
    # Loading robot model (load the saved data structure in Python)
    import pickle
    with open('walking_acrobot_model.pkl', 'rb') as f:
        data = pickle.load(f)

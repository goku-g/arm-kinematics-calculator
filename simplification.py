import sympy as sp
import numpy as np

#  Define individual transformation functions
def rotZ(theta):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta), 0, 0],
        [sp.sin(theta),  sp.cos(theta), 0, 0],
        [0,              0,             1, 0],
        [0,              0,             0, 1]
    ])

def transZ(d):
    return sp.Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ])

def transX(a):
    return sp.Matrix([
        [1, 0, 0, a],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def rotX(alpha):
    return sp.Matrix([
        [1, 0,           0,          0],
        [0, sp.cos(alpha), -sp.sin(alpha), 0],
        [0, sp.sin(alpha),  sp.cos(alpha), 0],
        [0, 0,           0,          1]
    ])

#  Combine - DH Transformation
def dhTransformMat(theta, alpha, d, a):
    """Compute the Denavit-Hartenberg transformation matrix
    Args:
        theta (sp.Symbol): Joint angle
        alpha (sp.Symbol): Link twist angle
        d (sp.Symbol): Link offset
        a (sp.Symbol): Link length
    
    Returns:
        A (sp.Matrix[4,4]): DH transformation matrix
    """
    A = rotZ(theta) * transZ(d) * transX(a) * rotX(alpha)
    return sp.simplify(A)

#  Calculate A
def forwardMat(thetas, alphas, ds, ase):
    """Compute the overall transformation matrix from base to end-effector using DH parameters
    Args:
        thetas (list[n, 1]): list of all joint variables
        alphas (list[n, 1]): list of all link twist angles
        ds (list[n, 1]): list of all link offsets
        ase (list[n, 1]): list of all link lengths
    
    Returns:
        T (sp.Matrix[4,4]): Overall transformation matrix from base to end-effector
    """
    T = sp.Matrix.eye(4)
    for theta, alpha, d, a in zip(thetas, alphas, ds, ase):
        A_i = dhTransformMat(theta, alpha, d, a)
        T = T * A_i
    return T

# calculate origins and z axes of each frame
def computeOriginsAndZAxes(thetas, alphas, ds, ase):
    """Extract each Origin and Zaxis based on DH parameters

    Args:
        thetas (list[n, 1]): list of all joint variables
        alphas (list[n, 1]): list of all link twist angles
        ds (list[n, 1]): list of all link offsets
        ase (list[n, 1]): list of all link lengths

    Returns:
        origins (list[n-1, 3, 1]): origins of each frame [O_0, O_1, ..., O_n]
        z_axes (list[n-1, 3, 1]): z-axis of each frame [Z_0, Z_1, ..., Z_n-1]
    """
    T = sp.Matrix.eye(4)
    origins = [T[:3, 3]]  # origin of base frame O_0
    z_axes = [T[:3, 2]]  # z-axis of base frame Z_0
    
    # initial approach (less efficient)
    # for theta, alpha, d, a in zip(thetas, alphas, ds, ase):
    #     A_i = dhTransformMat(theta, alpha, d, a)
    #     T = T * A_i
    #     origins.append(T[:3, 3])
    #     z_axes.append(T[:3, 2])
    
    for i in range(len(thetas)):
        T = forwardMat(thetas[:i+1], alphas[:i+1], ds[:i+1], ase[:i+1])
        origins.append(T[:3, 3])
        z_axes.append(T[:3, 2])
        # Debug prints
        # print("\nTransformation Matrix T_0", i+1, ":\n")
        # sp.pprint(sp.simplify(T), use_unicode=True)
    
    # Debug prints
    # sp.pprint(origins, use_unicode=True)
    # sp.pprint(z_axes, use_unicode=True)
    return origins, z_axes

def computeJacobian(origins, z_axes, joint_types):
    """Compute the Jacobian matrix for a robotic manipulator.
    Args:
        origins (list[n, 3, 1]): list of origin positions [O_0, O_1, ..., O_n]
        z_axes (list[n, 3, 1]): list of z-axis vectors [Z_0, Z_1, ..., Z_n-1]
        joint_types (list[n, str]): list of joint types ('r' for revolute, 'p' for prismatic)
    
    Returns:
        J (sp.Matrix[6,n]): Jacobian matrix where J = [Jv; Jw]
    """
    n = len(joint_types)  # number of joints
    
    J = sp.Matrix.zeros(6, n)  # Initialize Jacobian matrix
    o_n = origins[-1]  # end-effector position (O_n)
    
    for i in range(n):
        z_prev = z_axes[i]  # This is Z_{i-1} in standard notation
        o_prev = origins[i]  # This is O_{i-1} in standard notation
        
        if joint_types[i] == 'r':
            # revolute joint
            # Jvi = Z_{i-1} × (O_n - O_{i-1})
            J[:3, i] = z_prev.cross(o_n - o_prev)
            # Jwi = Z_{i-1}
            J[3:, i] = z_prev
            
        elif joint_types[i] == 'p':
            # prismatic joint
            # Jvi = Z_{i-1}
            J[:3, i] = z_prev
            # Jwi = 0
            J[3:, i] = sp.Matrix([
                [0],
                [0],
                [0]])
    
    return J

def printMatrices(origins, z_axes, J, compute_type='sym'):
    """Print origins, z-axes, and Jacobian matrix.

    Args:
        origins (list[n, 3, 1]): list of origin positions [O_0, O_1, ..., O_n]
        z_axes (list[n, 3, 1]): list of z-axis vectors [Z_0, Z_1, ..., Z_n-1]
        J (sp.Matrix[6,n]): Jacobian matrix where J = [Jv; Jw]
        compute_type (str, optional): 'sym' for symbolic, 'num' for numerical. Defaults to 'sym'.
    """
    sp.init_printing()  # Pretty output
    
    print("\nOrigins of each frame:\n")
    for i, origin, z_axis in zip(range(len(origins)), origins, z_axes):
        print(f"\nFrame {i}:\n")
        print("\nOrigin:\n")
        if compute_type == 'sym':
            sp.pprint(sp.simplify(origin), use_unicode=True)
            print()
            print(sp.latex(sp.simplify(origin)))
        elif compute_type == 'num':
            sp.pprint(origin.evalf(), use_unicode=True)
        print("\nZ-axis:\n")
        if compute_type == 'sym':
            sp.pprint(sp.simplify(z_axis), use_unicode=True)
            print()
            print(sp.latex(sp.simplify(z_axis)))
        elif compute_type == 'num':
            sp.pprint(z_axis.evalf(), use_unicode=True)
        print()
    
    print("\nJacobian Matrices Ji:\n")
    for i in range(J.rows):
        print(f"\nJ{i+1} - Jacobian for {i+1} ({'revolute' if joint_types[i]=='r' else 'prismatic'}) joint:\n")
        if compute_type == 'sym':
            sp.pprint(sp.simplify(J[:, i]), use_unicode=True)
            print()
            print(sp.latex(sp.simplify(J[:, i])))
        elif compute_type == 'num':
            sp.pprint(J[:, i].evalf(), use_unicode=True)
        print()
        
    print("\nFull Jacobian Matrix J:\n")
    if compute_type == 'sym':
        sp.pprint(sp.simplify(J), use_unicode=True)
        print()
        print(sp.latex(sp.simplify(J)))
    elif compute_type == 'num':
        sp.pprint(J.evalf(), use_unicode=True)
    
    if compute_type == 'num':
        # simplification and rounding for numerical rank calculation (MATLAB style)
        rounded_mat = np.array(J, dtype=float)
        rank = np.linalg.matrix_rank(rounded_mat)
        print("\nRank of Jacobian Matrix J:", rank)

if __name__ == "__main__":
    
    compute_type = 'sym'  # 'sym' or 'num'
    joint_types = [ 'r', 'r', 'r', 'r', 'r', 'r' ]  #joint types
    
    # DH parameters for two joints (example)
    # thetas = [ sp.symbols('theta_1'), sp.symbols('theta_2') ]
    # alphas = [ sp.symbols('alpha_1'), sp.symbols('alpha_2') ]
    # ds = [ sp.symbols('d_1'), sp.symbols('d_2') ]
    # ase = [ sp.symbols('a_1'), sp.symbols('a_2') ]
    
    # this is for a 6 DOF robot arm (it's FK solution has error in the robot modeling and control book's solution P.92) - Cylindrical robot with spherical wrist (symbolic values)
    # thetas = [ sp.symbols('theta_1'), 0, 0, sp.symbols('theta_4'), sp.symbols('theta_5'), sp.symbols('theta_6') ]
    # ds = [ sp.symbols('d_1'), sp.symbols('d_2'), sp.symbols('d_3'), 0, 0, sp.symbols('d_6') ]
    # ase = [ 0, 0, 0, 0, 0, 0 ]
    # alphas = [ 0, -sp.pi/2, 0, -sp.pi/2, sp.pi/2, 0 ]
    
    #  the Stanford manipulator: Lec06-Jacobian.pdf p34 (wrong o_1, J_6 and missing o_2)
    # thetas = [ sp.symbols('theta_1'), sp.symbols('theta_2'), 0, sp.symbols('theta_4'), sp.symbols('theta_5'), sp.symbols('theta_6') ]
    # ds = [ 0, sp.symbols('d_2'), sp.symbols('d_3'), 0, 0, sp.symbols('d_6') ]
    # ase = [ 0, 0, 0, 0, 0, 0 ]
    # alphas = [ -sp.pi/2, sp.pi/2, 0, -sp.pi/2, sp.pi/2, 0 ]
    
    # DH parameters for PUMA 560 manipulator (6 DoF) (numerical values)
    # thetas = [ sp.pi/8, sp.pi/4, sp.pi/3, sp.pi/4, 0, -0.3 ]
    # alphas = [ 0, -sp.pi/2, 0, -sp.pi/2, sp.pi/2, -sp.pi/2 ]
    # ase = [ 0, 0, 17, 3.7, 0, 0 ]
    # ds = [ 0, 0, 9.2, 17.05, 0, 0 ]
    
    # DH parameters for PUMA 560 manipulator (6 DoF) (symbolic values)
    thetas = [ sp.symbols('theta_1'), sp.symbols('theta_2'), sp.symbols('theta_3'), sp.symbols('theta_4'), sp.symbols('theta_5'), sp.symbols('theta_6') ]
    alphas = [ 0, -sp.pi/2, 0, -sp.pi/2, sp.pi/2, -sp.pi/2 ]
    ase = [ 0, 0, sp.symbols('a_2'), sp.symbols('a_3'), 0, 0 ]
    ds = [ 0, 0, sp.symbols('d_3'), sp.symbols('d_4'), 0, 0 ]
    
    # T_i0 = forwardMat(thetas, alphas, ds, ase)
    # sp.init_printing()  # Pretty output
    # print("Denavit-Hartenberg Transformation Matrix T_i0:\n")
    # sp.pprint(T_i0, use_unicode=True)
    
    origins, z_axes = computeOriginsAndZAxes(thetas, alphas, ds, ase)
    J = computeJacobian(origins, z_axes, joint_types)
    
    printMatrices(origins, z_axes, J, compute_type)


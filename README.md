# Robot Kinematics Calculator

This Python script provides tools for calculating Denavit-Hartenberg (DH) transformation matrices, forward kinematics, and Jacobian matrices for robotic manipulators. It supports both symbolic and numerical calculations using the SymPy library.

## Features

- Computation of DH transformation matrices
- Forward kinematics calculation
- Origin and Z-axis extraction for each frame
- Jacobian matrix computation
- Support for both revolute and prismatic joints
- LaTeX code generation for symbolic calculations
- Numerical calculations with rank computation

## Requirements

- Python 3.x
- SymPy
- NumPy

## Usage

### Basic Setup

```python
import sympy as sp
import numpy as np
```

### 1. Setting Computation Type

Choose between symbolic ('sym') or numerical ('num') calculations:

```python
compute_type = 'sym'  # or 'num' for numerical calculations
```

### 2. Define Joint Types

Specify the type of each joint in your robot:
- 'r' for revolute joints
- 'p' for prismatic joints

Example for a 6-DOF robot with all revolute joints:
```python
joint_types = ['r', 'r', 'r', 'r', 'r', 'r']
```

### 3. Define DH Parameters

Specify the DH parameters for your robot. You can use either symbolic or numerical values.

#### Symbolic Example (PUMA 560):
```python
# Joint angles (θ)
thetas = [sp.symbols('theta_1'), sp.symbols('theta_2'), sp.symbols('theta_3'),
          sp.symbols('theta_4'), sp.symbols('theta_5'), sp.symbols('theta_6')]
# Link twist angles (α)
alphas = [0, -sp.pi/2, 0, -sp.pi/2, sp.pi/2, -sp.pi/2]
# Link lengths (a)
ase = [0, 0, sp.symbols('a_2'), sp.symbols('a_3'), 0, 0]
# Link offsets (d)
ds = [0, 0, sp.symbols('d_3'), sp.symbols('d_4'), 0, 0]
```

#### Numerical Example (PUMA 560):
```python
thetas = [sp.pi/8, sp.pi/4, sp.pi/3, sp.pi/4, 0, -0.3]
alphas = [0, -sp.pi/2, 0, -sp.pi/2, sp.pi/2, -sp.pi/2]
ase = [0, 0, 17, 3.7, 0, 0]
ds = [0, 0, 9.2, 17.05, 0, 0]
```

### 4. Calculate Transformations

#### DH Transformation Matrix
The `dhTransformMat()` function computes the individual transformation matrix for each joint:
```python
A = dhTransformMat(theta, alpha, d, a)
```

#### Forward Kinematics
The `forwardMat()` function computes the overall transformation matrix from base to any given joints:
```python
T = forwardMat(thetas, alphas, ds, ase)
```

### 5. Compute Jacobian

First, calculate origins and z-axes for each frame:
```python
origins, z_axes = computeOriginsAndZAxes(thetas, alphas, ds, ase)
```

Then compute the Jacobian matrix:
```python
J = computeJacobian(origins, z_axes, joint_types)
```

### 6. Print Results

Use the `printMatrices()` function to display all results:
```python
printMatrices(origins, z_axes, J, compute_type)
```

This will print:
- Origins of each frame
- Z-axes of each frame
- Individual Jacobian columns for each joint
- Full Jacobian matrix
- LaTeX code for each matrix (in symbolic mode)
- Matrix rank (in numerical mode)

## Example Robot Configurations

The script includes example DH parameters for several robot configurations:

1. Simple 2-DOF Robot
2. 6-DOF Cylindrical Robot with Spherical Wrist
3. Stanford Manipulator
4. PUMA 560 (both symbolic and numerical versions)

## Output Format

### Symbolic Mode
- Prints simplified symbolic expressions
- Generates LaTeX code for each matrix
- Uses Unicode characters for better readability

### Numerical Mode
- Prints numerical values
- Calculates matrix rank
- Rounds values for better readability

## Notes

- For symbolic calculations, the output includes LaTeX code that can be used in mathematical documents
- The Jacobian matrix is structured as J = [J<sub>v</sub> ; J<sub>w</sub>] where:
  - Jv represents the linear velocity component
  - Jw represents the angular velocity component
- For revolute joints:
  - Jv = Z<sub>i-1</sub> × (O_n - O<sub>i-1</sub>)
  - Jw = Z<sub>i-1</sub>
- For prismatic joints:
  - Jv = Z<sub>i-1</sub>
  - Jw = 0

## References

1. Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2020). *Robot Modeling and Control (2nd Edition)*. John Wiley & Sons. Retrieved from: [https://app.knovel.com/hotlink/toc/id:kpRMCE0004/robot-modeling-control/robot-modeling-control](https://app.knovel.com/hotlink/toc/id:kpRMCE0004/robot-modeling-control/robot-modeling-control)

2. Wang, J. (n.d.). Lecture slides for MECH 3550 course, University of Manitoba. Lab website: [Robot Autonomy Lab](https://www.robotautonomylab.com/)

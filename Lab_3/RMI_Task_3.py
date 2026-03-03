import numpy as np
import sympy as sp
import matplotlib.pyplot as plt

# Function to compute the homogeneous transformation matrix using DH parameters
def transformation_matrix(q, a, d, alpha):
    """
    Compute the homogeneous transformation matrix for a given set of DH parameters.
    Parameters:
    q: joint angle
    a: link length
    d: link offset
    alpha: link twist
    Returns:
    T: 4x4 transformation matrix
    """
    return sp.Matrix([[sp.cos(q), -sp.sin(q)*sp.cos(alpha), sp.sin(q)*sp.sin(alpha), a*sp.cos(q)],
                      [sp.sin(q), sp.cos(q)*sp.cos(alpha), -sp.cos(q)*sp.sin(alpha), a*sp.sin(q)],
                      [0, sp.sin(alpha), sp.cos(alpha), d],
                      [0, 0, 0, 1]])

# Main function to compute the Jacobian matrix
def jacobian(q):
    """
    Compute the Jacobian matrix for a robot manipulator using the DH parameters.
    Parameters, relationship between joint velocities and end-effector velocities:
    q: List or ndarray of joint angles [q1, q2, q3, q4, q5, q6]
    Returns:
    j_numeric: 6x6 numeric Jacobian matrix
    """
    # Define symbolic variables for joint angles q1, q2, ..., q6
    q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')
    q_sym = [q1, q2, q3, q4, q5, q6]  # List to store symbolic variables for joint angles

    # Denavit-Hartenberg parameters for each joint: a (link lengths), d (link offsets), alpha (link twists)
    a = [0, 1, 0.5, 0, 0, 0]  # Link lengths
    d = [1, 0, 0, 1, 0.2, 1]  # Link offsets
    alpha = [sp.pi/2, 0, sp.pi/2, sp.pi/2, sp.pi/2, 0]  # Link twists

    # Compute transformation matrices for each joint based on DH parameters
    T00 = sp.eye(4)  # Identity matrix for base frame
    T01 = transformation_matrix(q1, a[0], d[0], alpha[0])  # Transformation from base to joint 1
    T12 = transformation_matrix(q2, a[1], d[1], alpha[1])  # Transformation from joint 1 to joint 2
    T23 = transformation_matrix(q3, a[2], d[2], alpha[2])  # Transformation from joint 2 to joint 3
    T34 = transformation_matrix(q4, a[3], d[3], alpha[3])  # Transformation from joint 3 to joint 4
    T45 = transformation_matrix(q5, a[4], d[4], alpha[4])  # Transformation from joint 4 to joint 5
    T56 = transformation_matrix(q6, a[5], d[5], alpha[5])  # Transformation from joint 5 to joint 6

    # Compute the cumulative transformation matrices for each link
    T02 = T01 * T12  # Transformation from base to joint 2
    T03 = T02 * T23  # Transformation from base to joint 3
    T04 = T03 * T34  # Transformation from base to joint 4
    T05 = T04 * T45  # Transformation from base to joint 5
    T06 = T05 * T56  # Transformation from base to joint 6

    # Extract the z-vectors (rotation axes) and the final position vector
    z0 = T00[0:3, 2]  # z-axis of the base frame
    z1 = T01[0:3, 2]  # z-axis of joint 1
    z2 = T02[0:3, 2]  # z-axis of joint 2
    z3 = T03[0:3, 2]  # z-axis of joint 3
    z4 = T04[0:3, 2]  # z-axis of joint 4
    z5 = T05[0:3, 2]  # z-axis of joint 5
    p06 = T06[0:3, 3]  # Position of end-effector (last joint)

    # Compute the Jacobian matrix columns by taking the derivatives of the position
    # vector p06 with respect to each joint angle (q1, q2, ..., q6)
    j1 = sp.Matrix.vstack(sp.diff(p06, q1), z0)  # Column for joint 1
    j2 = sp.Matrix.vstack(sp.diff(p06, q2), z1)  # Column for joint 2
    j3 = sp.Matrix.vstack(sp.diff(p06, q3), z2)  # Column for joint 3
    j4 = sp.Matrix.vstack(sp.diff(p06, q4), z3)  # Column for joint 4
    j5 = sp.Matrix.vstack(sp.diff(p06, q5), z4)  # Column for joint 5
    j6 = sp.Matrix.vstack(sp.diff(p06, q6), z5)  # Column for joint 6

    # Assemble the full Jacobian matrix by horizontally stacking all the columns
    j = sp.Matrix.hstack(j1, j2, j3, j4, j5, j6)

    # Substitute numerical values for the joint angles (q1, q2, ..., q6)
    J_numeric = j.subs({q_sym[i]: q[i] for i in range(6)})

    # Convert the symbolic Jacobian matrix to a numeric numpy array
    j_numeric = np.array(J_numeric).astype(np.float64)
    
    return j_numeric

# Example joint angles in radians
q_values = [0.1, np.pi/2, 0.3, np.pi/2, 0.5, 0.6]

# Compute the Jacobian matrix for the given joint angles
j = jacobian(q_values)

# Print the resulting Jacobian matrix
print("Jacobian Matrix:")
print(j)

# Compute numeric transformation matrix using DH parameters
def compute_transformation(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st*ca,  st*sa, a*ct],
                     [st,  ct*ca, -ct*sa, a*st],
                     [0,     sa,     ca,    d],
                     [0,      0,      0,    1]])

# Visualize the 6-DOF robotic arm
def visualize_robot(q):
    a = [0, 1, 0.5, 0, 0, 0]
    d = [1, 0, 0, 1, 0.2, 1]
    alpha = [np.pi/2, 0, np.pi/2, np.pi/2, np.pi/2, 0]
    thetas = [q[0], q[1], q[2]+np.pi/2, q[3], q[4], q[5]]

    points = [np.array([0,0,0,1])]
    T = np.eye(4)
    for i in range(6):
        T = T @ compute_transformation(thetas[i], d[i], a[i], alpha[i])
        points.append(T @ np.array([0,0,0,1]))
    points = np.array(points)

    fig = plt.figure(figsize=(10,8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(points[:,0], points[:,1], points[:,2], '-o', linewidth=4, markersize=8, label='Robotic Arm')
    ax.scatter(points[-1,0], points[-1,1], points[-1,2], color='green', s=100, label='End Effector')
    
    for i, p in enumerate(points):
        ax.text(p[0], p[1], p[2], f' J{i}', color='red')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('6-DOF Robotic Arm Configuration')
    ax.legend()

    max_range = np.array([points[:,0].max()-points[:,0].min(),
                          points[:,1].max()-points[:,1].min(),
                          points[:,2].max()-points[:,2].min()]).max()/2
    mid_x = (points[:,0].max()+points[:,0].min())*0.5
    mid_y = (points[:,1].max()+points[:,1].min())*0.5
    mid_z = (points[:,2].max()+points[:,2].min())*0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()

# Call this at the end of your code
visualize_robot(q_values)
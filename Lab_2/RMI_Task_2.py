import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def compute_transformation(theta, d, a, alpha):
    """
    Compute the homogeneous transformation matrix using DH parameters.
    Parameters:
        theta (float): Joint angle in radians.
        d (float): Link offset along z-axis.
        a (float): Link length along x-axis.
        alpha (float): Link twist in radians.
    Returns:
        np.ndarray: 4x4 transformation matrix.
    """
    c_theta, s_theta = np.cos(theta), np.sin(theta)
    c_alpha, s_alpha = np.cos(alpha), np.sin(alpha)
    # Handle special cases for theta and alpha
    c_theta = 0 if (theta == np.pi/2 or theta == -np.pi/2) else np.cos(theta)
    c_alpha = 0 if (alpha == np.pi/2 or alpha == -np.pi/2) else np.cos(alpha)
    
    transformation_matrix = np.array([
        [c_theta, -s_theta * c_alpha,  s_theta * s_alpha, a * c_theta],
        [s_theta,  c_theta * c_alpha, -c_theta * s_alpha, a * s_theta],
        [0,        s_alpha,            c_alpha,           d],
        [0,        0,                  0,                 1]
    ])
    return transformation_matrix


def inverse_kinematics(xi):
    """
    Inverse kinematics calculation for a 6-DOF robotic arm.
    Parameters:
        xi (ndarray): Desired end-effector state [x, y, z, phi, theta, psi].
    Returns:
        q (ndarray): Joint angles [q1, q2, q3, q4, q5, q6].
    """
    # Print xi values
    print(f"Desired end-effector state (xi): x={xi[0]}, y={xi[1]}, z={xi[2]}, phi={xi[3]}, theta={xi[4]}, psi={xi[5]}")

    x, y, z, phi, theta, psi = xi

    # Robot-specific parameters (replace with your robot's DH parameters)
    a = np.array([0, 1, 0, 0, 0, 0])
    d = np.array([1, 0, 0, 1, 0, 1])
    alpha = np.array([np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0])

    # Step 1: Compute position of wrist center (P_04)
    p_06 = np.array([x, y, z])
    R_06 = (
        np.array([ 
            [np.cos(phi), -np.sin(phi), 0],
            [np.sin(phi),  np.cos(phi), 0],
            [0,            0,           1]
        ]) @
        np.array([ 
            [np.cos(theta),  0, np.sin(theta)],
            [0,              1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ]) @
        np.array([ 
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi),  np.cos(psi), 0],
            [0,            0,           1]
        ])
    )
    p_04 = p_06 - d[5] * R_06@np.array([0, 0, 1])
    xc, yc, zc = p_04

    # Step 2: Compute q1, q2, q3
    q = [0] * 6
    q[0] = np.arctan2(yc, xc)

    # Compute q2 and q3 using geometry
    r2 = xc**2 + yc**2
    r1 = zc - d[0]
    cos_q3 = (r1**2 + r2 - a[1]**2 - d[3]**2) / (2 * a[1] * d[3])
    
    if np.isclose(cos_q3, 1):
        q[2] = 0
        q[1] = np.arctan2(zc-d[0], np.sqrt(r2))
    elif np.isclose(cos_q3, -1):
        q[2] = np.pi
    elif abs(cos_q3) < 1:
        q[2] = np.arctan2(np.sqrt(1 - cos_q3**2), cos_q3)
    
    q[1] = np.arctan2(r1, np.sqrt(r2)) - np.arctan2(d[3] * np.sin(q[2]), a[1] + d[3] * np.cos(q[2]))

    # Step 3: Compute q4, q5, q6
    T01 = compute_transformation(q[0], d[0], a[0], alpha[0])
    T12 = compute_transformation(q[1], d[1], a[1], alpha[1])
    T23 = compute_transformation(q[2] + np.pi/2, d[2], a[2], alpha[2])

    T03 = T01 @ T12 @ T23
    R03 = T03[:3, :3]
    R36 = R03.T @ R_06

    if abs(R36[2, 2]) < 1:
        q[3] = np.arctan2(R36[1, 2], R36[0, 2])
        q[4] = np.arctan2(np.sqrt(1 - R36[2, 2]**2), R36[2, 2])
        q[5] = np.arctan2(R36[2, 1], -R36[2, 0])
    elif R36[2, 2] == 1:
        q[3] = 0
        q[4] = 0
        q[5] = np.arctan2(R36[1, 0], R36[0, 0])
    elif R36[2, 2] == -1:
        q[3] = np.arctan2(-R36[0, 1], -R36[0, 0])
        q[4] = np.pi
        q[5] = 0

    return q


# Test the function with sample xi values
xi = [2.422, 0.06, 2.49, -0.22, 0.63, -1.82]  # Desired end-effector state
joint_angles = inverse_kinematics(xi)

# Print the joint angles
print("Joint Angles (radians):")
print(f"q1: {joint_angles[0]:.2f}")
print(f"q2: {joint_angles[1]:.2f}")
print(f"q3: {joint_angles[2]:.2f}")
print(f"q4: {joint_angles[3]:.2f}")
print(f"q5: {joint_angles[4]:.2f}")
print(f"q6: {joint_angles[5]:.2f}")

def visualize_robot(q):
    """
    Visualize the 6-DOF robotic arm given joint angles q.
    """
    # Robot DH parameters (matching your inverse_kinematics function)
    a = np.array([0, 1, 0, 0, 0, 0])
    d = np.array([1, 0, 0, 1, 0, 1])
    alpha = np.array([np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0])
    
    # Apply the same joint adjustments as in your forward kinematics steps
    # Note: T23 used q[2] + np.pi/2 in your code
    thetas = [q[0], q[1], q[2] + np.pi/2, q[3], q[4], q[5]]
    
    # Compute forward kinematics to find joint positions
    points = [np.array([0, 0, 0, 1])]
    T_accum = np.eye(4)
    
    for i in range(6):
        Ti = compute_transformation(thetas[i], d[i], a[i], alpha[i])
        T_accum = T_accum @ Ti
        points.append(T_accum @ np.array([0, 0, 0, 1]))
        
    points = np.array(points)
    
    # Create 3D Plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot links and joints
    ax.plot(points[:, 0], points[:, 1], points[:, 2], '-o', linewidth=4, markersize=8, label='Robotic Arm')
    
    # Label joints
    for i, p in enumerate(points):
        ax.text(p[0], p[1], p[2], f' J{i}', color='red')
        
    # Highlight End Effector
    ax.scatter(points[-1, 0], points[-1, 1], points[-1, 2], color='green', s=100, label='End Effector')
    
    # Formatting
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('6-DOF Robotic Arm Configuration')
    ax.legend()
    
    # Set equal aspect ratio for realistic proportions
    max_range = np.array([points[:, 0].max()-points[:, 0].min(), 
                          points[:, 1].max()-points[:, 1].min(), 
                          points[:, 2].max()-points[:, 2].min()]).max() / 2.0

    mid_x = (points[:, 0].max()+points[:, 0].min()) * 0.5
    mid_y = (points[:, 1].max()+points[:, 1].min()) * 0.5
    mid_z = (points[:, 2].max()+points[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    plt.show()

# Visualize the calculated joint angles
visualize_robot(joint_angles)

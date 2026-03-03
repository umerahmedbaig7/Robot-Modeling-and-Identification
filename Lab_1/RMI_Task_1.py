import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Function to calculate the homogeneous transformation matrix for a single joint
def ht(theta, d, a, alpha):
    """
    Calculate the homogeneous transformation matrix using Denavit-Hartenberg parameters.

    Parameters:
    - theta: joint angle (rotation around z-axis)
    - d: offset along z-axis
    - a: link length along x-axis
    - alpha: twist angle around x-axis

    Returns:
    - A 4x4 transformation matrix.
    """
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                 np.cos(alpha),                 d],
        [0,              0,                             0,                             1]
    ])

# Function to perform forward kinematics using DH parameters
def forward_kinematics(joint_angles, a, d, alpha):
    q0 = np.array([0, 0, np.pi/2, 0, 0, 0])  # Offset angle
    joint_angles = np.array(joint_angles) + q0
    """
    Calculate the forward kinematics of a manipulator using Denavit-Hartenberg parameters.
    Returns:
    - The position (x, y, z) and orientation (phi, theta, psi) of the end-effector.
    """
    # Number of joints in the manipulator
    n = len(joint_angles)

    # Check that the number of joint angles matches the number of DH parameters
    if not (len(a) == len(d) == len(alpha) == n):
        raise ValueError("Mismatch in the number of DH parameters and joint angles")
    
    # Initialize the transformation matrix as the identity matrix
    T = np.eye(4)
    
    # Loop through each joint and multiply the individual transformation matrices
    for i in range(n):
        T_i = ht(joint_angles[i], d[i], a[i], alpha[i])  # Get the transformation matrix for the current joint
        T = T @ T_i  # Multiply to get the cumulative transformation matrix
    
    # Extract the position (x, y, z) from the transformation matrix
    x, y, z = T[:3, 3]
    
    # Extract the rotation matrix (R) from the top-left 3x3 part of the matrix
    R = T[:3, :3]
    
    # Compute Euler angles (phi, theta, psi) from the rotation matrix
    if abs(R[2, 2]) < 1:  # General case where pitch is not ±90 degrees
        phi = np.arctan2(R[1, 2], R[0, 2])  # Roll angle
        theta = np.arctan2(np.sqrt(1 - R[2, 2]**2), R[2, 2])  # Pitch angle
        psi = np.arctan2(R[2, 1], -R[2, 0])  # Yaw angle
    elif R[2, 2] == 1:  # Special case where pitch = 0 (no rotation about y-axis)
        phi, theta, psi = 0, 0, np.arctan2(R[1, 0], R[0, 0])  # Only yaw rotation
    else:  # Special case where pitch = ±90 degrees
        phi = np.arctan2(-R[0, 1], -R[0, 0])  # Roll angle
        theta = np.pi  # Pitch is 180 degrees
        psi = 0  # No yaw rotation
    
    # Return the position (x, y, z) and orientation (phi, theta, psi) of the end-effector
    return [round(x, 2), round(y, 2), round(z, 2),
            round(phi, 2), round(theta, 2), round(psi, 2)]

# Define the DH parameters for a 6-DOF manipulator (example configuration)
a = [0, 1, 0, 0, 0, 0]  # Link lengths (a)
d = [1, 0, 0, 1, 0, 1]  # Link offsets (d)
alpha = [np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0]  # Link twist angles (alpha)

# Define the joint angles (in degrees) and convert them to radians
joint_angles = [1.1, 1.2, 1.3, 1.4, 1.5, 1.7]  # Joint angles in degrees

# Perform forward kinematics to compute the end-effector pose
end_effector_pose = forward_kinematics(joint_angles, a, d, alpha)

print("\nEnd-Effector Pose (x, y, z, phi, theta, psi):")
print(end_effector_pose)

def forward_kinematics_with_frames(joint_angles, a, d, alpha):
    q0 = np.array([0, 0, np.pi/2, 0, 0, 0])
    joint_angles = np.array(joint_angles) + q0

    T = np.eye(4)
    frames = [T.copy()]

    for i in range(len(joint_angles)):
        T_i = ht(joint_angles[i], d[i], a[i], alpha[i])
        T = T @ T_i
        frames.append(T.copy())

    return frames


def plot_robot(frames):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    xs, ys, zs = [], [], []

    for T in frames:
        xs.append(T[0, 3])
        ys.append(T[1, 3])
        zs.append(T[2, 3])

    ax.plot(xs, ys, zs, marker='o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('6-DOF Manipulator')
    plt.show()


# ---- RUN VISUALIZATION ----
frames = forward_kinematics_with_frames(joint_angles, a, d, alpha)
plot_robot(frames)
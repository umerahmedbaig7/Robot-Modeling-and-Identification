<div align="center">

# 🤖 Robot Modeling and Identification
### MSc Robotics and Artificial Intelligence — Course Repository

[![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![NumPy](https://img.shields.io/badge/NumPy-1.21%2B-013243?style=for-the-badge&logo=numpy&logoColor=white)](https://numpy.org/)
[![SymPy](https://img.shields.io/badge/SymPy-1.9%2B-3B5526?style=for-the-badge&logo=sympy&logoColor=white)](https://www.sympy.org/)
[![Matplotlib](https://img.shields.io/badge/Matplotlib-3.4%2B-11557C?style=for-the-badge&logo=python&logoColor=white)](https://matplotlib.org/)
[![Status](https://img.shields.io/badge/Status-Complete-brightgreen?style=for-the-badge)]()
[![Field](https://img.shields.io/badge/Field-Robotics%20%26%20AI-blueviolet?style=for-the-badge&logo=ros&logoColor=white)]()

<br>

> *"To command a robot, you must first speak the language of mathematics it was built upon."*

<br>

**Author:** Umer Ahmed Baig Mughal <br>
**Programme:** MSc Robotics and Artificial Intelligence <br>
**Specialization:** Computer Vision · Robot Modeling and Control · Perception · Autonomous Systems

</div>

---

## 📋 Table of Contents

- [📖 About This Repository](#-about-this-repository)
- [🗂️ Repository Structure](#️-repository-structure)
- [🔬 Course Overview](#-course-overview)
- [🧪 Lab Summaries](#-lab-summaries)
  - [🔵 Lab 1 — Forward Kinematics](#-lab-1--forward-kinematics)
  - [🟠 Lab 2 — Inverse Kinematics](#-lab-2--inverse-kinematics)
  - [🟣 Lab 3 — Jacobian Matrix](#-lab-3--jacobian-matrix)
- [🔗 Progressive Learning Pathway](#-progressive-learning-pathway)
- [⚙️ Robot Description — The Common Platform](#️-robot-description--the-common-platform)
- [🚀 Quick Start](#-quick-start)
- [📊 Results at a Glance](#-results-at-a-glance)
- [🧰 Tech Stack](#-tech-stack)
- [👤 Author](#-author)
- [📄 License](#-license)

---

## 📖 About This Repository

This repository contains the complete implementation of all three laboratory assignments from the **Robot Modeling and Identification (RMI)** course, part of the MSc in Robotics and Artificial Intelligence. Every lab builds upon the mathematical foundations established in the previous one, forming a coherent and progressive study of the kinematics and velocity analysis of a 6-degree-of-freedom (6-DOF) serial robotic manipulator.

All solutions are implemented entirely from first principles in **Python**, using only NumPy, SymPy, and Matplotlib — without any robotics-specific toolboxes. This deliberate constraint ensures a deep understanding of the underlying mathematics rather than a surface-level application of pre-built functions.

### 🎯 What You Will Find Here

| 📁 Lab | 🏷️ Topic | 🧠 Core Concept | 🐍 Key Library |
|:------:|:--------:|:---------------:|:--------------:|
| Lab 1 | Forward Kinematics | DH Matrices · Euler Extraction | NumPy |
| Lab 2 | Inverse Kinematics | Wrist Decoupling · Geometric IK | NumPy |
| Lab 3 | Jacobian Matrix | Symbolic Differentiation · Velocity Mapping | SymPy + NumPy |

---

## 🗂️ Repository Structure

```
📦 Robot-Modeling-and-Identification/
│
├── 📁 Lab1/
│   ├── 🐍 MIR_Task_1.py          # Forward kinematics + 3D visualization
│   ├── 📄 README.md               # Lab 1 documentation
│   └── 📁 Results/
│       └── 🖼️ lab1_robot.png     # 3D kinematic chain plot
│
├── 📁 Lab2/
│   ├── 🐍 RMI_Task_2.py          # Inverse kinematics + 3D visualization
│   ├── 📄 README.md               # Lab 2 documentation
│   └── 📁 Results/
│       └── 🖼️ lab2_robot.png     # 3D arm configuration plot
│
├── 📁 Lab3/
│   ├── 🐍 RMI_Task_3.py          # Jacobian computation + 3D visualization
│   ├── 📄 README.md               # Lab 3 documentation
│   └── 📁 Results/
│       └── 🖼️ lab3_robot.png     # 3D arm configuration plot
│
└── 📄 README.md                   # ← You are here
```

---

## 🔬 Course Overview

The **Robot Modeling and Identification** course develops rigorous analytical and computational tools for characterising the motion of robotic mechanisms. The curriculum addresses both the geometric and differential aspects of robot kinematics — the two pillars upon which all advanced topics in robot control, motion planning, and force interaction are built.

The three labs progress in a deliberate sequence:

```
Joint Angles  ──[Lab 1: FK]──►  End-Effector Pose
                                       │
End-Effector Pose  ──[Lab 2: IK]──►  Joint Angles
                                       │
Joint Velocities  ──[Lab 3: J]──►  End-Effector Velocity
```

Each lab introduces a new layer of the kinematic modelling stack, with every subsequent lab depending on — and verifying — the results of the previous one.

---

## 🧪 Lab Summaries

---

### 🔵 Lab 1 — Forward Kinematics

<div align="center">

[![Lab1](https://img.shields.io/badge/Lab%201-Forward%20Kinematics-0078D7?style=flat-square&logo=python&logoColor=white)]()
[![DH](https://img.shields.io/badge/Method-DH%20Parameters-lightblue?style=flat-square)]()
[![Euler](https://img.shields.io/badge/Output-ZYZ%20Euler%20Angles-lightblue?style=flat-square)]()

</div>

#### 📌 Task Description

> Given a set of **joint angles**, compute the **position and orientation** of the robot's end-effector in Cartesian space, and visualize the full 3D kinematic chain.

The forward kinematics problem asks: *"If I know where each joint is pointing, where is the tip of the robot arm?"* This is the foundational problem of robot kinematics — every other kinematic computation either inverts it (IK) or differentiates it (Jacobian).

**What the task requires:**
- Model a 6-DOF revolute-joint manipulator using the **Denavit-Hartenberg (DH) parameterisation**.
- Construct a 4×4 **homogeneous transformation matrix** for each joint from four DH parameters (θ, d, a, α).
- Compose the full kinematic chain by sequential matrix multiplication to obtain the base-to-end-effector transformation `⁰T₆`.
- Extract the end-effector **Cartesian position** (x, y, z) from the translation column of `⁰T₆`.
- Extract the end-effector **orientation** as ZYZ Euler angles (φ, θ, ψ) from the rotation sub-matrix, with correct handling of all three singularity branches.
- Store all intermediate transformation matrices and render the full **3D skeleton** of the kinematic chain.

#### 🔑 Key Concepts

| Concept | Description |
|---------|-------------|
| 🔧 DH Convention | Four-parameter minimal geometric description of each link–joint pair |
| 🔢 Homogeneous Transform | 4×4 matrix encoding rotation + translation in a single operation |
| ⛓️ Kinematic Chain | `⁰T₆ = ⁰T₁ · ¹T₂ · ²T₃ · ³T₄ · ⁴T₅ · ⁵T₆` |
| 📐 ZYZ Euler Angles | Three-angle orientation representation extracted analytically |
| ⚠️ Singularity Handling | Three-branch atan2 formulation for θ = 0 and θ = π |
| 🖼️ 3D Visualization | Joint origins connected sequentially to form the robot skeleton |

#### 📤 Output

```
Input:  joint_angles = [1.1, 1.2, 1.3, 1.4, 1.5, 1.7]  rad
Output: End-Effector Pose → [x=0.61, y=−0.98, z=2.44, φ=−0.63, θ=1.66, ψ=2.62]
```

📂 **[→ View Lab 1 Full Documentation](Lab1/README.md)**

---

### 🟠 Lab 2 — Inverse Kinematics

<div align="center">

[![Lab2](https://img.shields.io/badge/Lab%202-Inverse%20Kinematics-E85D04?style=flat-square&logo=python&logoColor=white)]()
[![Method](https://img.shields.io/badge/Method-Geometric%20Decoupling-orange?style=flat-square)]()
[![Output](https://img.shields.io/badge/Output-Joint%20Angles-orange?style=flat-square)]()

</div>

#### 📌 Task Description

> Given a desired **end-effector position and orientation**, compute the **six joint angles** that would place the robot in exactly that configuration, then visualize the resulting arm posture.

The inverse kinematics problem is the mathematical inverse of Lab 1 — and considerably harder. A single end-effector pose may be achievable by multiple arm configurations (elbow-up vs. elbow-down), or by none at all if the target lies outside the robot's reachable workspace. The task requires a **closed-form analytic solution** — not iterative numerical approximation.

**What the task requires:**
- Accept a desired end-effector pose `ξ = [x, y, z, φ, θ, ψ]` and reconstruct the corresponding rotation matrix `R₀₆` from ZYZ Euler angles.
- Apply the **Pieper kinematic decoupling principle**: exploit the spherical wrist geometry to separate the 6-DOF problem into a 3-DOF position sub-problem and a 3-DOF orientation sub-problem.
- Compute the **wrist centre** position by stepping back from the end-effector along the tool's approach axis by the distal link offset `d₆`.
- Solve the **position sub-problem geometrically**: derive q₁ from the XY projection of the wrist centre; derive q₃ via the law of cosines on the arm triangle; derive q₂ from the residual planar geometry.
- Construct the partial transformation `T₀₃` and isolate the **wrist rotation matrix** `R₃₆ = R₀₃ᵀ · R₀₆`.
- Solve the **orientation sub-problem algebraically**: extract q₄, q₅, q₆ via ZYZ Euler decomposition of `R₃₆`.
- Implement a **numerical robustness improvement** over Lab 1: force cosines of exact ±π/2 angles to zero, preventing IEEE 754 floating-point residuals from accumulating across the kinematic chain.
- Verify the solution by **round-trip FK confirmation**.

#### 🔑 Key Concepts

| Concept | Description |
|---------|-------------|
| 🔄 Pieper Decoupling | Splits 6-DOF IK into independent 3+3 DOF sub-problems |
| 🎯 Wrist Centre | `p₀₄ = p₀₆ − d₆ · R₀₆ · ẑ` — pivot point of the spherical wrist |
| 📐 Law of Cosines | Solves the planar elbow triangle for q₃ |
| 🔢 R₃₆ Isolation | `R₃₆ = R₀₃ᵀ · R₀₆` — decouples wrist from arm orientation |
| 🔧 Zero-Forcing | `cos(±π/2)` forced to exact 0 for numerical cleanliness |
| ✅ Round-Trip Check | FK(IK(ξ)) = ξ — zero position error confirms correctness |

#### 📤 Output

```
Input:  xi = [2.422, 0.06, 2.49, −0.22, 0.63, −1.82]
Output: q = [0.10, 0.20, 0.30, 0.40, 0.50, 0.70]  rad
        FK verification error = 0.000000 m  ✓
```

📂 **[→ View Lab 2 Full Documentation](Lab2/README.md)**

---

### 🟣 Lab 3 — Geometric Jacobian Matrix

<div align="center">

[![Lab3](https://img.shields.io/badge/Lab%203-Jacobian%20Matrix-7209B7?style=flat-square&logo=python&logoColor=white)]()
[![Method](https://img.shields.io/badge/Method-Symbolic%20Differentiation-purple?style=flat-square)]()
[![Output](https://img.shields.io/badge/Output-6×6%20Jacobian-purple?style=flat-square)]()

</div>

#### 📌 Task Description

> Compute the **6×6 geometric Jacobian matrix** that linearly maps joint velocities to end-effector spatial velocities, using **exact symbolic differentiation** via SymPy, and evaluate it numerically at a given joint configuration.

The Jacobian is the differential counterpart of the forward kinematics map — it tells you *how fast* the end-effector moves (and rotates) for a given set of joint velocity commands. It is the central tool of velocity kinematics, static force analysis, singularity detection, redundancy resolution, and real-time motion control. The task requires deriving this matrix analytically rather than by finite difference.

**What the task requires:**
- Declare all six joint angles as **SymPy symbolic variables** and construct all six DH transformation matrices as **symbolic SymPy matrices** — not numerical arrays.
- Compose the cumulative transformations `T₀₁` through `T₀₆` symbolically to produce `p₀₆` as a closed-form trigonometric expression in all six joint variables.
- Compute the **linear Jacobian columns** by taking exact symbolic partial derivatives of `p₀₆` with respect to each joint angle: `Jv_i = ∂p₀₆/∂qᵢ`.
- Compute the **angular Jacobian columns** by extracting the z-axis (third column) of each cumulative frame transformation: `Jw_i = zᵢ₋₁`.
- Assemble the full **6×6 Jacobian** by stacking `[Jv_i; Jw_i]` for each joint and horizontally concatenating all columns.
- **Substitute numerical joint angles** via SymPy's `.subs()` method and convert to a NumPy array for output.
- Identify and interpret structural properties of the resulting matrix, including the **zero linear velocity column** for joint 6 and the **parallel-axis pattern** shared by joints 2 and 3.

#### 🔑 Key Concepts

| Concept | Description |
|---------|-------------|
| 📊 Geometric Jacobian | `J = [Jv; Jw]` — maps `q̇` to `[ẋ; ω]` |
| 🧮 Symbolic Diff | `Jv_i = sp.diff(p₀₆, qᵢ)` — exact, not approximate |
| 🔀 Z-Axis Propagation | `Jw_i = zᵢ₋₁` — rotation axis of joint *i* |
| 🔢 Manipulability | `w = √det(J·Jᵀ) = 0.2898` — distance from singularity |
| 📉 Singular Values | `[2.427, 1.575, 1.387, 0.585, 0.507, 0.184]` — velocity ellipsoid shape |
| 🔌 Two-Engine Design | Symbolic engine (Jacobian) + Numeric engine (visualization) |

#### 📤 Output

```
Input:  q = [0.1, π/2, 0.3, π/2, 0.5, 0.6]  rad

Jacobian Matrix J(q):
[[ 0.486  -1.696  -0.701   0.161   0.543   0.000]
 [-0.042  -0.170  -0.070  -0.185  -0.827   0.000]
 [ 0.000  -0.090  -0.090  -0.458   0.142   0.000]
 [ 0.000   0.100   0.100   0.951  -0.294  -0.786]
 [ 0.000  -0.995  -0.995   0.095  -0.030  -0.561]
 [ 1.000   0.000   0.000   0.296   0.955  -0.259]]

Rank: 6 (full) | det: 0.2898 | Manipulability: 0.2898
```

📂 **[→ View Lab 3 Full Documentation](Lab3/README.md)**

---

## 🔗 Progressive Learning Pathway

The three labs are carefully sequenced so that each one depends on and validates the previous:

```
╔═════════════════════════════════════════════════════════════════════════════╗
║                     KINEMATIC MODELLING PROGRESSION                         ║
╠══════════════════╦═══════════════════╦══════════════════════════════════════╣
║  🔵 LAB 1       ║  🟠 LAB 2         ║  🟣 LAB 3                           ║
║  Forward         ║  Inverse          ║  Jacobian                            ║
║  Kinematics      ║  Kinematics       ║  Matrix                              ║
╠══════════════════╬═══════════════════╬══════════════════════════════════════╣
║  q → ξ           ║  ξ → q            ║  q̇ → ξ̇                               ║
║  Joint space     ║  Task space       ║  Velocity space                      ║
║  → Task space    ║  → Joint space    ║  Linear map                          ║
╠══════════════════╬═══════════════════╬══════════════════════════════════════╣
║  DH matrices     ║  DH matrices +    ║  DH matrices +                       ║
║  Matrix chain    ║  Geometric IK     ║  Symbolic diff                       ║
║  Euler extract   ║  Wrist decoupling ║  Z-axis propagation                  ║
╠══════════════════╬═══════════════════╬══════════════════════════════════════╣
║  Output used     ║  Output verified  ║  Builds on FK                        ║
║  as IK target    ║  by Lab 1 FK      ║  chain from Lab 1                    ║
╚══════════════════╩═══════════════════╩══════════════════════════════════════╝
```

**Dependency chain:**
- 🔵 Lab 1 establishes the DH convention, the `ht()` transformation matrix, and the ZYZ Euler extraction — all reused in Labs 2 and 3.
- 🟠 Lab 2 uses Lab 1's FK as a verification oracle: `FK(IK(ξ)) = ξ` confirms the IK solution.
- 🟣 Lab 3 builds the full symbolic kinematic chain from the same DH matrices as Lab 1, then differentiates them to produce the Jacobian.

---

## ⚙️ Robot Description — The Common Platform

Labs 1 and 2 share the **same robot geometry**. Lab 3 uses a **modified variant** with three adjusted DH parameters.

### 🔵🟠 Labs 1 & 2 — Base Robot

A **6-DOF PUMA-class revolute-joint serial manipulator** characterised by:

| Joint | dᵢ (m) | aᵢ (m) | αᵢ (rad) | Role |
|:-----:|:-------:|:-------:|:--------:|------|
|   1   | 1       | 0       | +π/2     | Base rotation |
|   2   | 0       | 1       |  0       | Shoulder — primary reach |
|   3   | 0       | 0       | +π/2     | Elbow — no translation |
|   4   | 1       | 0       | −π/2     | Wrist pitch |
|   5   | 0       | 0       | +π/2     | Wrist yaw — no translation |
|   6   | 1       | 0       |  0       | Wrist roll |

> 💡 Joints 3 and 5 have `a=0, d=0` — they contribute **only rotation**, no translation. Their frame origins coincide with the preceding joint (5 visible markers from 7 computed frames).

### 🟣 Lab 3 — Modified Robot

Three parameters are changed to produce a **more general, fully extended geometry**:

| Parameter | Labs 1 & 2 | Lab 3 | Effect |
|-----------|:----------:|:-----:|--------|
| `a[2]` — Link 3 length | 0 m | **0.5 m** | Link 3 now has physical length → J2 ≠ J3 |
| `d[4]` — Joint 5 offset | 0 m | **0.2 m** | Joint 5 has non-zero offset → J4 ≠ J5 |
| `alpha[3]` — Joint 4 twist | −π/2 | **+π/2** | Wrist bending direction reversed |

> 💡 In Lab 3, **all 7 joint frame origins are geometrically distinct** — the 3D plot shows 7 visible markers, compared to 5 in Labs 1 and 2.

---

## 🚀 Quick Start

### 1️⃣ Clone the Repository

```bash
git clone https://github.com/umerahmedbaig7/Robot-Modeling-and-Identification.git
cd Robot-Modeling-and-Identification
```

### 2️⃣ Install Dependencies

```bash
pip install numpy matplotlib sympy
```

> 📌 All three packages are available in Anaconda by default. For a clean virtual environment:
> ```bash
> python -m venv rmi_env
> source rmi_env/bin/activate      # Windows: rmi_env\Scripts\activate
> pip install numpy matplotlib sympy
> ```

### 3️⃣ Run Each Lab

```bash
# 🔵 Lab 1 — Forward Kinematics
python Lab1/MIR_Task_1.py

# 🟠 Lab 2 — Inverse Kinematics
python Lab2/RMI_Task_2.py

# 🟣 Lab 3 — Jacobian Matrix  (takes ~5–30s due to SymPy)
python Lab3/RMI_Task_3.py
```

Each script prints its numerical output to the terminal and opens an **interactive 3D Matplotlib window** that can be rotated, panned, and zoomed with the mouse.

---

## 📊 Results at a Glance

### 🔵 Lab 1 — Forward Kinematics Result

| Input Joint Angles (rad) | q₁=1.1 | q₂=1.2 | q₃=1.3 | q₄=1.4 | q₅=1.5 | q₆=1.7 |
|--------------------------|:------:|:------:|:------:|:------:|:------:|:------:|
| **End-Effector Pose** | x=0.61 m | y=−0.98 m | z=2.44 m | φ=−0.63 rad | θ=1.66 rad | ψ=2.62 rad |

### 🟠 Lab 2 — Inverse Kinematics Result

| Input Pose (m / rad) | x=2.422 | y=0.06 | z=2.49 | φ=−0.22 | θ=0.63 | ψ=−1.82 |
|----------------------|:-------:|:------:|:------:|:-------:|:------:|:-------:|
| **Joint Angles (rad)** | q₁=0.10 | q₂=0.20 | q₃=0.30 | q₄=0.40 | q₅=0.50 | q₆=0.70 |
| **FK Verification Error** | | | **0.000000 m ✅** | | | |

### 🟣 Lab 3 — Jacobian Properties

| Property | Value | Interpretation |
|----------|:-----:|----------------|
| 📐 Shape | 6 × 6 | Square — 6 joints, 6 task DOF |
| 🏆 Rank | **6** | Full rank — non-singular configuration |
| 🔢 Determinant | **0.2898** | Non-zero — all motions achievable |
| 📏 Condition Number | **13.18** | Moderate directional anisotropy |
| 🌀 Manipulability | **0.2898** | Healthy distance from singularity |
| 📉 Singular Values | `2.43 · 1.57 · 1.39 · 0.59 · 0.51 · 0.18` | ~13× spread in velocity ellipsoid |

---

## 🧰 Tech Stack

<div align="center">

| 🛠️ Tool | 🔖 Version | 🎯 Role in This Project |
|:-------:|:---------:|:----------------------:|
| ![Python](https://img.shields.io/badge/-Python-3776AB?logo=python&logoColor=white) | 3.8+ | Core language — all three labs |
| ![NumPy](https://img.shields.io/badge/-NumPy-013243?logo=numpy&logoColor=white) | ≥ 1.21 | Matrix ops, FK/IK, visualization |
| ![SymPy](https://img.shields.io/badge/-SymPy-3B5526?logo=sympy&logoColor=white) | ≥ 1.9 | Symbolic DH matrices, exact differentiation (Lab 3) |
| ![Matplotlib](https://img.shields.io/badge/-Matplotlib-11557C?logo=python&logoColor=white) | ≥ 3.4 | 3D robot arm visualization (all labs) |

</div>

**No robotics toolboxes. No MATLAB. No ROS.** Every algorithm — from the DH matrix up to the Jacobian — is derived and implemented from mathematical first principles.

---

## 👤 Author

<div align="center">

### Umer Ahmed Baig Mughal

🎓 **MSc Robotics and Artificial Intelligence** <br>
🔬 *Specialisation: Computer Vision · Robot Modeling and Control · Perception · Autonomous Systems*

[![GitHub](https://img.shields.io/badge/GitHub-umerahmedbaig7-181717?style=for-the-badge&logo=github)](https://github.com/umerahmedbaig7)

</div>

---

## 📄 License

This repository is intended for **academic and research use**. All work was developed as part of the *Robot Modeling and Identification* course within the MSc Robotics and Artificial Intelligence program. Redistribution, modification, and use in derivative academic work are permitted with appropriate attribution to the original author.

---

<div align="center">

*Robot Modeling and Identification — MSc Robotics and Artificial Intelligence*

⭐ *If this repository helped you understand robot kinematics, consider giving it a star!* ⭐

</div>

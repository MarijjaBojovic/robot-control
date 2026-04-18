# 🤖 2DoF Robotic Manipulator: Simulation & Control

This repository contains a complete MATLAB implementation for the simulation and control of a **2-Degree-of-Freedom (2DoF) planar robotic arm**. The project demonstrates the integration of robotics theory, including kinematics, nonlinear dynamics, and PID control.

---

## ✨ Key Features

* **Dynamic Modeling:** Full implementation of Euler-Lagrange equations including:
    * **Inertia Matrix ($H$)**
    * **Coriolis & Centrifugal Forces ($C$)**
    * **Gravitational Vector ($G$)**
* **Kinematics Suite:**
    * **Forward Kinematics:** Mapping joint angles to Cartesian $(x, z)$ coordinates.
    * **Inverse Kinematics:** Analytical solution (elbow-down) for target reaching.
    * **Jacobian Matrix ($J$):** Handling velocity mapping and external force-to-torque transformations.
* **Trajectory Planning:**
    * **Cubic Polynomials:** For smooth $A \to B$ point transitions.
    * **Step Reference:** For $B \to C$ instantaneous position changes.
* **Control System:** Decentralized **PD Control** with high-gain $K_p$ and $K_d$ parameters to ensure tracking precision.
* **Disturbance Rejection:** Real-time simulation of external force vectors ($F_{ext}$) acting on the end-effector.

---

## 🚀 Tech Stack

* **Software:** MATLAB
* **Solver:** `ode45` (Runge-Kutta) for non-linear system integration.
* **Concepts:** Robotics, Control Theory, Differential Equations.

---

## 🛠 Project Structure

| File | Description |
| :--- | :--- |
| `Main.m` | Central script for simulation parameters and execution. |
| `matrix_dyn.m` | Calculates dynamic matrices ($H, C, G$). |
| `matrix_kin.m` | Computes the system Jacobian matrix ($J$). |
| `int_2DoF.m` | State-space representation of the robot's dynamics. |
| `cubic.m` | 3rd-degree polynomial trajectory generator. |
| `forward_kinematics.m` | Converts joint angles to Cartesian positions. |
| `inverzna.m` | Calculates joint angles for specific $(x, z)$ targets. |
| `write_in_memory.m` | Data logging utility for post-simulation analysis. |

---

## 📊 Results & Visualization

The simulation provides four primary outputs to evaluate performance:
1.  **Joint Positions:** Tracking accuracy between reference and actual angles.
2.  **Joint Velocities:** Angular speed profiles.
3.  **Cartesian Path:** End-effector trajectory in the vertical $X-Z$ plane.
4.  **Joint Torques:** The control effort (Nm) required to move the robot and resist disturbances.

---

## ⚙️ How to Run

1.  Clone this repository to your local machine.
2.  Open MATLAB and navigate to the project folder.
3.  Execute the main simulation:
    ```matlab
    run('Main.m')
    ```
4.  The script will perform the simulation and display the 2D animation and result plots.

---

## 📝 Physical Parameters
* **Link Lengths:** $l_1 = 0.3m, l_2 = 0.3m$
* **Masses:** $m_1 = 2kg, m_2 = 2kg$
* **Control Gains:** $K_p = 2000, K_d = 400$

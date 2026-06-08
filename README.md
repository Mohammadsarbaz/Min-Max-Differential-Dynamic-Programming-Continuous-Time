# Min-Max Differential Dynamic Programming — Continuous Time

MATLAB implementation of **Min-Max Differential Dynamic Programming (Min-Max DDP)** for zero-sum differential games in continuous time. This repository provides the code associated with the following paper:

---

## 📖 Overview

Min-Max DDP is an optimal control algorithm that solves zero-sum differential games, where one player minimizes and the other maximizes a cost function. This framework is applied to nonlinear dynamical systems and provides a powerful tool for robust control design under worst-case disturbances.

---

## 📁 Repository Structure

### 1. `DDP_QuadRotor` — Standard DDP on a Quadrotor
Implementation of standard Differential Dynamic Programming (DDP) applied to a quadrotor system. This serves as a baseline comparison for the Min-Max DDP algorithm.

### 2. `Main_Inverted_Pendulum_First` — Min-Max DDP on Inverted Pendulum (without terminal cost)
Implementation of Min-Max DDP on an inverted pendulum system. The cost function does **not** include a terminal cost term.

### 3. `Main_Inverted_Pendulum_Second` — Min-Max DDP on Inverted Pendulum (with terminal cost)
Implementation of Min-Max DDP on an inverted pendulum system. The cost function **includes** a terminal cost term, demonstrating the effect of terminal cost on convergence and performance.

### 4. `Min-MaxDDP_Quad` — Min-Max DDP on a Quadrotor
Implementation of Min-Max DDP applied to a quadrotor (UAV) system, demonstrating robust control performance under adversarial disturbances.

---

## 🛠️ Requirements
- MATLAB R2020a or later
- Control System Toolbox (optional)

## 🚀 How to Run
1. Clone or download this repository
2. Open MATLAB and navigate to the desired system folder
3. Run the `main.m` file

## 📊 Results
Figures and simulation results are available in the `figures/` folder.

## 🎥 Video
Implementation video is available in the `videos/` folder.

---

## 📬 Contact
**Mohammad Sarbaz** — mohammad.sarbaz@ou.edu
🔗 [LinkedIn](https://www.linkedin.com/in/mohammad-sarbaz-94256b1b7/) | 🎓 [Google Scholar](https://scholar.google.com/citations?user=St87OnMAAAAJ&hl=en&oi=ao)

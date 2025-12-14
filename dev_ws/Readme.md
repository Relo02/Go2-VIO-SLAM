# Point-LIO — Technical Overview

📄 **Based on:**  
**Point-LIO: Robust High-Bandwidth LiDAR–Inertial Odometry**  
Advanced Intelligent Systems, 2023  
DOI: 10.1002/aisy.202200459

---

## 📌 Introduction

**Point-LIO** is a tightly coupled **LiDAR–Inertial Odometry (LIO)** framework designed for **high-bandwidth, low-latency, and aggressive motion scenarios**.  
Unlike traditional frame-based LiDAR odometry systems, Point-LIO performs **point-by-point sensor fusion**, enabling odometry outputs at **kHz-level frequency**.

This README provides a **technical summary** of the paper, focusing on:
- System architecture
- State formulation
- Continuous-time IMU propagation
- Point-wise LiDAR update
- Key mathematical models required for implementation

---

## 🧠 Core Contributions

1. **Point-by-Point LiDAR Fusion**
   - Each LiDAR point is fused at its exact timestamp
   - Eliminates intra-scan motion distortion
   - Enables ultra-high-rate odometry output (4–8 kHz)

2. **Stochastic Process–Augmented IMU Model**
   - IMU measurements are modeled as *outputs* of a stochastic process
   - Improves robustness under IMU saturation and aggressive maneuvers

3. **Iterated EKF with Continuous Updates**
   - EKF prediction and update are interleaved for each LiDAR point
   - No batch scan registration is required

---

## 📐 State Vector Definition

The system state is defined as:

$$
\mathbf{x} =
\begin{bmatrix}
\mathbf{p} \\
\mathbf{v} \\
\mathbf{q} \\
\mathbf{b}_g \\
\mathbf{b}_a
\end{bmatrix}
$$

Where:

| Symbol | Meaning |
|------|--------|
| $\mathbf{p} \in \mathbb{R}^3$ | Position |
| $\mathbf{v} \in \mathbb{R}^3$ | Velocity |
| $\mathbf{q} \in \mathbb{H}$ | Orientation (quaternion) |
| $\mathbf{b}_g \in \mathbb{R}^3$ | Gyroscope bias |
| $\mathbf{b}_a \in \mathbb{R}^3$ | Accelerometer bias |

Optionally, LiDAR–IMU extrinsics can be appended to the state.

---

## 🔄 Continuous-Time IMU Motion Model

### Position and Velocity

$$
\dot{\mathbf{p}} = \mathbf{v}
$$

$$
\dot{\mathbf{v}} =
\mathbf{R}(\mathbf{q})
(\mathbf{a}_m - \mathbf{b}_a - \mathbf{n}_a)
- \mathbf{g}
$$

### Orientation (Quaternion Kinematics)

$$
\dot{\mathbf{q}} =
\frac{1}{2}
\boldsymbol{\Omega}(\boldsymbol{\omega}_m - \mathbf{b}_g - \mathbf{n}_g)
\mathbf{q}
$$

Where:
- $\mathbf{a}_m$, $\boldsymbol{\omega}_m$ are IMU measurements
- $\mathbf{n}_a$, $\mathbf{n}_g$ are IMU noise
- $\mathbf{g}$ is gravity
- $\boldsymbol{\Omega}(\cdot)$ is the quaternion multiplication matrix

---

## 📊 Stochastic Process–Augmented IMU Model

**Key idea:**  
IMU measurements are treated as **system outputs** rather than purely noisy inputs.

$$
\mathbf{y}_{imu} = h(\mathbf{x}) + \mathbf{n}
$$

This formulation:
- Improves observability
- Maintains filter stability under sensor saturation
- Allows consistent covariance propagation

---

## 📍 Point-to-Plane LiDAR Measurement Model

Each LiDAR point is processed independently.

### Point Transformation

$$
\mathbf{p}_i^{world} =
\mathbf{R}(\mathbf{q})
(\mathbf{p}_i^{lidar} + \mathbf{t}_{L}^{I})
+ \mathbf{p}
$$

### Residual (Point-to-Plane)

$$
r_i =
\mathbf{n}^T
(\mathbf{p}_i^{world} - \mathbf{p}_{map})
$$

Where:
- $\mathbf{n}$ is the local surface normal
- $\mathbf{p}_{map}$ is the closest map point or plane projection

---

## 🔁 EKF Update (Per LiDAR Point)

### Kalman Gain

$$
\mathbf{K}_i =
\mathbf{P}^- \mathbf{H}_i^T
(\mathbf{H}_i \mathbf{P}^- \mathbf{H}_i^T + \mathbf{R})^{-1}
$$

### State Update

$$
\mathbf{x}^+ =
\mathbf{x}^- + \mathbf{K}_i r_i
$$

### Covariance Update

$$
\mathbf{P}^+ =
(\mathbf{I} - \mathbf{K}_i \mathbf{H}_i)\mathbf{P}^-
$$

This update is executed **for every LiDAR point**, not per scan.

---

## 🗺 Incremental Mapping

- A KD-tree stores map points
- Only new, non-redundant points are added
- A sliding local map window is maintained to bound computation

---

## ⚡ Advantages Over Frame-Based LIO

| Feature | Frame-Based LIO | Point-LIO |
|------|---------------|-----------|
| Motion distortion | Post-correction | Eliminated |
| Update frequency | 10–100 Hz | 4–8 kHz |
| Latency | High | Very low |
| Aggressive motion | Fragile | Robust |

---

## 🧪 Experimental Highlights (from the paper)

- Odometry output: **kHz-level**
- Angular velocity tolerance: **~75 rad/s**
- Robust to IMU saturation
- Real-time capable on embedded hardware

---

## 📌 Key Takeaways for Developers

- Fuse **each LiDAR point at its timestamp**
- Use **continuous-time IMU propagation**
- Apply **iterated EKF updates**
- Maintain a **local incremental map**
- Treat IMU as a **stochastic process output**
*

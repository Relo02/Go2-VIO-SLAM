# Point-LIO — Documentation

📄 **Paper**  
**Point-LIO: Robust High-Bandwidth LiDAR–Inertial Odometry**  
Advanced Intelligent Systems, 2023  
DOI: 10.1002/aisy.202200459  

---

## 1. Overview

**Point-LIO** is a **tightly coupled LiDAR–IMU odometry** system that performs **continuous-time, point-by-point sensor fusion** using an **Iterated Extended Kalman Filter (IEKF)**.

### Key Idea
> Instead of processing LiDAR scans as discrete frames, **each LiDAR point is fused at its exact timestamp**, interleaving IMU propagation and LiDAR updates.

This enables:
- kHz-level odometry
- zero motion distortion
- robustness under aggressive motion and IMU saturation

---

## 2. Coordinate Frames

| Frame | Description |
|-----|------------|
| **W** | World frame |
| **I** | IMU body frame |
| **L** | LiDAR frame |

Extrinsics:
$$
\[
\mathbf{p}_L^I,\; \mathbf{R}_L^I
\]
$$
---

## 3. State Vector Definition

The system state at time \( t \) is:

\[
\mathbf{x}(t) =
\begin{bmatrix}
\mathbf{p}(t) \\
\mathbf{v}(t) \\
\mathbf{q}(t) \\
\mathbf{b}_g(t) \\
\mathbf{b}_a(t)
\end{bmatrix}
\]

Where:

| Symbol | Dimension | Meaning |
|-----|---------|--------|
| \( \mathbf{p} \) | \( \mathbb{R}^3 \) | Position |
| \( \mathbf{v} \) | \( \mathbb{R}^3 \) | Velocity |
| \( \mathbf{q} \) | \( \mathbb{H} \) | Orientation (quaternion) |
| \( \mathbf{b}_g \) | \( \mathbb{R}^3 \) | Gyro bias |
| \( \mathbf{b}_a \) | \( \mathbb{R}^3 \) | Acc bias |

Optional:
- LiDAR–IMU extrinsics
- Gravity vector

---

## 4. Continuous-Time IMU Process Model

### 4.1 Kinematics

\[
\dot{\mathbf{p}} = \mathbf{v}
\]

\[
\dot{\mathbf{v}} =
\mathbf{R}(\mathbf{q})
(\mathbf{a}_m - \mathbf{b}_a - \mathbf{n}_a)
- \mathbf{g}
\]

\[
\dot{\mathbf{q}} =
\frac{1}{2}
\boldsymbol{\Omega}(\boldsymbol{\omega}_m - \mathbf{b}_g - \mathbf{n}_g)
\mathbf{q}
\]

\[
\dot{\mathbf{b}}_g = \mathbf{n}_{bg},
\quad
\dot{\mathbf{b}}_a = \mathbf{n}_{ba}
\]

Where:
- \( \boldsymbol{\omega}_m \): gyro measurement
- \( \mathbf{a}_m \): accelerometer measurement
- \( \mathbf{n}_* \): white Gaussian noise
- \( \mathbf{g} \): gravity

---

## 5. Stochastic Process-Augmented IMU Model (Core Innovation)

Instead of treating IMU purely as input, Point-LIO models IMU readings as **stochastic system outputs**:

\[
\mathbf{y}_{imu} =
\begin{bmatrix}
\boldsymbol{\omega}_m \\
\mathbf{a}_m
\end{bmatrix}
=
h_{imu}(\mathbf{x}) + \mathbf{n}
\]

This formulation:
- Preserves observability
- Maintains filter stability
- Handles IMU saturation gracefully

---

## 6. LiDAR Point Time Encoding

Each LiDAR point \( i \) has timestamp:

\[
t_i = t_{scan} + \Delta t_i
\]

Where:
- \( \Delta t_i \) is encoded in the point (e.g., intensity / curvature)

This enables **true continuous-time fusion**.

---

## 7. Point-Wise State Propagation

For each LiDAR point \( i \):

1. **Propagate IMU** from \( t_{i-1} \to t_i \):

\[
\mathbf{x}_{i|i-1} = f(\mathbf{x}_{i-1}, \mathbf{u}_{imu})
\]

\[
\mathbf{P}_{i|i-1} =
\mathbf{F}_i \mathbf{P}_{i-1} \mathbf{F}_i^T + \mathbf{Q}_i
\]

---

## 8. LiDAR Measurement Model

### 8.1 Point Transformation

\[
\mathbf{p}_i^W =
\mathbf{R}(\mathbf{q})
(\mathbf{R}_L^I \mathbf{p}_i^L + \mathbf{p}_L^I)
+ \mathbf{p}
\]

---

### 8.2 Point-to-Plane Residual

Let:
- \( \mathbf{p}_{map} \): closest surface point
- \( \mathbf{n} \): surface normal

Residual:

\[
r_i =
\mathbf{n}^T
(\mathbf{p}_i^W - \mathbf{p}_{map})
\]

---

## 9. Linearization (Jacobian)

\[
r_i \approx
r_i(\hat{\mathbf{x}}) +
\mathbf{H}_i \delta \mathbf{x}
\]

Where:

\[
\mathbf{H}_i =
\frac{\partial r_i}{\partial \mathbf{x}}
\]

Includes derivatives w.r.t.:
- position
- orientation
- (optionally) extrinsics

---

## 10. Iterated EKF Update (Per Point)

### 10.1 Kalman Gain

\[
\mathbf{K}_i =
\mathbf{P}_{i|i-1} \mathbf{H}_i^T
(\mathbf{H}_i \mathbf{P}_{i|i-1} \mathbf{H}_i^T + \mathbf{R})^{-1}
\]

---

### 10.2 State Update

\[
\delta \mathbf{x}_i =
\mathbf{K}_i r_i
\]

\[
\mathbf{x}_i =
\mathbf{x}_{i|i-1} \oplus \delta \mathbf{x}_i
\]

Quaternion update uses exponential map:

\[
\mathbf{q} \leftarrow
\mathbf{q} \otimes \exp(\tfrac{1}{2}\delta \boldsymbol{\theta})
\]

---

### 10.3 Covariance Update

\[
\mathbf{P}_i =
(\mathbf{I} - \mathbf{K}_i \mathbf{H}_i)
\mathbf{P}_{i|i-1}
\]

---

## 11. Incremental Mapping

After updating state:

1. Transform downsampled points to world
2. Insert into KD-tree map
3. Maintain a **sliding local map window**

This ensures:
- bounded memory
- fast nearest-neighbor search

---

## 12. Full Algorithm Summary

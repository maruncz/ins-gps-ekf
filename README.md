# INS-GPS-EKF

A C++17 static library implementing a **15-dimensional Error-State Extended Kalman Filter** for fusing Inertial Navigation System (INS) data with GPS measurements. The filter estimates and corrects position, velocity, and orientation errors in real time, with optional GPS-derived azimuth alignment.

---

## Overview

An Inertial Navigation System computes position and velocity by integrating accelerometer and gyroscope measurements. This process is autonomous and high-rate, but it **drifts over time** because sensor biases and noise accumulate through double-integration.

GPS provides absolute position and velocity at a lower rate (~1-10 Hz) with bounded errors, but is subject to outages and multipath.

This library fuses both sources using an **Error-State (Indirect) Extended Kalman Filter**. Instead of estimating the full navigation state directly, the EKF tracks the *errors* in the INS solution. When a GPS fix arrives, the filter computes optimal corrections and resets. This indirect formulation keeps the error state small (linearisation stays accurate) and lets the INS run undisturbed between GPS updates.

---

## Mathematical Formulation

### Coordinate Frames

| Frame | Description |
|-------|-------------|
| **Navigation frame (n)** | Local NED (North-East-Down) tangent plane |
| **Body frame (b)** | Sensor-fixed frame aligned with the IMU |

The Direction Cosine Matrix (DCM) $T_b^n$ transforms vectors from the body frame to the navigation frame.

#### WGS-84 Earth Model

The Earth is modelled as an ellipsoid with:

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Semi-major axis | $R_e$ | 6,378,137.0 m |
| Eccentricity | $e$ | 0.081819 |
| Turn rate | $\omega_{ei}$ | 7.292 &times; 10<sup>-5</sup> rad/s |

The meridian and prime-vertical radii of curvature are:

$$R_M(\phi) = \frac{R_e(1 - e^2)}{(1 - e^2 \sin^2\phi)^{3/2}}$$

$$R_N(\phi) = \frac{R_e}{\sqrt{1 - e^2 \sin^2\phi}}$$

where $\phi$ is geodetic latitude.

---

### State Vector

The 15-dimensional error state vector is:

$$\delta \mathbf{x} = \begin{bmatrix} \delta \mathbf{p}^n \\ \delta \mathbf{v}^n \\ \boldsymbol{\varepsilon}^n \\ \mathbf{b}_a \\ \mathbf{b}_g \end{bmatrix} \in \mathbb{R}^{15}$$

| Index | Symbol | Dimension | Description |
|-------|--------|-----------|-------------|
| 0-2 | $\delta \mathbf{p}^n$ | 3 | Position error (latitude, longitude, altitude) |
| 3-5 | $\delta \mathbf{v}^n$ | 3 | Velocity error in NED (m/s) |
| 6-8 | $\boldsymbol{\varepsilon}^n$ | 3 | Attitude error / misalignment angles (rad) |
| 9-11 | $\mathbf{b}_a$ | 3 | Accelerometer bias (m/s<sup>2</sup>) |
| 12-14 | $\mathbf{b}_g$ | 3 | Gyroscope bias (rad/s) |

---

### INS Mechanization (Strapdown Equations)

The navigation state is propagated using strapdown mechanization equations in the NED frame:

**Position rate:**

$$\dot{\mathbf{p}}^n = \mathbf{D} \, \mathbf{v}^n$$

where $\mathbf{D}$ converts NED velocities to geodetic rates:

$$\mathbf{D} = \begin{bmatrix} \frac{1}{R_M + h} & 0 & 0 \\ 0 & \frac{1}{(R_N + h)\cos\phi} & 0 \\ 0 & 0 & -1 \end{bmatrix}$$

**Velocity rate:**

$$\dot{\mathbf{v}}^n = T_b^n \mathbf{f}^b + \mathbf{g}^n - (\Omega_{ne}^n + 2\Omega_{ei}^n) \mathbf{v}^n$$

where $\mathbf{f}^b$ is the specific force from the accelerometer, $\mathbf{g}^n = [0, 0, g]^T$ is the gravity vector, and $\Omega_{ne}^n$, $\Omega_{ei}^n$ are skew-symmetric matrices of the transport rate and Earth rotation rate respectively.

**DCM rate:**

$$\dot{T}_b^n = T_b^n \Omega_{bi}^b - (\Omega_{ei}^n + \Omega_{ne}^n) T_b^n$$

where $\Omega_{bi}^b$ is the skew-symmetric matrix of the gyroscope measurements.

All states are integrated using first-order Euler integration:

$$\mathbf{x}(t + \Delta t) = \mathbf{x}(t) + \Delta t \cdot \dot{\mathbf{x}}(t)$$

---

### Error State Dynamics

The error state evolves according to:

$$\delta \dot{\mathbf{x}} = \mathbf{F} \, \delta \mathbf{x} + \mathbf{G} \, \mathbf{w}$$

where $\mathbf{F}$ is the $15 \times 15$ system dynamics matrix with the following block structure:

$$\mathbf{F} = \begin{bmatrix} F_{rr} & F_{rv} & F_{re} & \mathbf{0} & \mathbf{0} \\ F_{vr} & F_{vv} & F_{ve} & T_b^n & \mathbf{0} \\ F_{er} & F_{ev} & F_{ee} & \mathbf{0} & T_b^n \\ \mathbf{0} & \mathbf{0} & \mathbf{0} & F_a & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & F_g \end{bmatrix}$$

Each $3 \times 3$ sub-block captures a specific coupling:

| Block | Coupling | Description |
|-------|----------|-------------|
| $F_{rr}$ | position &rarr; position | Effect of position error on position rate (curvature terms) |
| $F_{rv}$ | velocity &rarr; position | $\mathbf{D}$ matrix — velocity errors drive position errors |
| $F_{re}$ | attitude &rarr; position | Zero (attitude errors do not directly affect position rate) |
| $F_{vr}$ | position &rarr; velocity | Coriolis and centripetal terms that depend on latitude |
| $F_{vv}$ | velocity &rarr; velocity | Coriolis coupling between velocity components |
| $F_{ve}$ | attitude &rarr; velocity | Skew-symmetric of specific force: $-[T_b^n \mathbf{f}^b \times]$ |
| $F_{er}$ | position &rarr; attitude | Earth-rate and transport-rate dependence on position |
| $F_{ev}$ | velocity &rarr; attitude | Transport-rate dependence on velocity |
| $F_{ee}$ | attitude &rarr; attitude | Skew-symmetric of angular velocity: $-[T_b^n \boldsymbol{\omega}^b \times]$ |
| $F_a$ | accel. bias dynamics | Zero (random walk model) |
| $F_g$ | gyro bias dynamics | Zero (random walk model) |

---

### Prediction Step

The state transition matrix is computed via matrix exponential:

$$\Phi = e^{\mathbf{F} \Delta t}$$

The error state covariance is propagated as:

$$\mathbf{P}_{k+1}^{-} = \Phi \, \mathbf{P}_k \, \Phi^T + \mathbf{Q}$$

where $\mathbf{Q}$ is the process noise covariance matrix (15 &times; 15 diagonal).

---

### GPS Update Step

When GPS measurements arrive, the measurement vector is:

$$\mathbf{z} = \begin{bmatrix} \mathbf{p}_{INS}^n \\ \mathbf{v}_{INS}^n \end{bmatrix} - \begin{bmatrix} \mathbf{p}_{GPS}^n \\ \mathbf{v}_{GPS}^n \end{bmatrix}$$

The observation matrix maps the error state to the measurement space:

$$\mathbf{H} = \begin{bmatrix} \mathbf{I}_{6 \times 6} & \mathbf{0}_{6 \times 9} \end{bmatrix}$$

**Kalman gain:**

$$\mathbf{K} = \mathbf{P}^{-} \mathbf{H}^T (\mathbf{H} \mathbf{P}^{-} \mathbf{H}^T + \mathbf{R})^{-1}$$

**State correction:**

$$\delta \hat{\mathbf{x}} = \delta \mathbf{x}^{-} + \mathbf{K}(\mathbf{z} - \mathbf{H} \, \delta \mathbf{x}^{-})$$

**Covariance update:**

$$\mathbf{P}^{+} = (\mathbf{I} - \mathbf{K} \mathbf{H}) \mathbf{P}^{-}$$

**Navigation state correction:**

$$\mathbf{p}_{corrected}^n = \mathbf{p}_{INS}^n - \delta \hat{\mathbf{p}}^n$$

$$\mathbf{v}_{corrected}^n = \mathbf{v}_{INS}^n - \delta \hat{\mathbf{v}}^n$$

$$T_{corrected} = (\mathbf{I} - [\boldsymbol{\varepsilon}^n \times]) \, T_{INS}$$

**Reset:** After correction, the error state is reset to zero, the navigation state is set to the corrected values, and the covariance is reset.

---

### Azimuth Alignment

When horizontal velocity exceeds a threshold (5 m/s), the filter computes a GPS-derived heading correction:

$$\psi_{GPS} = \text{atan2}(v_E, v_N)$$

$$z_{heading} = \psi_{INS} - \psi_{GPS}$$

During the error state integration, the down-axis attitude error $\varepsilon_d$ is replaced with a value derived from the acceleration changes:

$$\varepsilon_d = -\frac{\delta\dot{v}_E \, f_N - \delta\dot{v}_N \, f_E + g(\varepsilon_N f_N + \varepsilon_E f_E)}{f_N^2 + f_E^2}$$

At GPS update, the orientation correction uses Euler angle subtraction:

$$T_{corrected} = R(\boldsymbol{\theta}_{INS} - \boldsymbol{\varepsilon})$$

where $\varepsilon_d$ is set to $z_{heading}$.

---

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                        EKF                          │
│  (public API, GPS update, Kalman correction, mutex) │
└──────────────┬──────────────────────────────────────┘
               │ owns (unique_ptr)
               ▼
┌─────────────────────────────────────────────────────┐
│                     Tracking                        │
│  (IMU buffering, dt computation, orchestration)     │
└──┬──────────────────┬──────────────────┬────────────┘
   │ shared_ptr       │ shared_ptr       │ shared_ptr
   ▼                  ▼                  ▼
┌──────────┐  ┌─────────────┐  ┌─────────────────────┐
│Navigation│  │  ErrorState  │  │ErrorStateCovariance  │
│  State   │  │ (15D, F mat) │  │  (P, Q, Phi)        │
└──────────┘  └─────────────┘  └─────────────────────┘
                    │ uses
                    ▼
              ┌───────────┐
              │   Utils   │
              │ (WGS-84,  │
              │  Rm, Rn,  │
              │  skew-sym)│
              └───────────┘
```

### Data Flow

```
 IMU (accel)──► updateWithInertialMeasurement(data, ACCELEROMETER)
                          │
 IMU (gyro) ──► updateWithInertialMeasurement(data, GYRO)
                          │
                          ▼
                  Tracking::checkAndUpdate()
                     (when both accel & gyro available)
                          │
                  ┌───────┴────────┐
                  │  Compute dt    │
                  │  Mean of       │
                  │  buffered IMU  │
                  └───────┬────────┘
                          │
            ┌─────────────┼─────────────┐
            ▼             ▼             ▼
     NavigationState  ErrorState  ErrorStateCovariance
      propagate()    propagate()    P = Φ P Φᵀ + Q
            │             │             │
            └─────────────┼─────────────┘
                          │
 GPS ─────► updateWithGPSMeasurements(gps_data)
                          │
                  ┌───────┴────────┐
                  │  Compute z     │
                  │  Kalman gain K │
                  │  Correct state │
                  │  Reset errors  │
                  └────────────────┘
```

---

## API Reference

### `EKF_INS::EKF`

Main filter class. All public methods are thread-safe for reading state.

| Method | Description |
|--------|-------------|
| `EKF()` | Constructor. Initializes default Q and R matrices, logging, and internal components. |
| `void setInitialState(Vector3d p_0, Vector3d v_0, Matrix3d T_0)` | Set initial navigation state. `p_0` = [latitude (rad), longitude (rad), altitude (m)], `v_0` = NED velocity (m/s), `T_0` = body-to-nav DCM. |
| `void start()` | Start the filter (resets the clock). Must be called before feeding measurements. |
| `void updateWithInertialMeasurement(Vector3d data, Type type)` | Feed an IMU sample. `type` is `ACCELEROMETER` (m/s<sup>2</sup>, body frame) or `GYRO` (rad/s, body frame). |
| `void updateWithGPSMeasurements(vector<Matrix<double,6,1>> gps_data)` | Feed one or more GPS fixes. Each 6&times;1 vector: [lat (rad), lon (rad), alt (m), v_N, v_E, v_D] (m/s). Multiple fixes are averaged. |
| `void setQMatrix(const MatrixXd &Q)` | Set process noise covariance (15&times;15). |
| `void setRMatrix(const MatrixXd &R)` | Set measurement noise covariance (6&times;6). |
| `tuple<Vector3d, Vector3d, Matrix3d> getNavigationState()` | Returns (position, velocity, DCM). |
| `Vector3d getPositionState()` | Returns position [lat, lon, alt]. |
| `Vector3d getVelocityState()` | Returns NED velocity. |
| `Matrix3d getOrientationState()` | Returns body-to-nav DCM. |
| `VectorXd getErrorState()` | Returns the 15D error state. |
| `MatrixXd getErrorStateCovariance()` | Returns 15&times;15 covariance matrix. |
| `double getAzimuth()` | Returns current heading (rad). |

### `EKF_INS::Type`

Enum: `ACCELEROMETER`, `GYRO`.

### `EKF_INS::Utils`

Static utility class with WGS-84 constants and helper functions:

| Member | Description |
|--------|-------------|
| `Rm(phi)` | Meridian radius of curvature at latitude `phi` (rad) |
| `Rn(phi)` | Prime-vertical radius of curvature at latitude `phi` (rad) |
| `toSkewSymmetricMatrix(M, v)` | Fills 3&times;3 matrix `M` with the skew-symmetric form of vector `v` |
| `toEulerAngles(T)` | Extract Euler angles (roll, pitch, yaw) from rotation matrix |
| `toRotationMatrix(euler)` | Build rotation matrix from Euler angles |
| `fromENUtoNED(enu)` | Coordinate transform (note: uses a non-standard transformation matrix) |
| `calcMeanVector(measurements)` | Online (incremental) mean of a vector of Eigen vectors |
| `degreeToRadian(x)` / `radianToDegree(x)` | Angle conversion |

---

## Usage Example

```cpp
#include "EKF.h"

int main() {
    EKF_INS::EKF ekf;

    // Set initial state: position (lat, lon, alt), velocity (NED), orientation (DCM)
    Eigen::Vector3d p0(0.5585, 0.6109, 100.0);  // ~32 deg N, ~35 deg E, 100m
    Eigen::Vector3d v0(0.0, 0.0, 0.0);
    Eigen::Matrix3d T0 = Eigen::Matrix3d::Identity();

    ekf.setInitialState(p0, v0, T0);
    ekf.start();

    // Main loop: feed IMU data at high rate
    Eigen::Vector3d accel_body(0.0, 0.0, -9.81);  // body-frame specific force
    Eigen::Vector3d gyro_body(0.0, 0.0, 0.0);      // body-frame angular rate

    ekf.updateWithInertialMeasurement(accel_body, EKF_INS::ACCELEROMETER);
    ekf.updateWithInertialMeasurement(gyro_body, EKF_INS::GYRO);

    // When GPS is available: feed position + velocity
    std::vector<Eigen::Matrix<double, 6, 1>> gps_data;
    Eigen::Matrix<double, 6, 1> gps_fix;
    gps_fix << 0.5585, 0.6109, 100.0, 0.0, 0.0, 0.0;
    gps_data.push_back(gps_fix);
    ekf.updateWithGPSMeasurements(gps_data);

    // Read corrected state
    auto [pos, vel, dcm] = ekf.getNavigationState();

    return 0;
}
```

---

## Build Instructions

### Dependencies

- **C++17** compiler (GCC 7+, Clang 5+)
- **Eigen3** — linear algebra
- **Boost** — system component
- **spdlog** — logging (bundled in `3rdparty/`)

### Build

```bash
mkdir build && cd build
cmake ..
make
```

This produces a static library `libEKF-INS.a`. Link it into your project along with the Eigen3 and Boost include paths.

### CMake Integration

```cmake
add_subdirectory(path/to/ins-gps-ekf)
target_link_libraries(your_target EKF-INS)
```

---

## References

1. **Groves, P.D.** *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed., Artech House, 2013.
2. **Titterton, D. & Weston, J.** *Strapdown Inertial Navigation Technology*, 2nd ed., IET, 2004.

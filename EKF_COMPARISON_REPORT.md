# INS-GPS-EKF: Design Analysis & Comparison Report

> **Date**: 2026-05-26
> **Scope**: Comparative analysis of this repository's 15-state Error-State EKF against PX4 EKF2 and ArduPilot EKF3, with a focus on land vehicle sensor fusion.
> **Reference Implementations**: PX4-Autopilot @ `94580ab1`, ArduPilot @ `c979b1c8d`

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Repository Overview](#2-repository-overview)
3. [State Vector Design](#3-state-vector-design)
4. [Formulation: Error-State vs Total-State](#4-formulation)
5. [Sensor Fusion Architecture](#5-sensor-fusion-architecture)
6. [Covariance and Process Noise Modeling](#6-covariance-and-process-noise)
7. [Implementation Oddities and Non-Standard Practices](#7-implementation-oddities)
8. [Time-Horizon and Latency Handling](#8-time-horizon)
9. [Initialization and Alignment](#9-initialization)
10. [GPS Outage and Re-acquisition](#10-gps-outage)
11. [Outlier Rejection and Fault Detection](#11-outlier-rejection)
12. [Land Vehicle Readiness Assessment](#12-land-vehicle-readiness)
13. [Recommended Improvements](#13-recommended-improvements)

---

## 1. Executive Summary

This repository implements a **15-dimensional Error-State Extended Kalman Filter (ES-EKF)** for inertial navigation system (INS) and GPS fusion. It follows the textbook formulations of Groves (2013) and Titterton & Weston (2004) closely, with a clean C++17 architecture separating NavigationState, ErrorState, and ErrorStateCovariance into distinct classes.

**Key finding: The filter is a correct textbook implementation but has several gaps that make it unsuitable for production land-vehicle use without significant modifications.** The three most critical issues are:

1. **No magnetometer fusion** — heading is only observable when GPS velocity >5 m/s, leaving the filter without heading at low speeds or when stationary
2. **No non-holonomic constraints** — the filter does not exploit the fact that a wheeled vehicle cannot slide sideways, allowing unphysical velocity estimates
3. **Non-standard noise injection** — uniform random noise is added directly to error states outside the EKF covariance framework, making the filter's uncertainty reporting inaccurate

Compared to PX4 EKF2 (24 states, error-state formulation, SymForce-generated Jacobians) and ArduPilot EKF3 (24 states, total-state formulation, wheel encoder fusion, multi-core architecture), this implementation is roughly **10-15 years behind production autopilot EKF capability** for land vehicle applications, though its core INS mechanization is mathematically sound.

---

## 2. Repository Overview

### Architecture

```
EKF  (public API, GPS update, Kalman correction, mutex)
 │
 └── Tracking  (IMU buffering, dt computation, orchestration)
      ├── NavigationState  (strapdown mechanization, Euler integration)
      ├── ErrorState  (15D error state, F matrix, random noise injection)
      └── ErrorStateCovariance  (P, Q, Phi propagation)
```

### Current Sensor Support

| Sensor | Status | Implementation |
|--------|--------|----------------|
| IMU (accelerometer + gyroscope) | ✅ Fully implemented | Strapdown mechanization in NED frame |
| GPS position + velocity | ✅ Fully implemented | 6-DOF measurement update |
| GPS heading (velocity-based) | ✅ Partial | Azimuth alignment, threshold >5 m/s |

### Missing Sensors

- Magnetometer / compass
- Barometric pressure
- Wheel encoder / odometry
- Optical flow
- Visual-inertial odometry
- Range finder
- Airspeed

---

## 3. State Vector Design

### Current State Vector (15 states)

```
δx = [δp^n(3), δv^n(3), ε^n(3), b_a(3), b_g(3)]^T
```

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0-2 | δp^n | Position error (lat, lon, alt) | rad, rad, m |
| 3-5 | δv^n | NED velocity error | m/s |
| 6-8 | ε^n | Attitude error (misalignment angles) | rad |
| 9-11 | b_a | Accelerometer bias | m/s² |
| 12-14 | b_g | Gyroscope bias | rad/s |

### Comparison with Production Implementations

| State Group | This Repo | PX4 EKF2 | ArduPilot EKF3 | Notes |
|-------------|-----------|-----------|----------------|-------|
| Attitude | 3 error angles (ε) | 4 quaternion + 3 error | 4 quaternion | PX4 uses SO(3) tangent space |
| Position | 3 | 3 | 3 | PX4/EKF3 use local NED, this repo uses geodetic |
| Velocity | 3 | 3 | 3 | — |
| Gyro bias | 3 | 3 | 3 | — |
| Accel bias | 3 | 3 | 3 | — |
| Mag Earth | ❌ | 3 (optional) | 3 | **Critical gap for heading** |
| Mag Body | ❌ | 3 (optional) | 3 | **Critical gap for heading** |
| Wind velocity | ❌ | 2 (optional) | 2 | Less relevant for land vehicles |
| Terrain | ❌ | 1 (optional) | ❌ | Not essential |
| Gyro scale factor | ❌ | ❌ | 0 (EKF2 had 3) | EKF3 dropped this |
| **Total DOF** | **15** | **24** (configurable) | **24** | |

### Impact Assessment

**The 15-state vector covers the core kinematic states but has two major omissions:**

1. **No magnetic field states** — Without magnetometer fusion, the filter cannot determine heading without GPS velocity, which requires movement. For land vehicles, this means heading is unavailable at startup, in stop-and-go traffic, and in parking lots.

2. **No position error states for lever arm / antenna offset** — The GPS antenna is assumed to be at the IMU origin. For land vehicles, this lever arm can be significant (~1-2 m) and introduces velocity errors during turns.

---

## 4. Formulation: Error-State vs Total-State

### This Repository: Error-State (Indirect)

The filter tracks errors in the INS solution rather than the states directly. When a GPS fix arrives, errors are computed, the navigation state is corrected, and the error state is reset to zero.

```
Between GPS updates:  δx propagates via F matrix
At GPS update:        x_corrected = x_INS - δx
                      δx = 0 (reset)
```

**Advantages**: Good linearity (errors stay small), clean separation of INS and filter.

**Disadvantage**: Hard reset discards accumulated error dynamics — the error state is zeroed after every correction, losing any "memory" of how errors evolve.

### PX4 EKF2: Error-State (Modern, Continuously Coupled)

PX4's error-state formulation (adopted in late 2023, PR #22262) is based on Sola's 2017 paper *"Quaternion kinematics for the error-state Kalman filter"*. The error state is a perturbation on the tangent space of SO(3):

```
δq ≈ [1, θ/2]^T    (quaternion perturbation)
x_nominal = f(x_nominal, u)    (continuous propagation)
δx injected into nominal state at each step
```

This is mathematically more principled than this repo's approach because:
- The error state is never "reset" — it's continuously reinjected
- The quaternion is maintained on the SO(3) manifold (not in Euler angles)
- Covariance is defined in the tangent space, matching the error state dimension (not the quaternion's 4D space)

### ArduPilot EKF3: Total-State (Direct)

The filter states ARE the navigation states. The quaternion, velocity, and position are propagated and updated directly:

```
x = f(x, u)  (continuous propagation)
x += K(z - Hx)  (direct correction)
```

ArduPilot EKF2 (predecessor) used error-state. EKF3 switched to total-state for simplicity and because the error-state advantages were less important with proper covariance tuning.

### Key Distinction

This repo's approach is closest to PX4 in philosophy (both error-state), but the reset mechanism is fundamentally different:
- **This repo**: Hard reset (δx → 0 after each update), loses error dynamics
- **PX4**: Soft injection (error continuously applied to nominal), preserves consistency
- **ArduPilot EKF3**: No error state at all

---

## 5. Sensor Fusion Architecture

### This Repository: Single-Threaded, Sequential

```
IMU data ──► buffer until both accel+gyro received
           ──► compute mean of buffered measurements
           ──► propagate NavigationState + ErrorState + Covariance
GPS data  ──► mean of multiple fixes (if provided)
           ──► compute innovation z = x_INS - x_GPS
           ──► chi-square gate (6-DOF, single threshold 18.0)
           ──► Kalman gain → correction → reset
```

**Key characteristics:**
- No per-sensor time synchronization
- GPS measurement time is assumed to match current INS time
- Out-of-sequence measurements are crudely handled by averaging
- Single measurement model: H = [I₆, 0₆×₉]

### PX4 EKF2: Delayed Horizon, Per-Sensor FIFO Buffers

```cpp
// Each sensor has its own TimestampedRingBuffer
_gps_buffer.push(gps_sample, time_us - EKF2_GPS_DELAY);
_mag_buffer.push(mag_sample, time_us - EKF2_MAG_DELAY);
_baro_buffer.push(baro_sample, time_us - EKF2_BARO_DELAY);

// EKF operates on delayed time horizon
imu_sample_delayed = _imu_buffer.get_oldest();
gps_sample_delayed = _gps_buffer.pop_first_older_than(imu_delayed.time_us);

// Output predictor propagates to current time
_output_predictor.calculateOutputStates(current_time);
```

### ArduPilot EKF3: Similar Delayed Horizon

```cpp
// obs_buffer per sensor type, IMU buffer separately
// Fusion order matters (defined in controlFusionModes):
//   1. IMU prediction
//   2. Mag fusion
//   3. GPS vel/pos fusion
//   4. Body odometry (wheel encoder / VO)
//   5. Airspeed / sideslip
//   6. Drag
```

### Impact for Land Vehicles

The repo's current-time fusion creates a subtle but important error: when GPS has nonzero latency (typically 100-200ms), the measurement is fused against a state that has moved forward in time. During turns or acceleration, this introduces systematic errors:

| Vehicle speed | 200ms GPS delay | Position error |
|---------------|-----------------|----------------|
| 10 m/s (36 km/h) | 200ms | 2.0 m |
| 20 m/s (72 km/h) | 200ms | 4.0 m |
| During turn (0.3 rad/s) | 200ms | ~2° heading error |

The delayed-horizon architecture used by both PX4 and ArduPilot eliminates this by running the EKF at a fixed delay and using an output predictor for real-time estimates. This is a significant architectural difference.

---

## 6. Covariance and Process Noise

### This Repository: Prediction

**ErrorStateCovariance::updateCovarianceMatrix():**
```cpp
Eigen::MatrixXd phi = error_state_ptr_->getTransitionMatrix(dt);
P_ = phi * P_ * phi.transpose() + Q_ * dt;
```

**ErrorState::getTransitionMatrix():**
```cpp
Eigen::MatrixXd coeff = F() * dt;
return coeff.exp();  // matrix exponential
```

**Notes:**
- `Q_ * dt` is a first-order approximation of the discrete-time process noise. Standard practice is `Q_d = G * Q_c * G^T * dt` or the full Van Loan discretization.
- The matrix exponential of `F * dt` is correct but computationally expensive. Many implementations use a Taylor series approximation (first-order: `I + F*dt`) for small dt.
- `P` is initialized as a zero matrix, meaning the filter starts with infinite confidence in its initial state.

### PX4 EKF2: SymForce-Generated

```cpp
// Auto-generated from symbolic derivation
P = sym::PredictCovariance(_state.vector(), P,
    imu_delayed.delta_vel / imu_delayed.delta_vel_dt, accel_var,
    imu_delayed.delta_ang / imu_delayed.delta_ang_dt, gyro_var, dt);
```

PX4 uses **SymForce** to symbolically derive the covariance prediction Jacobians, generating C++ code that is verified, fast, and mathematically exact. The derivation is in discrete-time directly, avoiding continuous→discrete approximation errors.

### ArduPilot EKF3: Numerical Jacobians

EKF3 hand-derives the state transition and uses numerical checks for stability. The covariance prediction (`CovariancePrediction()`) handles:
- Gyro/accel process noise from spectral densities
- Wind process noise scaled by vertical velocity
- Accelerometer clipping detection (increases noise when sensors saturate)
- Heading unobservability handling (zeroes heading correlations when not observable)
- Bias variance limiting to prevent filter divergence

### Key Difference: Process Noise Structure

| Aspect | This Repo | PX4 EKF2 | ArduPilot EKF3 |
|--------|-----------|----------|----------------|
| Discretization | Q × dt (first-order) | SymForce symbolic (exact) | Van Loan / numerical |
| Q matrix | Diagonal 15×15 | Structured (G Q G^T) | Per-state spectral density |
| Adaptive tuning | ❌ | Clipping detection, wind scaling | Clipping, velocity-scaling, bias limiting |
| Covariance init | Zero matrix (∞ confidence) | Parameter-driven (EKF2_*_INIT) | Parameter-driven |
| Joseph form | ❌ (standard KF) | ✅ (Joseph stabilized) | ✅ (Joseph form) |

---

## 7. Implementation Oddities and Non-Standard Practices

This section documents design decisions that deviate from standard EKF practice, production-grade implementations, or the cited references (Groves, Titterton & Weston).

### 7.1 Random Noise Injection into Error State Dynamics

**File**: `ErrorState.cpp`, lines 165-187

```cpp
Eigen::Vector3d ErrorState::omega_a() {
    Eigen::Vector3d omega_a_(dis_(gen_), dis_(gen_), dis_(gen_));
    return omega_a_max_ * omega_a_;
}
Eigen::Vector3d ErrorState::omega_g() {
    Eigen::Vector3d omega_g_(dis_(gen_), dis_(gen_), dis_(gen_));
    return omega_g_max_ * omega_g_;
}
```

These are used in `updateStateWithMeasurements()` (line 38-41):

```cpp
delta_v_dot_n_ = ... + T_bn_ * b_a_ + T_bn_ * omega_a();
epsilon_dot_n_ = ... + T_bn_ * b_g_ + T_bn_ * omega_g();
```

**Problem**: Uniform random noise is added directly to the error state time derivatives. This is NOT standard EKF practice. In a standard EKF:
- Process noise is modeled through the covariance prediction: `P = F P F^T + Q`
- The state itself propagates deterministically (zero-mean noise)
- If non-zero noise injection were desired, it would be Gaussian (not uniform), and it would still affect the covariance

**Consequences**:
1. The actual error state contains random components that the covariance `P` does not account for
2. The Kalman gain calculation is based on an incorrect P, making the filter either under- or over-confident
3. The `omega_a_max_` and `omega_g_max_` parameters (default 0.0) are essentially tuning knobs that control uncontrolled random perturbations
4. This makes the filter non-deterministic — repeated runs with identical data will produce different results

**Neither PX4 EKF2 nor ArduPilot EKF3 does this.** Both propagate the error state/covariance deterministically through the standard EKF equations.

### 7.2 Bias Random Walk via uniform distribution (lines 177-186)

```cpp
Eigen::Vector3d ErrorState::omega_a_gm() {
    Eigen::Vector3d omega_a_gm_(dis_(gen_), dis_(gen_), dis_(gen_));
    return omega_a_gm_max_ * omega_a_gm_;
}
Eigen::Vector3d ErrorState::omega_g_gm() {
    Eigen::Vector3d omega_g_gm_(dis_(gen_), dis_(gen_), dis_(gen_));
    return omega_g_gm_max_ * omega_g_gm_;
}
```

Same issue as 7.1 — bias random walk should be modeled through the covariance prediction (`Q` matrix), not by injecting random perturbations into the state itself.

### 7.3 Q_ * dt covariance scaling

**File**: `ErrorStateCovariance.cpp`, line 23

```cpp
P_ = phi * P_ * phi.transpose() + Q_ * dt;
```

The standard discrete-time EKF covariance prediction is:

$$P_{k+1} = \Phi P_k \Phi^T + Q_d$$

where $Q_d$ is the discretized process noise. For a continuous-time system:

$$Q_d \approx \int_0^{\Delta t} \Phi(\tau) G Q_c G^T \Phi^T(\tau) d\tau$$

The first-order approximation is $Q_d \approx G Q_c G^T \Delta t$, not $Q_c \Delta t$. The repo uses the Q matrix directly (which is supposed to be in continuous-time units) multiplied by dt, which means:
- If Q is a continuous spectral density, then `Q * dt` gives the correct discrete noise magnitude, BUT only if F and G are identity
- Since Q is diagonal and doesn't include the G matrix, the noise is not being projected through the correct noise input channels

**Impact**: The process noise magnitude may be incorrect by orders of magnitude depending on the state. The default Q matrix in `EKF.cpp` (lines 30-32) was presumably empirically tuned to work despite this, which masks the underlying mathematical issue.

### 7.4 P matrix initialized to zero

**File**: `ErrorStateCovariance.cpp`, line 10

```cpp
P_.setZero();
```

A zero covariance matrix means the filter has **infinite confidence** in its initial state estimate. Standard practice is to initialize with plausible uncertainties:

- PX4: `EKF2_GBIAS_INIT`, `EKF2_ABIAS_INIT`, `EKF2_ANGERR_INIT` parameters
- ArduPilot: Per-state initial variances based on sensor specs

With P=0, the first few GPS updates will barely change the state (because the filter believes it's already correct), and the filter convergence will be slow. In practice, this is partially mitigated because the process noise `Q * dt` quickly inflates P away from zero.

### 7.5 State transition via full matrix exponential

**File**: `ErrorState.cpp`, lines 206-209

```cpp
Eigen::MatrixXd ErrorState::getTransitionMatrix(double dt) {
    Eigen::MatrixXd coeff = F() * dt;
    return coeff.exp();
}
```

The matrix exponential `expm(F·dt)` is the exact solution for linear time-invariant systems, but:
1. It's computationally expensive — Eigen's matrix exponential uses the Pade approximation with scaling and squaring
2. For small dt (typical IMU: 1-10ms), `I + F·dt` is an excellent approximation at much lower cost
3. Both PX4 and ArduPilot use the first-order Taylor approximation for performance on embedded hardware

**This repo can afford it** because it only propagates the covariance once per IMU batch (not per IMU sample — see 7.6), but it's still non-standard for embedded/real-time systems.

### 7.6 IMU mean-before-propagation strategy

**File**: `Tracking.cpp`, lines 52-62

```cpp
void Tracking::checkAndUpdate() {
    if (!f_measurements_.empty() && !g_measurements_.empty()) {
        Eigen::Vector3d f_mean = Utils::calcMeanVector(f_measurements_);
        Eigen::Vector3d g_mean = Utils::calcMeanVector(g_measurements_);
        f_measurements_.clear();
        g_measuresments_.clear();
        updateDT();
        updateTrackingWithMeasurements(f_mean, g_mean);
        resetClock();
    }
}
```

**Problem**: IMU data arriving between GPS updates is averaged and processed as a single step. This means:
1. High-frequency dynamics (vibration, bumps) are lost in averaging
2. The covariance prediction happens once per batch, not at IMU rate
3. The effective prediction rate varies with GPS arrival (1-10 Hz typical), meaning the prediction step uses a large dt

**Standard practice** (both PX4 and ArduPilot): propagate the filter at IMU rate (100-400 Hz) and only fuse measurements when they arrive. This gives better tracking of high-frequency dynamics and more accurate covariance propagation.

**Impact for land vehicles**: At typical road speeds (50-100 km/h), vehicle dynamics are at 1-10 Hz, so the averaging may be acceptable. However, the covariance prediction with large dt reduces the accuracy of the P propagation.

### 7.7 Latitude-dependent Earth model but local NED for correction

The repo uses full WGS-84 for INS mechanization (Rm, Rn, transport rate, Earth rotation rate) but GPS updates use a simple Cartesian subtraction (`z = pos_vel_state - gps_data_mean`). This assumes the difference between INS position and GPS position can be computed as a simple vector difference in [lat, lon, alt] space.

**Standard practice**: Convert to a local Cartesian frame (e.g., ECEF or local tangent plane) before computing the innovation. This matters when the INS and GPS positions are significantly different (e.g., during initialization or after a long outage).

**This is partially correct** because [lat, lon, alt] with small errors is approximately Cartesian, but the latitude curvature terms in the D matrix are non-linear for larger errors.

### 7.8 Single chi-square gate with no persistent failure handling

**File**: `EKF.cpp`, lines 112-120

```cpp
double mahalanobis_sq = z.transpose() * S.llt().solve(z);
constexpr double chi_sq_threshold_6dof = 18.0;  // χ²(6) ~99.7% (3σ)
if (mahalanobis_sq > chi_sq_threshold_6dof) {
    logger_->warn("GPS innovation rejected: d²={:.1f} > threshold={:.1f}",
                  mahalanobis_sq, chi_sq_threshold_6dof);
    return;  // skip measurement entirely
}
```

- Single threshold for all 6 axes (position + velocity combined)
- No persistent failure tracking — the next measurement is accepted unconditionally
- No selective axis rejection — one bad axis rejects all 6
- No innovation variance inflation for near-rejection measurements

**Contrast with ArduPilot**: per-axis innovation gates, persistent failure tracking (`faultStatus.bad_nvel`, `bad_evel`, etc.), GPS-glitch handling with position reset, and variance inflation for soft rejection.

### 7.9 GPS azimuth alignment: hard threshold at 5 m/s

**File**: `EKF.cpp`, line 94

```cpp
bool is_vel_higher_than_threshold = vel >= velocity_threshold_;
```

With `velocity_threshold_ = 5.0 m/s` (default), heading is only corrected by GPS velocity when horizontal speed exceeds 18 km/h. This means:
- At speeds below 18 km/h, heading drifts freely
- During acceleration from a stop, heading jumps when the threshold is crossed
- No hysteresis — the filter can oscillate around the threshold

**Contrast with PX4/ArduPilot**: Both use the EKF-GSF (Gaussian Sum Filter) yaw estimator, which runs 5 parallel yaw hypotheses and works at any speed where GPS velocity has a consistent direction. There is no hard threshold.

### 7.10 Euler angle subtraction for orientation correction

**File**: `EKF.cpp`, lines 137-138

```cpp
Eigen::Vector3d ins_navigation_euler_angles = EKF_INS::Utils::toEulerAngles(...);
std::get<2>(fixed_navigation_state_) = EKF_INS::Utils::toRotationMatrix(ins_navigation_euler_angles - epsilon_n);
```

When azimuth alignment is active, the orientation correction uses Euler angle subtraction:

```cpp
T_corrected = R(θ_INS - ε)
```

This is mathematically problematic because:
1. Euler angles have singularities (gimbal lock)
2. Euler angle subtraction is not the same as the correct rotation composition
3. The correction `R(θ - ε)` ≠ `(I - [ε×]) · R(θ)` (the small-angle approximation used in the non-alignment path)

When azimuth alignment is NOT active (line 141-143), the repo uses the correct small-angle approximation:
```cpp
T_corrected = (I - [ε×]) · T_INS
```

This inconsistency means the filter behaves differently depending on whether azimuth alignment is triggered, which is mathematically incoherent.

---

## 8. Time-Horizon and Latency Handling

### 8.1 Current Approach

This repo has no mechanism for handling sensor latency:
- GPS measurements are used as soon as they arrive
- The INS state at GPS arrival time is used as the reference
- The error is computed as `z = x_INS(t_now) - x_GPS(t_measurement)` — a temporal mismatch

### 8.2 Production Standard

Both PX4 and ArduPilot use a **delayed fusion horizon**:

```
Sensor input timing:
IMU @ t=0ms ─────────────► FIFO buffer
GPS @ t=0ms ──[200ms delay]──► GPS buffer (timestamp-adjusted: t=0ms)

EKF timeline:
t= -100ms    t=0ms (latest)    t=+100ms  (current)
    ▲                            ▲
    │                            │
EKF fusion horizon          Output predictor
(runs on delayed data)      (propagates to real-time)
```

The delayed approach adds complexity (buffer management, output predictor) but ensures:
- All sensor measurements are fused at their correct time
- The state and covariance are consistent at each fusion time
- Real-time output is available via the complementary filter

### 8.3 Impact on Land Vehicles

For low-speed land vehicles (< 10 m/s), the temporal misalignment error from the current-time approach is small (sub-meter). However, if this filter were used in a control loop (e.g., autonomous driving), the latency between state estimate and actual vehicle state would cause stability issues. The output predictor is essential for control applications.

---

## 9. Initialization and Alignment

### 9.1 Current Approach

```cpp
ekf.setInitialState(p_0, v_0, T_0);
ekf.start();
```

The user provides the full navigation state (position, velocity, DCM). There is no automated alignment:
- No gravity-based tilt alignment (roll/pitch from accelerometer)
- No magnetometer-based yaw alignment
- No in-motion yaw initialization

The azimuth alignment (GPS-velocity-based heading) only activates after `start()` and when speed >5 m/s.

### 9.2 Production Standard

**PX4 EKF2 alignment sequence:**

```
1. Check if stationary (accel_norm ≈ 1G, gyro_norm < 15°/s)
2. Tilt alignment: compare accelerometer vector to gravity (roll/pitch)
3. Yaw alignment: try in order:
   a. Magnetometer + WMM declination
   b. EKF-GSF (5 parallel AHRS models, GPS velocity-based)
   c. Dual-antenna GPS yaw
   d. External vision yaw
4. Set initial covariance from parameters (EKF2_GBIAS_INIT, etc.)
```

**ArduPilot EKF3 alignment:**
- Similar sequence, with additional magnetometer handling modes (`EK3_MAG_CAL`)
- GSF yaw estimator used as default backup

### 9.3 Impact

The repo's dependency on user-provided initial state means:
- No plug-and-play operation — the user must have an external alignment source
- No re-alignment capability — if the initial heading is wrong, it can only correct via GPS heading (slow convergence at moderate speeds)
- The initial DCM `T_0 = Identity` (from the usage example) means the filter assumes the vehicle body frame aligns with NED at startup, which is almost never true

---

## 10. GPS Outage and Re-acquisition

### 10.1 Current Approach

- No explicit GPS outage handling
- If `updateWithGPSMeasurements` is not called, the filter continues in pure INS mode (IMU integration only)
- Position and velocity drift without bounds
- When GPS resumes, the next measurement is fused normally (subject to the chi-square gate)

### 10.2 Production Standard

**PX4 EKF2 dead-reckoning hierarchy:**

```
Normal aiding (GPS/vision/flow active)
  ↓  <no_aid_timeout_max = 1s>
Wind Dead Reckoning (airspeed + sideslip still fusing)
  ↓  <no_aid_timeout_max = 1s>
Inertial Dead Reckoning (IMU only)
  ↓  <ekf2_noaid_tout = 5s>
=> Navigation invalid
```

**ArduPilot EKF3:**

- Three aiding modes: `AID_ABSOLUTE` (GPS), `AID_RELATIVE` (optical flow), `AID_NONE` (IMU only)
- During IMU-only coasting: horizontal acceleration is artificially constrained to 5 m/s² to limit attitude drift
- With wheel encoder: forward velocity is directly measured, giving bounded position drift
- GPS re-acquisition: position reset if variance exceeds glitch radius

### 10.3 Impact for Land Vehicles

The repo's approach is acceptable only if GPS dropout is brief (< a few seconds). For longer outages:
- Position error grows as ~t³ (double-integrated accelerometer noise + bias drift)
- After 60 seconds of IMU-only coasting with typical MEMS IMU: ~100-1000m position error
- With wheel encoders (ArduPilot): after 60 seconds: ~1-10m position error (heading drift only)

---

## 11. Outlier Rejection and Fault Detection

### 11.1 Current Approach

- Single 6-DOF chi-squared innovation gate
- Threshold: 18.0 (≈99.7% confidence for 6 DOF)
- No per-axis rejection
- No persistent fault tracking
- No innovation variance inflation
- No sensor quality checks (satellite count, HDOP, etc.)

### 11.2 Production Standard

**ArduPilot EKF3 fault tracking:**

```cpp
struct {
    bool bad_xmag, bad_ymag, bad_zmag;      // per-axis mag failures
    bool bad_airspeed, bad_sideslip;
    bool bad_nvel, bad_evel, bad_dvel;       // per-axis velocity
    bool bad_npos, bad_epos, bad_dpos;       // per-axis position
    bool bad_yaw, bad_decl;
    bool bad_xflow, bad_yflow;               // per-axis optical flow
    bool bad_rngbcn;
    bool bad_xvel, bad_yvel, bad_zvel;       // per-axis body velocity
} faultStatus;
```

Per-axis innovation gating with configurable gates per sensor type. Persistent failures trigger fusion suspension after a timeout, preventing the filter from being corrupted by a continuously failing sensor.

### 11.3 Impact

Without proper fault detection, a single bad GPS measurement (common in urban environments due to multipath) can corrupt the filter state. The chi-square gate provides basic protection but has no memory — a burst of bad measurements will all be accepted individually.

---

## 12. Land Vehicle Readiness Assessment

### 12.1 Functional Scorecard

| Requirement | Importance | Score (1-5) | Notes |
|-------------|-----------|-------------|-------|
| Position accuracy (open sky) | Critical | 4 | Full WGS-84, proper GPS fusion |
| Position accuracy (urban) | Critical | 1 | No multipath rejection, no mag |
| Heading accuracy (moving) | Critical | 3 | GPS velocity heading >5 m/s |
| Heading accuracy (stopped) | Critical | 1 | No magnetometer, heading drifts |
| Heading accuracy (initial) | High | 1 | No automated alignment |
| GPS outage tolerance | Critical | 1 | Pure IMU, no wheel encoder |
| Stop-and-go handling | Critical | 1 | No ZVU, no stationary detection |
| Real-time output | High | 2 | No output predictor |
| Sensor failure tolerance | High | 1 | No persistent fault tracking |

**Overall: 1.7 / 5** — Not production-ready for land vehicle navigation.

### 12.2 Airworthiness vs Roadworthiness

This filter was designed for aerial applications (the code references are from an 2018 academic context). The key differences:

| Aspect | Aerial (designed for) | Land vehicle (needed) |
|--------|----------------------|----------------------|
| Heading | GPS velocity works well (constant motion) | Need mag heading for stops |
| Dynamics | High-frequency, all axes | Low-frequency, constrained to ground plane |
| GPS environment | Open sky (mostly) | Urban canyons, tunnels, multipath |
| Outage tolerance | Short (landing) | Long (parking garage, tunnel) |
| Motion constraints | 6-DOF free | 3-DOF constrained (NHC) |
| Sensor diversity | Airspeed, baro | Wheel encoder, magnetometer |

---

## 13. Recommended Improvements

### Tier 1 — Critical (mathematical correctness)

| # | Improvement | File(s) | Effort | Impact |
|---|-------------|---------|--------|--------|
| 1 | Remove random noise injection (`omega_a()`, `omega_g()`, `omega_a_gm()`, `omega_g_gm()`) | `ErrorState.cpp` | Low | Fixes incorrect EKF math |
| 2 | Implement proper covariance initialization (non-zero P₀) | `ErrorStateCovariance.cpp` | Low | Faster convergence, correct uncertainty |
| 3 | Fix process noise discretization: `P = F*P*F^T*dt + G*Q*G^T*dt` (or use discrete form) | `ErrorStateCovariance.cpp` | Low | Correct covariance growth |
| 4 | Remove Euler-angle-based orientation correction in azimuth alignment path | `EKF.cpp` | Low | Correct attitude correction |

### Tier 2 — Important (land vehicle capability)

| # | Improvement | Effort | Impact |
|---|-------------|--------|--------|
| 5 | Add non-holonomic constraint (zero lateral/vertical velocity in body frame) as synthetic measurement | Medium | Free information, big improvement |
| 6 | Add zero-velocity detection + update (stop detection from IMU) | Medium | Bias estimation, drift suppression |
| 7 | Add magnetometer heading fusion model (simplified: heading-only, no full 3D mag states) | Medium | Heading at all times |
| 8 | Add hysteresis on velocity threshold for azimuth alignment | Low | Smooth heading transitions |

### Tier 3 — Enhancement (performance / robustness)

| # | Improvement | Effort | Impact |
|---|-------------|--------|--------|
| 9 | Per-axis innovation gating with persistent failure tracking | Medium | Robust outlier rejection |
| 10 | IMU-rate propagation (instead of batch mean) | Medium | Better high-frequency tracking |
| 11 | Wheel encoder velocity measurement model | Medium | Bounded GPS-outage drift |
| 12 | Barometric altitude fusion | Low | Better height than GPS altitude |
| 13 | Delayed fusion horizon + output predictor | High | Correct temporal alignment |

---

## References

1. **Groves, P.D.** *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed., Artech House, 2013.
2. **Titterton, D. & Weston, J.** *Strapdown Inertial Navigation Technology*, 2nd ed., IET, 2004.
3. **Sola, J.** *Quaternion kinematics for the error-state Kalman filter*, arXiv:1711.02508, 2017.
4. PX4 EKF2 source: `src/modules/ekf2/` @ `94580ab1` ([GitHub](https://github.com/PX4/PX4-Autopilot))
5. ArduPilot EKF3 source: `libraries/AP_NavEKF3/` @ `c979b1c8d` ([GitHub](https://github.com/ArduPilot/ardupilot))
6. PX4 EKF2 documentation: [docs.px4.io](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html)

---

*Report prepared by Sisyphus — 2026-05-26*

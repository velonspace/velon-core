# Vertical Landing System (v0)

## 1. Objective

Simulate and control the vertical descent of a rocket to achieve a safe landing.

The system must:

* Start from an initial height and velocity
* Apply thrust as control input
* Reach ground (h = 0) with near-zero velocity

---

## 2. System Type

* 1D vertical motion
* No rotation
* No aerodynamics
* No disturbances

This is a simplified physics model for control development.

---

## 3. State Variables

The system state at any time t is:

* h (height in meters)
* v (velocity in m/s)

---

## 4. Control Input

* T (thrust in Newtons)

Constraints:

* 0 ≤ T ≤ T_max

---

## 5. Constants / Parameters

* m (mass in kg)
* g (gravity = 9.81 m/s²)

Optional (later):

* T_max (maximum thrust)
* dt (time step)

---

## 6. System Dynamics

Acceleration:
a = (T / m) - g

State update:

* v(t+1) = v(t) + a * dt
* h(t+1) = h(t) + v(t) * dt

---

## 7. Initial Conditions

Example:

* h₀ = 100 m
* v₀ = 0 m/s (or negative for descent)

---

## 8. Constraints

* h ≥ 0 (ground level)
* T ≥ 0 (no negative thrust)
* T ≤ T_max

---

## 9. Success Criteria

A successful landing must satisfy:

* Final height ≈ 0
* Final velocity ≈ 0
* No oscillation near ground
* No excessive thrust usage (efficiency consideration)

---

## 10. Failure Conditions

* Impact with high velocity (crash)
* Oscillation near ground (unstable control)
* Thrust saturation for long duration

---

## 11. Future Extensions (NOT NOW)

* 2D/3D motion
* Rotation (attitude control)
* Aerodynamic drag
* Sensor noise
* Engine delay

---
 

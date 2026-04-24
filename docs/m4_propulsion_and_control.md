# M4 — Propulsion, Control & Robust Landing

## Objective
Extend the vertical landing simulation to include propulsion dynamics, actuator limits, and real-world uncertainties, and validate robustness under varying conditions.

---

## System Additions

### 1. Propulsion Model
- Variable mass (fuel consumption)
- Specific impulse (Isp-based mass flow)
- Thrust saturation limits

### 2. Actuator Dynamics
- Thrust rate limiting (engine response lag)
- Prevents unrealistic instantaneous thrust changes

### 3. Environmental Disturbances
- Random external forces (±2N to ±10N)
- Simulates wind / modeling uncertainty

### 4. Sensor Noise
- Height and velocity noise added
- Tests controller robustness to imperfect measurements

---

## Key Challenge

When actuator lag was introduced:
- Late braking caused high impact velocities
- Resulted in extreme deceleration spikes (~15–50 g)

### Root Cause
Controller assumed instantaneous thrust → invalid under real actuator constraints

---

## Solution: Lag-Aware Trigger

Instead of fixed-height triggering, we implemented:

- Stopping distance based trigger:
  
  d_stop = v² / (2 * a_max)

- Adjusted with:
  - Lag factor (engine delay compensation)
  - Safety margin

### Result
- Early, smooth braking
- No last-second thrust spikes
- Stable descent across all conditions

---

## Validation Method

Monte Carlo testing across:

1. Nominal conditions  
2. High disturbances (±7N)  
3. High sensor noise  
4. Slow engine response  
5. Extreme combined stress  

---

## Results

- Success Rate: 100% across all tests
- Peak g-load: ~0.9 – 1.2 g
- Max g (worst case): ~1.21 g
- Fuel usage: ~0.18 – 0.22 kg
- G-limit violations (>5g): 0

---

## Key Insights

- Control strategy dominates structural load reduction
- Actuator limits must be included in control design
- Anticipative control outperforms reactive control
- Robustness must be validated statistically, not by single runs

---

## System Capability

The system now includes:

- Guidance & control (M2)
- Structural interaction (M3)
- Propulsion dynamics (M4)
- Disturbance handling
- Sensor uncertainty
- Robust validation framework

---

## Conclusion

A lag-aware, physics-driven control system enables safe and reliable landings with near-1 g impact loads under realistic disturbances and actuator constraints.

---

## Next Step — M5

Extend to 2D dynamics:
- Lateral motion (x-axis)
- Tilt dynamics
- Thrust vectoring
- Precision landing guidance
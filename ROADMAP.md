# 🚀 Velon Space Systems — Development Roadmap

## 🧭 Vision
Build a complete guidance, navigation, and control (GNC) system for vertical landing rockets, starting from first principles.

---

## ✅ M1 — Physics Core (COMPLETED)
- 1D vertical dynamics (height, velocity)
- Correct force modeling (thrust vs gravity)
- Stable simulation loop

---

## ✅ M2 — Controlled Landing (COMPLETED)
- Energy-based braking (predictive control)
- Feedback stabilization (velocity + position)
- Robustness:
  - sensor noise
  - actuator lag
  - disturbances
- Smooth touchdown (~ -0.3 m/s)

---

## 🔜 M3 — Landing Gear & Ground Interaction
- Spring-damper landing model
- Ground contact handling
- Impact absorption

---

## 🔜 M4 — Engine & Mass Realism
- Fuel consumption (mass variation)
- Throttle rate limits
- Engine dynamics

---

## 🔜 M5 — 2D Motion (Planar Dynamics)
- Horizontal position (x-axis)
- Tilt / attitude
- Thrust vectoring

---

## 🔜 M6 — Guidance System
- Trajectory planning
- Target landing point
- Path shaping

---

## 🔜 M7 — State Estimation
- Sensor fusion
- Filtering (Kalman / complementary)

---

## 🔜 M8 — Optimization
- Fuel efficiency
- Trajectory optimization
- Control tuning

---

## 🔜 M9 — System-Level Simulation
- Modular architecture
- Config-driven runs
- Multiple scenarios

---

## 📌 Current Status
➡️ M1 + M2 COMPLETE  
➡️ Next: M3 (Landing Gear)

---

## 🧠 Notes
- Each milestone builds on the previous
- Focus on robustness over perfection
- Prioritize physical realism progressively
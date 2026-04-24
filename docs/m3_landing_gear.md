# M3 — Landing Gear & Structural Interaction

## 📌 Objective
Introduce a physical landing gear model (spring-damper) to simulate ground impact and analyze structural loads during landing.

---

## ⚙️ Model Description

Landing gear is modeled as a spring-damper system:

F = k * x - c * v

Where:
- k = stiffness (N/m)
- c = damping coefficient (Ns/m)
- x = compression (m) = -h when h < 0
- v = vertical velocity (m/s)

Force is applied only when:
h < 0 (i.e., ground contact)

---

## 🧪 Experiments Conducted

### 1. Drop Test (No Control)
- Initial height: 10 m
- No thrust applied
- Purpose: simulate worst-case impact

### 2. Controlled Landing
- Initial height: 100 m
- Hybrid control (predictive + feedback)
- Velocity-targeted final approach

---

## 📊 Results

### Drop Test
- Max Force       : 1707.58 N
- Max Compression : 0.0856 m
- Peak g-load     : 174.07 g

### Controlled Landing
- Max Force       : 41.58 N
- Max Compression : 0.0018 m
- Peak g-load     : 4.24 g

---

## 📈 Key Observations

- Controlled landing reduces peak force by ~97.6%
- Compression reduced by ~97.8%
- Structural load is dominated by impact velocity
- Control system significantly reduces energy before contact

---

## 🧠 Insights

- Landing is not just a structural problem — it is a **control + structure interaction**
- Reducing velocity before contact is more effective than relying on structural absorption
- Damping reduces oscillations but can increase peak force
- Stiffness vs damping trade-off is critical in design

---

## ⚠️ Limitations

- Constant mass (no fuel burn)
- No lateral dynamics (1D only)
- Simplified ground model (no terrain variation)
- No structural stress distribution (lumped model)

---

## 🔜 Next Step (M4)

Introduce propulsion realism:
- Fuel consumption → variable mass
- Thrust dynamics
- Engine constraints

---

## 📌 Conclusion

M3 successfully introduces structural realism and demonstrates that control strategy plays a dominant role in reducing landing loads.
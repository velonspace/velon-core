from core.dynamics.vertical_model import step
import random

# -----------------------
# Parameters
# -----------------------
params = {
    "m": 1.0,
    "g": 9.81,
    "T_max": 40.0
}

dt = 0.01
t_final = 15.0

# -----------------------
# Initial State
# -----------------------
state = {
    "h": 100.0,
    "v": 0.0
}

t = 0.0
thrust_actual = 0.0

# gains
k_v = 0.8        # mid-altitude velocity damping
k_h = 0.3        # position pull
k_v_final = 2.2  # stronger near ground

# final approach settings
h_switch = 0.5
target_v = -0.35  # m/s (gentle descent)

print("Hybrid control with velocity-targeted final approach...\n")

while t < t_final:

    h = state["h"]
    v = state["v"]

    m = params["m"]
    g = params["g"]
    T_max = params["T_max"]

    # -----------------------
    # SENSOR NOISE
    # -----------------------
    measured_h = h + random.uniform(-0.2, 0.2)
    measured_v = v + random.uniform(-0.2, 0.2)

    # -----------------------
    # CONTROL
    # -----------------------
    if measured_h <= 40:

        if measured_h < h_switch:
            # -------- FINAL APPROACH (velocity target) --------
            # drive (v -> target_v)
            a_total = -k_v_final * (measured_v - target_v)

        else:
            # -------- HYBRID (physics + feedback) --------
            if measured_h > 1e-3:
                a_physics = (measured_v**2) / (2 * measured_h)
            else:
                a_physics = 0.0

            a_feedback = -k_v * measured_v
            a_position = -k_h * measured_h

            a_total = a_physics + a_feedback + a_position

        thrust_cmd = m * (a_total + g)

        # clamp to actuator limits
        thrust_cmd = max(0.0, min(thrust_cmd, T_max))

    else:
        thrust_cmd = 0.0

    # -----------------------
    # THRUST LAG
    # -----------------------
    alpha = 0.1
    thrust_actual += alpha * (thrust_cmd - thrust_actual)

    # -----------------------
    # DISTURBANCE
    # -----------------------
    disturbance = random.uniform(-0.5, 0.5)
    thrust_effective = thrust_actual + m * disturbance

    # -----------------------
    # STEP
    # -----------------------
    state = step(state, thrust_effective, dt, params)

    # debug every ~0.5 s
    if int(t * 100) % 50 == 0:
        print(f"t={t:.2f}, h={h:.2f}, v={v:.2f}, T_cmd={thrust_cmd:.2f}, T_act={thrust_actual:.2f}")

    # -----------------------
    # TOUCHDOWN CONDITION
    # -----------------------
    if state["h"] <= 0.05:
        print("\nTouched down (within tolerance)")
        print("Impact velocity:", state["v"])
        break

    t += dt

# -----------------------
# Final Output
# -----------------------
print("\nFinal state:", state)
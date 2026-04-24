from core.dynamics.vertical_model import step
import random

# -----------------------
# Parameters
# -----------------------
params = {
    "m": 1.0,
    "g": 9.81,
    "T_max": 40.0,

    # Landing gear (NEW)
    "k": 800.0,   # stiffness
    "c": 80.0     # damping
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

# control gains
k_v = 0.8
k_h = 0.3
k_v_final = 2.2

h_switch = 0.5
target_v = -0.35

print("M3: Landing gear simulation...\n")

# -----------------------
# Simulation Loop
# -----------------------
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
            a_total = -k_v_final * (measured_v - target_v)

        else:
            if measured_h > 1e-3:
                a_physics = (measured_v**2) / (2 * measured_h)
            else:
                a_physics = 0.0

            a_feedback = -k_v * measured_v
            a_position = -k_h * measured_h

            a_total = a_physics + a_feedback + a_position

        thrust_cmd = m * (a_total + g)
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
    # LANDING GEAR FORCE (NEW)
    # -----------------------
    k = params["k"]
    c = params["c"]

    if state["h"] < 0:
        compression = -state["h"]  # positive
        gear_force = k * compression - c * state["v"]
    else:
        gear_force = 0.0

    # -----------------------
    # TOTAL FORCE
    # -----------------------
    total_thrust = thrust_effective + gear_force

    # -----------------------
    # STEP
    # -----------------------
    state = step(state, total_thrust, dt, params)

    # -----------------------
    # DEBUG
    # -----------------------
    if int(t * 100) % 50 == 0:
        print(f"t={t:.2f}, h={h:.2f}, v={v:.2f}, gearF={gear_force:.2f}")

    # -----------------------
    # STOP CONDITION
    # -----------------------
    if state["h"] < -0.5:  # too deep compression
        print("\nHard crash (structure failed)")
        break

    # near rest condition
    if abs(state["h"]) < 0.02 and abs(state["v"]) < 0.1:
        print("\nStable landing achieved")
        print("Final compression:", -state["h"])
        break

    t += dt

# -----------------------
# Final Output
# -----------------------
print("\nFinal state:", state)
from core.dynamics.vertical_model import step
import random

# -----------------------
# PARAMETERS
# -----------------------
params = {
    "m": 1.0,
    "g": 9.81,
    "T_max": 40.0,

    # Landing gear
    "k": 800.0,
    "c": 120.0
}

dt = 0.01
t_final = 15.0


# -----------------------
# LANDING GEAR FORCE
# -----------------------
def gear_force(h, v, params):
    if h < 0:
        compression = -h
        return params["k"] * compression - params["c"] * v, compression
    return 0.0, 0.0


# -----------------------
# DROP TEST
# -----------------------
def run_drop():
    state = {"h": 10.0, "v": 0.0}
    t = 0.0

    max_force = 0.0
    max_comp = 0.0

    settle_count = 0
    settle_required = 50  # 0.5 sec

    while t < t_final:

        # gear force (based on CURRENT state)
        gF, comp = gear_force(state["h"], state["v"], params)

        max_force = max(max_force, gF)
        max_comp = max(max_comp, comp)

        # step simulation
        state = step(state, gF, dt, params)

        # settle detection (robust)
        if abs(state["v"]) < 0.05 and state["h"] < 0:
            settle_count += 1
        else:
            settle_count = 0

        if settle_count >= settle_required:
            break

        t += dt

    return max_force, max_comp


# -----------------------
# CONTROLLED LANDING
# -----------------------
def run_controlled():
    state = {"h": 100.0, "v": 0.0}
    t = 0.0
    thrust_actual = 0.0

    max_force = 0.0
    max_comp = 0.0

    settle_count = 0
    settle_required = 50

    # control gains
    k_v = 0.8
    k_h = 0.3
    k_v_final = 2.2

    h_switch = 0.5
    target_v = -0.35

    while t < t_final:

        h, v = state["h"], state["v"]

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
        # GEAR FORCE (CONSISTENT)
        # -----------------------
        gF, comp = gear_force(state["h"], state["v"], params)

        max_force = max(max_force, gF)
        max_comp = max(max_comp, comp)

        # total force
        total_force = thrust_effective + gF

        # step
        state = step(state, total_force, dt, params)

        # -----------------------
        # SETTLE DETECTION
        # -----------------------
        if abs(state["v"]) < 0.05 and state["h"] < 0:
            settle_count += 1
        else:
            settle_count = 0

        if settle_count >= settle_required:
            break

        t += dt

    return max_force, max_comp


# -----------------------
# RUN BOTH
# -----------------------
print("\nRunning comparison...\n")

drop_force, drop_comp = run_drop()
ctrl_force, ctrl_comp = run_controlled()

# -----------------------
# RESULTS
# -----------------------
print("====== RESULTS ======\n")

print("DROP TEST:")
print(f"Max Force       : {drop_force:.2f} N")
print(f"Max Compression : {drop_comp:.4f} m")
print(f"Peak g-load     : {drop_force / (params['m'] * params['g']):.2f} g\n")

print("CONTROLLED LANDING:")
print(f"Max Force       : {ctrl_force:.2f} N")
print(f"Max Compression : {ctrl_comp:.4f} m")
print(f"Peak g-load     : {ctrl_force / (params['m'] * params['g']):.2f} g\n")

print("IMPROVEMENT:")
print(f"Force Reduction : {100*(1 - ctrl_force/drop_force):.2f}%")
print(f"Compression Red.: {100*(1 - ctrl_comp/drop_comp):.2f}%")
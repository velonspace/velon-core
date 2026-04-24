from core.dynamics.vertical_model import step
import random

# -----------------------
# PARAMETERS
# -----------------------
params = {
    "m": 5.0,
    "g": 9.81,
    "T_max": 100.0,

    # propulsion
    "Isp": 300.0,

    # landing gear
    "k": 800.0,
    "c": 120.0,
    "max_compression": 0.2
}

dt = 0.01
t_final = 20.0

# -----------------------
# INITIAL STATE
# -----------------------
state = {"h": 100.0, "v": 0.0}

mass = params["m"]
fuel_used = 0.0

t = 0.0
thrust_actual = 0.0

# control gains
k_v = 0.8
k_h = 0.3
k_v_final = 2.2

h_switch = 0.5
target_v = -0.35
control_start = 60.0

# settle detection
settle_count = 0
settle_required = 50

# metrics
max_force = 0.0
max_comp = 0.0
mass_at_peak = mass

print("\n🚀 M4 FINAL: Propulsion + Structure + Disturbances\n")

# -----------------------
# SIMULATION LOOP
# -----------------------
while t < t_final:

    h, v = state["h"], state["v"]

    g = params["g"]
    T_max = params["T_max"]
    Isp = params["Isp"]

    # -----------------------
    # SENSOR NOISE
    # -----------------------
    measured_h = h + random.uniform(-0.1, 0.1)
    measured_v = v + random.uniform(-0.1, 0.1)

    # -----------------------
    # CONTROL
    # -----------------------
    if measured_h <= control_start:

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

        thrust_cmd = mass * (a_total + g)
        thrust_cmd = max(0.0, min(thrust_cmd, T_max))

    else:
        thrust_cmd = 0.0

    # -----------------------
    # THRUST RATE LIMIT
    # -----------------------
    max_rate = 200.0  # N/s
    delta = thrust_cmd - thrust_actual
    delta = max(-max_rate * dt, min(delta, max_rate * dt))
    thrust_actual += delta

    # -----------------------
    # DISTURBANCE
    # -----------------------
    disturbance = random.uniform(-7.0, 7.0)
    thrust_effective = thrust_actual + disturbance

    # -----------------------
    # MASS FLOW
    # -----------------------
    mdot = thrust_actual / (Isp * g) if thrust_actual > 0 else 0.0
    mass -= mdot * dt
    fuel_used += mdot * dt
    mass = max(mass, 1.0)

    # -----------------------
    # LANDING GEAR + CLAMP
    # -----------------------
    if h < 0:
        comp = -h

        # hard stop
        if comp > params["max_compression"]:
            comp = params["max_compression"]
            state["h"] = -comp
            state["v"] = 0.0

        gear_force = params["k"] * comp - params["c"] * v

        # metrics
        if gear_force > max_force:
            max_force = gear_force
            mass_at_peak = mass

        max_comp = max(max_comp, comp)

    else:
        gear_force = 0.0

    # -----------------------
    # TOTAL FORCE
    # -----------------------
    total_force = thrust_effective + gear_force

    # update mass
    params["m"] = mass

    # -----------------------
    # STEP
    # -----------------------
    state = step(state, total_force, dt, params)

    # -----------------------
    # SETTLE DETECTION
    # -----------------------
    if abs(state["v"]) < 0.05 and state["h"] < 0:
        settle_count += 1
    else:
        settle_count = 0

    if settle_count >= settle_required:
        print("\n🟢 Landing complete")
        break

    # -----------------------
    # DEBUG
    # -----------------------
    if int(t * 100) % 50 == 0:
        print(f"t={t:.2f}, h={h:.2f}, v={v:.2f}, m={mass:.2f}, T={thrust_actual:.2f}")

    t += dt

# -----------------------
# FINAL OUTPUT
# -----------------------
print("\n====== FINAL RESULTS ======")
print("Final state:", state)
print(f"Final mass: {mass:.2f} kg")
print(f"Fuel used: {fuel_used:.2f} kg")

print("\nSTRUCTURAL METRICS:")
print(f"Max gear force: {max_force:.2f} N")
print(f"Max compression: {max_comp:.4f} m")
print(f"Peak g-load: {max_force / (mass_at_peak * params['g']):.2f} g")
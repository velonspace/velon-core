from core.dynamics.vertical_model import step
import random

def run_sim():

    params = {
        "m": 5.0,
        "g": 9.81,
        "T_max": 100.0,
        "Isp": 300.0,
        "k": 800.0,
        "c": 120.0,
        "max_compression": 0.2
    }

    dt = 0.01
    t_final = 20.0

    state = {"h": 100.0, "v": 0.0}
    mass = params["m"]
    fuel_used = 0.0
    thrust_actual = 0.0

    k_v = 0.8
    k_h = 0.3
    k_v_final = 2.2

    h_switch = 0.5
    target_v = -0.35
    control_start = 60.0

    settle_count = 0
    settle_required = 50

    max_force = 0.0
    mass_at_peak = mass

    t = 0.0

    while t < t_final:

        h, v = state["h"], state["v"]

        measured_h = h + random.uniform(-0.1, 0.1)
        measured_v = v + random.uniform(-0.1, 0.1)

        if measured_h <= control_start:

            if measured_h < h_switch:
                a_total = -k_v_final * (measured_v - target_v)
            else:
                if measured_h > 1e-3:
                    a_physics = (measured_v**2) / (2 * measured_h)
                else:
                    a_physics = 0.0

                a_total = a_physics - k_v * measured_v - k_h * measured_h

            thrust_cmd = mass * (a_total + params["g"])
            thrust_cmd = max(0.0, min(thrust_cmd, params["T_max"]))
        else:
            thrust_cmd = 0.0

        # thrust rate limit
        max_rate = 200.0
        delta = thrust_cmd - thrust_actual
        delta = max(-max_rate * dt, min(delta, max_rate * dt))
        thrust_actual += delta

        # disturbance (STRONG)
        disturbance = random.uniform(-7.0, 7.0)
        thrust_effective = thrust_actual + disturbance

        # mass flow
        mdot = thrust_actual / (params["Isp"] * params["g"]) if thrust_actual > 0 else 0
        mass -= mdot * dt
        fuel_used += mdot * dt
        mass = max(mass, 1.0)

        # gear
        if h < 0:
            comp = -h
            if comp > params["max_compression"]:
                comp = params["max_compression"]
                state["h"] = -comp
                state["v"] = 0.0

            gear_force = params["k"] * comp - params["c"] * v

            if gear_force > max_force:
                max_force = gear_force
                mass_at_peak = mass
        else:
            gear_force = 0.0

        total_force = thrust_effective + gear_force
        params["m"] = mass

        state = step(state, total_force, dt, params)

        if abs(state["v"]) < 0.05 and state["h"] < 0:
            settle_count += 1
        else:
            settle_count = 0

        if settle_count >= settle_required:
            return True, max_force / (mass_at_peak * params["g"]), fuel_used

        t += dt

    return False, max_force / (mass_at_peak * params["g"]), fuel_used


# -----------------------
# RUN MANY TIMES
# -----------------------
N = 30

success = 0
g_values = []
fuel_values = []

print("\nRunning Monte Carlo...\n")

for i in range(N):
    ok, g, fuel = run_sim()
    if ok:
        success += 1
    g_values.append(g)
    fuel_values.append(fuel)

print("====== RESULTS ======\n")
print(f"Success Rate : {success}/{N}")
print(f"Avg Peak g   : {sum(g_values)/N:.2f}")
print(f"Max Peak g   : {max(g_values):.2f}")
print(f"Avg Fuel     : {sum(fuel_values)/N:.2f}")
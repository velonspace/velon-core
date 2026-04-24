from core.dynamics.vertical_model import step
import random
import statistics as stats
import csv

# -----------------------
# BASELINE CONFIG
# -----------------------
BASELINE = {
    "k_v": 0.8,
    "k_h": 0.3,
    "k_v_final": 2.2,
    "target_v": -0.35,
    "lag_factor": 1.8,
    "safety_margin": 5.0
}

# -----------------------
# BASE SIMULATION
# -----------------------
def run_sim(disturbance_level=2.0, noise_level=0.1, thrust_rate=200.0):

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

    settle_count = 0
    settle_required = 50

    max_force = 0.0
    mass_at_peak = mass

    burn_started = False
    t = 0.0

    while t < t_final:

        h, v = state["h"], state["v"]

        # -----------------------
        # SENSOR NOISE
        # -----------------------
        measured_h = h + random.uniform(-noise_level, noise_level)
        measured_v = v + random.uniform(-noise_level, noise_level)

        # -----------------------
        # LAG-AWARE TRIGGER
        # -----------------------
        a_max = params["T_max"] / mass - params["g"]

        if a_max > 0:
            d_stop = (v**2) / (2 * a_max)
        else:
            d_stop = float("inf")

        trigger_height = BASELINE["lag_factor"] * d_stop + BASELINE["safety_margin"]

        if measured_h <= trigger_height:
            burn_started = True

        # -----------------------
        # CONTROL
        # -----------------------
        if burn_started:

            if measured_h < 0.5:
                a_total = -BASELINE["k_v_final"] * (measured_v - BASELINE["target_v"])
            else:
                if measured_h > 1e-3:
                    a_physics = (measured_v**2) / (2 * measured_h)
                else:
                    a_physics = 0.0

                a_total = (
                    a_physics
                    - BASELINE["k_v"] * measured_v
                    - BASELINE["k_h"] * measured_h
                )

            thrust_cmd = mass * (a_total + params["g"])
            thrust_cmd = max(0.0, min(thrust_cmd, params["T_max"]))
        else:
            thrust_cmd = 0.0

        # -----------------------
        # THRUST RATE LIMIT
        # -----------------------
        delta = thrust_cmd - thrust_actual
        delta = max(-thrust_rate * dt, min(delta, thrust_rate * dt))
        thrust_actual += delta

        # -----------------------
        # DISTURBANCE
        # -----------------------
        disturbance = random.uniform(-disturbance_level, disturbance_level)
        thrust_effective = thrust_actual + disturbance

        # -----------------------
        # MASS FLOW
        # -----------------------
        mdot = thrust_actual / (params["Isp"] * params["g"]) if thrust_actual > 0 else 0
        mass -= mdot * dt
        fuel_used += mdot * dt
        mass = max(mass, 1.0)

        # -----------------------
        # LANDING GEAR
        # -----------------------
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

        # -----------------------
        # LANDING CHECK
        # -----------------------
        if abs(state["v"]) < 0.05 and state["h"] < 0:
            settle_count += 1
        else:
            settle_count = 0

        if settle_count >= settle_required:
            peak_g = max_force / (mass_at_peak * params["g"])
            return True, peak_g, fuel_used

        t += dt

    return False, 999, fuel_used


# -----------------------
# MONTE CARLO
# -----------------------
def monte_carlo(N, disturbance, noise, thrust_rate):

    results = [run_sim(disturbance, noise, thrust_rate) for _ in range(N)]

    success = sum(1 for r in results if r[0])
    g_vals = [r[1] for r in results]
    fuel_vals = [r[2] for r in results]

    print("\n------ RESULTS ------\n")
    print(f"Success Rate : {success}/{N}")
    print(f"Avg g        : {sum(g_vals)/N:.2f}")
    print(f"Max g        : {max(g_vals):.2f}")
    print(f"Min g        : {min(g_vals):.2f}")
    print(f"Std g        : {stats.pstdev(g_vals):.2f}")
    print(f"Avg Fuel     : {sum(fuel_vals)/N:.2f}")
    print(f"Max Fuel     : {max(fuel_vals):.2f}")
    print(f"Std Fuel     : {stats.pstdev(fuel_vals):.3f}")

    # -----------------------
    # PASS / FAIL
    # -----------------------
    g_limit = 5.0
    violations = sum(1 for g in g_vals if g > g_limit)

    print(f"\nG-limit violations (> {g_limit}g): {violations}/{N}")
    print("\nPASS/FAIL:")
    print(f"Reliability OK : {success == N}")
    print(f"G-limit OK     : {max(g_vals) <= g_limit}")

    # -----------------------
    # CSV LOGGING
    # -----------------------
    with open("m4_results.csv", "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            N, disturbance, noise, thrust_rate,
            sum(g_vals)/N, max(g_vals), sum(fuel_vals)/N
        ])


# -----------------------
# TEST SUITE
# -----------------------
print("\n🚀 M4 FINAL TEST SUITE\n")

print("\n1️⃣ Nominal Case")
monte_carlo(20, disturbance=2.0, noise=0.1, thrust_rate=200)

print("\n2️⃣ High Disturbance")
monte_carlo(20, disturbance=7.0, noise=0.1, thrust_rate=200)

print("\n3️⃣ High Sensor Noise")
monte_carlo(20, disturbance=2.0, noise=0.2, thrust_rate=200)

print("\n4️⃣ Slow Engine Response")
monte_carlo(20, disturbance=2.0, noise=0.1, thrust_rate=120)

print("\n5️⃣ Extreme Stress Test")
monte_carlo(30, disturbance=10.0, noise=0.2, thrust_rate=120)
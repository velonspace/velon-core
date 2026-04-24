import math
import random

# -----------------------
# PARAMETERS
# -----------------------
params = {
    "m": 5.0,
    "g": 9.81,
    "T_max": 120.0,
    "k": 800.0,
    "c": 120.0,
    "max_compression": 0.2
}

dt = 0.01
t_final = 30.0


# -----------------------
# ADAPTIVE LIMIT
# -----------------------
def get_accel_limit(h):
    if h > 40:
        return 8.0
    elif h > 15:
        return 5.0
    elif h > 5:
        return 3.0
    else:
        return 2.0


# -----------------------
# SINGLE SIMULATION
# -----------------------
def run_sim():

    state = {"x": 20.0, "h": 100.0, "vx": 0.0, "vh": 0.0}
    max_g = 0

    wind_bias = random.uniform(-2, 2)
    sensor_bias = random.uniform(-0.1, 0.1)

    t = 0.0
    while t < t_final:

        # SENSOR
        x = state["x"] + sensor_bias + random.uniform(-0.05, 0.05)
        h = state["h"] + random.uniform(-0.05, 0.05)
        vx = state["vx"] + random.uniform(-0.02, 0.02)
        vh = state["vh"] + random.uniform(-0.02, 0.02)

        m = params["m"]
        g = params["g"]

        on_ground = state["h"] < 0

        # -----------------------
        # COMMIT LOGIC
        # -----------------------
        commit = h < 8

        # -----------------------
        # HORIZONTAL CONTROL
        # -----------------------
        if on_ground:
            ax_cmd = -2.0 * x - 1.5 * vx
        elif commit:
            ax_cmd = -1.0 * x - 1.5 * vx
        else:
            if h > 12:
                ax_cmd = -0.35 * x - 1.0 * vx
            elif h > 5:
                ax_cmd = -0.20 * x - 0.7 * vx
            else:
                ax_cmd = -0.08 * x - 1.2 * vx

        theta = math.atan2(ax_cmd, g)
        theta = max(min(theta, 0.35), -0.35)

        # -----------------------
        # VERTICAL CONTROL (SMOOTHED)
        # -----------------------
        if h < 3.0:
            # 🔥 smoother final descent
            v_target = -0.8
            k_v = 3.5

        elif commit:
            v_target = -0.7
            k_v = 4.0

        else:
            if h > 20:
                v_target = -8.0
            elif h > 10:
                v_target = -5.0
            elif h > 5:
                v_target = -3.0
            elif h > 2:
                v_target = -1.5
            else:
                v_target = -0.5

            if h > 10:
                k_v = 1.8
            elif h > 3:
                k_v = 2.8
            else:
                k_v = 4.0

        v_error = vh - v_target
        a_damp = -0.6 * vh
        ah_cmd = -k_v * v_error + a_damp

        # -----------------------
        # THRUST
        # -----------------------
        thrust = m * (ah_cmd + g)
        thrust *= random.uniform(0.95, 1.05)
        thrust = max(0.0, min(thrust, params["T_max"]))

        Fx = thrust * math.sin(theta)
        Fh = thrust * math.cos(theta)

        # -----------------------
        # WIND
        # -----------------------
        wind_bias += random.uniform(-0.2, 0.2)
        if random.random() < 0.1:
            Fx += random.uniform(-5, 5)

        # -----------------------
        # LANDING GEAR
        # -----------------------
        if state["h"] < 0:
            comp = -state["h"]

            if comp > params["max_compression"]:
                comp = params["max_compression"]
                state["h"] = -comp
                state["vh"] = 0.0

            gear_force = params["k"] * comp - params["c"] * state["vh"]
        else:
            gear_force = 0.0

        # -----------------------
        # REAL ACCEL (g)
        # -----------------------
        ax_real = Fx / m
        ah_real = (Fh + gear_force) / m - g

        g_load = abs(ah_real + g) / g
        max_g = max(max_g, g_load)

        # -----------------------
        # SAFE ACCEL
        # -----------------------
        limit = get_accel_limit(state["h"])
        ax = max(min(ax_real, limit), -limit)
        ah = max(min(ah_real, limit), -limit)

        # -----------------------
        # INTEGRATION
        # -----------------------
        state["vx"] += ax * dt
        state["vh"] += ah * dt

        state["x"] += state["vx"] * dt
        state["h"] += state["vh"] * dt

        # -----------------------
        # SUCCESS CONDITION
        # -----------------------
        if (
            state["h"] < 0
            and abs(state["vx"]) < 0.1
            and abs(state["vh"]) < 0.1
            and abs(state["x"]) < 1.0
        ):
            return True, max_g, state, t, "success"

        t += dt

    # -----------------------
    # FAILURE CLASSIFICATION
    # -----------------------
    reason = "timeout"

    if abs(state["x"]) > 5:
        reason = "horizontal drift"
    elif state["h"] > 1:
        reason = "never descended"
    elif abs(state["vh"]) > 1:
        reason = "too fast vertical"
    elif abs(state["vx"]) > 1:
        reason = "too fast horizontal"

    return False, max_g, state, t, reason


# -----------------------
# MONTE CARLO
# -----------------------
def monte_carlo(runs=10):

    success = 0
    g_vals = []

    print("\n====== M5.1 FINAL (SMOOTH LANDING) ======\n")

    for i in range(runs):

        ok, g, state, t, reason = run_sim()

        status = "SUCCESS" if ok else "FAIL"

        print(f"\nRun {i+1}: {status}")
        print(f"  Reason       : {reason}")
        print(f"  Final x      : {state['x']:.2f}")
        print(f"  Final h      : {state['h']:.2f}")
        print(f"  Final vx     : {state['vx']:.2f}")
        print(f"  Final vh     : {state['vh']:.2f}")
        print(f"  Peak g       : {g:.2f}")
        print(f"  Time         : {t:.2f}")

        if ok:
            success += 1

        g_vals.append(g)

    print("\n====== SUMMARY ======\n")
    print(f"Success Rate : {success}/{runs}")
    print(f"Avg g        : {sum(g_vals)/len(g_vals):.2f}")
    print(f"Max g        : {max(g_vals):.2f}")
    print(f"Min g        : {min(g_vals):.2f}")


# -----------------------
# RUN
# -----------------------
if __name__ == "__main__":
    print("\n🚀 M5.1 FINAL: Smoothed Precision Landing System\n")
    monte_carlo()
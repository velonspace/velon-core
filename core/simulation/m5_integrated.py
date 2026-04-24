import math
import random

# -----------------------
# PARAMETERS
# -----------------------
params = {
    "m0": 5.0,
    "g": 9.81,
    "T_max": 120.0,
    "fuel_rate": 0.02,

    "k": 800.0,
    "c": 120.0,
    "max_compression": 0.2
}

dt = 0.01
t_final = 30.0


def get_accel_limit(h):
    if h > 40:
        return 8.0
    elif h > 15:
        return 5.0
    elif h > 5:
        return 3.0
    else:
        return 2.0


def run_sim():

    state = {
        "x": 20.0,
        "h": 100.0,
        "vx": 0.0,
        "vh": 0.0,
        "m": params["m0"]
    }

    trajectory = []

    max_g = 0
    fuel_used = 0

    wind_bias = random.uniform(-2, 2)
    sensor_bias = random.uniform(-0.1, 0.1)

    t = 0.0
    while t < t_final:

        # LOG TRAJECTORY
        trajectory.append({
            "x": state["x"],
            "h": state["h"]
        })

        # SENSOR
        x = state["x"] + sensor_bias + random.uniform(-0.05, 0.05)
        h = state["h"] + random.uniform(-0.05, 0.05)
        vx = state["vx"] + random.uniform(-0.02, 0.02)
        vh = state["vh"] + random.uniform(-0.02, 0.02)

        m = state["m"]
        g = params["g"]

        on_ground = state["h"] < 0
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
        # VERTICAL CONTROL
        # -----------------------
        if h < 3.0:
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

        # THRUST
        thrust = m * (ah_cmd + g)
        thrust *= random.uniform(0.95, 1.05)
        thrust = max(0.0, min(thrust, params["T_max"]))

        # FUEL
        fuel_flow = params["fuel_rate"] * (thrust / params["T_max"])
        state["m"] -= fuel_flow * dt
        fuel_used += fuel_flow * dt

        if state["m"] < 2.0:
            state["m"] = 2.0

        # FORCES
        Fx = thrust * math.sin(theta)
        Fh = thrust * math.cos(theta)

        if random.random() < 0.1:
            Fx += random.uniform(-5, 5)

        # GEAR
        if state["h"] < 0:
            comp = -state["h"]
            if comp > params["max_compression"]:
                comp = params["max_compression"]
                state["h"] = -comp
                state["vh"] = 0.0

            gear_force = params["k"] * comp - params["c"] * state["vh"]
        else:
            gear_force = 0.0

        # ACCEL
        ax_real = Fx / state["m"]
        ah_real = (Fh + gear_force) / state["m"] - g

        g_load = abs(ah_real + g) / g
        max_g = max(max_g, g_load)

        limit = get_accel_limit(state["h"])
        ax = max(min(ax_real, limit), -limit)
        ah = max(min(ah_real, limit), -limit)

        # INTEGRATE
        state["vx"] += ax * dt
        state["vh"] += ah * dt

        state["x"] += state["vx"] * dt
        state["h"] += state["vh"] * dt

        # SUCCESS
        if (
            state["h"] < 0
            and abs(state["vx"]) < 0.1
            and abs(state["vh"]) < 0.1
            and abs(state["x"]) < 1.0
        ):
            return True, max_g, fuel_used, state, t, trajectory

        t += dt

    return False, max_g, fuel_used, state, t, trajectory
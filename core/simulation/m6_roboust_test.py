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
# GAINS
# -----------------------
# vertical (M5)
kph = 0.3
kvh = 0.8
k_v_final = 2.5
target_vh = -0.3

# guidance (M6)
kx = 0.5
kv = 1.0


# -----------------------
# SINGLE RUN SIMULATION
# -----------------------
def run_sim():

    # randomized initial conditions
    state = {
        "x": random.uniform(-30, 30),
        "h": random.uniform(80, 120),
        "vx": random.uniform(-2, 2),
        "vh": random.uniform(-2, 0),
        "theta": 0.0
    }

    t = 0.0
    max_g = 0.0

    while t < t_final:

        x = state["x"]
        h = state["h"]
        vx = state["vx"]
        vh = state["vh"]

        m = params["m"]
        g = params["g"]

        # =======================
        # M6 GUIDANCE
        # =======================
        target_vx = max(min(-kx * x, 5.0), -5.0)
        ax_cmd = kv * (target_vx - vx)

        theta = math.atan2(ax_cmd, g)
        theta = max(min(theta, 0.35), -0.35)

        # =======================
        # M5 VERTICAL
        # =======================
        if h > 2:
            ah_cmd = -kvh * vh - kph * h
        else:
            ah_cmd = -k_v_final * (vh - target_vh)

        thrust = m * (ah_cmd + g)
        thrust = max(0.0, min(thrust, params["T_max"]))

        Fx = thrust * math.sin(theta)
        Fh = thrust * math.cos(theta)

        # =======================
        # LANDING GEAR
        # =======================
        if h < 0:
            comp = -h

            if comp > params["max_compression"]:
                comp = params["max_compression"]
                h = -comp
                vh = 0.0

            gear_force = params["k"] * comp - params["c"] * vh
        else:
            gear_force = 0.0

        # =======================
        # DYNAMICS
        # =======================
        ax = Fx / m
        ah = (Fh + gear_force) / m - g

        # track g-force
        total_acc = math.sqrt(ax**2 + ah**2)
        g_force = total_acc / g
        max_g = max(max_g, g_force)

        vx += ax * dt
        vh += ah * dt

        x += vx * dt
        h += vh * dt

        state = {
            "x": x,
            "h": h,
            "vx": vx,
            "vh": vh,
            "theta": theta
        }

        # success condition
        success = (
            abs(x) < 1.0 and
            abs(vx) < 0.1 and
            abs(vh) < 0.1 and
            h < 0
        )

        if success:
            return True, state, max_g, t

        t += dt

    return False, state, max_g, t


# -----------------------
# MONTE CARLO
# -----------------------
def monte_carlo(n_runs=20):

    results = []

    print("\n🚀 M6.1: Robustness Test\n")

    for i in range(n_runs):

        success, state, max_g, t = run_sim()

        result = {
            "success": success,
            "state": state,
            "max_g": max_g,
            "time": t
        }

        results.append(result)

        # -----------------------
        # PER RUN OUTPUT
        # -----------------------
        print(f"\nRun {i+1}: {'SUCCESS' if success else 'FAIL'}")
        print(f"  Final x  : {state['x']:.2f}")
        print(f"  Final h  : {state['h']:.2f}")
        print(f"  vx       : {state['vx']:.2f}")
        print(f"  vh       : {state['vh']:.2f}")
        print(f"  Peak g   : {max_g:.2f}")
        print(f"  Time     : {t:.2f}")

    # -----------------------
    # SUMMARY
    # -----------------------
    successes = sum(1 for r in results if r["success"])
    avg_g = sum(r["max_g"] for r in results) / n_runs
    max_g = max(r["max_g"] for r in results)
    min_g = min(r["max_g"] for r in results)

    print("\n====== SUMMARY ======")
    print(f"Success Rate : {successes}/{n_runs}")
    print(f"Avg g        : {avg_g:.2f}")
    print(f"Max g        : {max_g:.2f}")
    print(f"Min g        : {min_g:.2f}")


# -----------------------
# RUN
# -----------------------
if __name__ == "__main__":
    monte_carlo(20)
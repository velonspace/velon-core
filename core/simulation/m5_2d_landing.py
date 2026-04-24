import math

# -----------------------
# PARAMETERS
# -----------------------
params = {
    "m": 5.0,
    "g": 9.81,
    "T_max": 100.0,

    # landing gear
    "k": 800.0,
    "c": 120.0,
    "max_compression": 0.2
}

dt = 0.01
t_final = 30.0

state = {
    "x": 20.0,
    "h": 100.0,
    "vx": 0.0,
    "vh": 0.0,
    "theta": 0.0
}

t = 0.0

print("\n🚀 M5 FINAL: Precision Landing (with ground correction)\n")

# -----------------------
# GAINS
# -----------------------
kph = 0.3
kvh = 0.8
k_v_final = 2.5
target_vh = -0.3

while t < t_final:

    x = state["x"]
    h = state["h"]
    vx = state["vx"]
    vh = state["vh"]

    m = params["m"]
    g = params["g"]

    on_ground = h < 0

    # -----------------------
    # HORIZONTAL CONTROL
    # -----------------------
    if not on_ground:

        if h > 5:
            ax_cmd = -0.30 * x - 0.9 * vx
        elif h > 1:
            ax_cmd = -0.20 * x - 0.7 * vx
        else:
            ax_cmd = -0.10 * x - 1.5 * vx

    else:
        # 🔥 NEW: ground correction
        ax_cmd = -1.0 * x - 2.0 * vx

    # tilt
    theta = math.atan2(ax_cmd, g)
    theta = max(min(theta, 0.3), -0.3)

    # -----------------------
    # VERTICAL CONTROL
    # -----------------------
    if h > 2:
        ah_cmd = -kvh * vh - kph * h
    else:
        ah_cmd = -k_v_final * (vh - target_vh)

    thrust = m * (ah_cmd + g)
    thrust = max(0.0, min(thrust, params["T_max"]))

    Fx = thrust * math.sin(theta)
    Fh = thrust * math.cos(theta)

    # -----------------------
    # LANDING GEAR
    # -----------------------
    if h < 0:
        comp = -h

        if comp > params["max_compression"]:
            comp = params["max_compression"]
            h = -comp
            vh = 0.0

        gear_force = params["k"] * comp - params["c"] * vh

    else:
        gear_force = 0.0

    # -----------------------
    # DYNAMICS
    # -----------------------
    ax = Fx / m
    ah = (Fh + gear_force) / m - g

    vx += ax * dt
    vh += ah * dt

    x += vx * dt
    h += vh * dt

    # snap small values
    if abs(vx) < 0.01:
        vx = 0.0
    if abs(x) < 0.05:
        x = 0.0

    state = {
        "x": x,
        "h": h,
        "vx": vx,
        "vh": vh,
        "theta": theta
    }

    # debug
    if int(t * 100) % 50 == 0:
        print(f"t={t:.2f}, x={x:.2f}, h={h:.2f}, vx={vx:.2f}, vh={vh:.2f}, theta={theta:.2f}")

    # -----------------------
    # LANDING CONDITION
    # -----------------------
    if on_ground and abs(vx) < 0.01 and abs(vh) < 0.05 and abs(x) < 0.05:
        print("\n🟢 PERFECT LANDING (x ≈ 0)")
        break

    t += dt

print("\nFinal state:", state)
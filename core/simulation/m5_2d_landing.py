import math

# -----------------------
# PARAMETERS
# -----------------------
params = {
    "m": 5.0,
    "g": 9.81,
    "T_max": 120.0,

    # landing gear
    "k": 800.0,
    "c": 120.0,
    "max_compression": 0.2
}

dt = 0.01
t_final = 30.0

# -----------------------
# INITIAL STATE
# -----------------------
state = {
    "x": 20.0,
    "h": 100.0,
    "vx": 0.0,
    "vh": 0.0,
    "theta": 0.0
}

t = 0.0

print("\n🚀 M6: Guidance + Control (Stable v1.1)\n")

# -----------------------
# M5 VERTICAL GAINS (UNCHANGED)
# -----------------------
kph = 0.3
kvh = 0.8
k_v_final = 2.5
target_vh = -0.3

# -----------------------
# M6 GUIDANCE GAINS
# -----------------------
kx = 0.5
kv = 1.0

# -----------------------
# MAIN LOOP
# -----------------------
while t < t_final:

    x = state["x"]
    h = state["h"]
    vx = state["vx"]
    vh = state["vh"]

    m = params["m"]
    g = params["g"]

    # =======================
    # M6 GUIDANCE (HORIZONTAL)
    # =======================
    # velocity target (capped for stability)
    target_vx = max(min(-kx * x, 5.0), -5.0)

    # track velocity
    ax_cmd = kv * (target_vx - vx)

    # convert to tilt
    theta = math.atan2(ax_cmd, g)

    # tilt limit
    theta = max(min(theta, 0.35), -0.35)

    # =======================
    # M5 VERTICAL CONTROL (UNCHANGED)
    # =======================
    if h > 2:
        ah_cmd = -kvh * vh - kph * h
    else:
        ah_cmd = -k_v_final * (vh - target_vh)

    # =======================
    # THRUST
    # =======================
    thrust = m * (ah_cmd + g)
    thrust = max(0.0, min(thrust, params["T_max"]))

    # =======================
    # FORCE RESOLUTION
    # =======================
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

    # =======================
    # DEBUG PRINT
    # =======================
    if int(t * 100) % 50 == 0:
        print(f"t={t:.2f}, x={x:.2f}, h={h:.2f}, vx={vx:.2f}, vh={vh:.2f}, theta={theta:.2f}")

    # =======================
    # SUCCESS CONDITION
    # =======================
    success = (
        abs(x) < 1.0 and
        abs(vx) < 0.1 and
        abs(vh) < 0.1 and
        h < 0
    )

    if success:
        print("\n🟢 Successful landing (M6 stable)")
        break

    t += dt

# -----------------------
# FINAL STATE
# -----------------------
print("\nFinal state:", state)
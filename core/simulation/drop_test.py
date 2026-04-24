from core.dynamics.vertical_model import step

# -----------------------
# Parameters
# -----------------------
params = {
    "m": 1.0,
    "g": 9.81,
    "T_max": 0.0,   # OFF → pure drop test

    # Landing gear
    "k": 800.0,
    "c": 150.0
}

dt = 0.01
t_final = 10.0

# -----------------------
# Initial State (drop)
# -----------------------
state = {
    "h": 10.0,   # drop from 10 m
    "v": 0.0
}

t = 0.0

# -----------------------
# Metrics
# -----------------------
max_force = 0.0
max_compression = 0.0

print("M3: Drop test (structures focus)\n")

while t < t_final:

    h = state["h"]
    v = state["v"]

    m = params["m"]
    g = params["g"]

    # -----------------------
    # Landing gear force
    # -----------------------
    k = params["k"]
    c = params["c"]

    if h < 0:
        compression = -h
        gear_force = k * compression - c * v

        # track metrics
        if gear_force > max_force:
            max_force = gear_force
        if compression > max_compression:
            max_compression = compression
    else:
        gear_force = 0.0

    # total force = gear only (no thrust)
    total_force = gear_force

    # step
    state = step(state, total_force, dt, params)

    # debug
    if int(t * 100) % 50 == 0:
        print(f"t={t:.2f}, h={h:.2f}, v={v:.2f}, gearF={gear_force:.2f}")

    # stop when settled
    if abs(v) < 0.05 and h < 0:
        print("\nSettled after impact")
        break

    t += dt

# -----------------------
# Results
# -----------------------
print("\nFinal state:", state)
print("Max compression:", max_compression)
print("Max gear force:", max_force)
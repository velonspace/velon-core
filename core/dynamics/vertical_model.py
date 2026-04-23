def step(state, thrust, dt, params):
    h = state["h"]
    v = state["v"]

    m = params["m"]
    g = params["g"]

    # Correct physics: thrust up, gravity down
    a = (thrust / m) - g

    # Update velocity
    v_new = v + a * dt

    # Update position (with acceleration term)
    h_new = h + v * dt + 0.5 * a * dt**2

    return {"h": h_new, "v": v_new}
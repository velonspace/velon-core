# M5 CONFIG — LOCKED PARAMETERS

CONFIG = {

    # Physics
    "g": 9.81,
    "m0": 5.0,
    "T_max": 120.0,
    "fuel_rate": 0.02,

    # Landing Gear
    "k": 800.0,
    "c": 120.0,
    "max_compression": 0.2,

    # Control Limits
    "theta_limit": 0.35,
    "accel_limits": {
        "high": 8.0,
        "mid": 5.0,
        "low": 3.0,
        "final": 2.0
    },

    # Acceptance Criteria
    "landing": {
        "x_tol": 1.0,
        "vx_tol": 0.1,
        "vh_tol": 0.1,
        "g_limit": 5.0
    }
}
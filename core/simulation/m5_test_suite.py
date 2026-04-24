import json
from core.simulation.m5_integrated import run_sim

def run_tests(runs=20):

    results = []
    success_count = 0

    for i in range(runs):

        success, g, fuel, state, t, trajectory = run_sim()

        run_data = {
            "run": i + 1,
            "success": success,
            "final_x": state["x"],
            "final_h": state["h"],
            "vx": state["vx"],
            "vh": state["vh"],
            "mass": state["m"],
            "g": g,
            "fuel": fuel,
            "time": t,
            "trajectory": trajectory
        }

        results.append(run_data)

        if success:
            success_count += 1

    summary = {
        "success_rate": success_count,
        "total_runs": runs,
        "avg_g": sum(r["g"] for r in results) / runs,
        "max_g": max(r["g"] for r in results),
        "min_g": min(r["g"] for r in results),
        "avg_fuel": sum(r["fuel"] for r in results) / runs
    }

    data = {
        "summary": summary,
        "runs": results
    }

    with open("dashboard/results.json", "w") as f:
        json.dump(data, f, indent=2)

    print("\n✅ Results saved to dashboard/results.json")


if __name__ == "__main__":
    print("\n🚀 Running M5 Test Suite...\n")
    run_tests(30)
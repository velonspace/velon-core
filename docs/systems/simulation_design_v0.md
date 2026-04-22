# Simulation Design (v0)

## 1. Simulation Flow

Initialize state

Loop over time:

* compute thrust (control input)
* compute acceleration (physics)
* update velocity
* update height
* log data

Terminate when:

* height ≤ 0 (landing)
* OR time exceeds limit

---

## 2. Data Structures

### State

```
state = {
    h: height (m),
    v: velocity (m/s)
}
```

### Control Input

```
T = thrust (N)
```

### Parameters

```
params = {
    m: mass (kg),
    g: gravity (9.81 m/s²),
    T_max: maximum thrust (N),
    dt: time step (s)
}
```

### Log (Flight Recorder)

```
log = {
    time: [],
    h: [],
    v: [],
    T: []
}
```

---

## 3. Time Strategy

* Discrete time simulation
* Time progresses in steps of dt

---

## 4. Simulation Loop Logic

At each timestep:

```
a = (T / m) - g
v = v + a * dt
h = h + v * dt
```

---

## 5. Control Input (Initial)

* Thrust is defined externally
* Can be:

  * constant value
  * simple rule-based logic

(No feedback control yet)

---

## 6. Simulation Scenarios

1. Free Fall

   * T = 0

2. Hover

   * T = m * g

3. Controlled Descent

   * T < m * g

---

## 7. Outputs

Each simulation run should produce:

* height vs time
* velocity vs time
* thrust vs time

---

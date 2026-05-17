# MPC Heating Controller — Raspberry Pi

> An IoT-based smart heating controller that uses **Model Predictive Control (MPC)** to minimise electricity costs while keeping indoor temperature within a defined comfort range, using real-time Finnish electricity prices from ENTSO-E.

---

## Overview

This project implements a closed-loop heating control system running entirely on a Raspberry Pi. It combines a physical temperature sensor, live electricity market data, and an optimisation-based controller to make smarter heating decisions than a traditional rule-based thermostat.

The core idea: instead of simply turning the heater on when it gets cold and off when it gets warm, the MPC looks ahead at the next hours of electricity prices and pre-heats during cheap periods so it can reduce power during expensive ones — all while keeping the indoor temperature within a defined comfort band.

This project was developed as part of an IoT course.

---

## How It Works

```
[ENTSO-E API] ── fetch prices ──┐
                                ▼
[DHT11 sensor] ── T_in ──► [Raspberry Pi]
                                │
                           MPC runs here
                          (every 15 min)
                                │
                 ┌──────────────┴──────────────┐
                 ▼                             ▼
          [LED via GPIO]               [Terminal log]
          (heating output)             (T, price, power)
```

Every 15 minutes the controller:
1. Reads the current indoor temperature from the DHT11 sensor
2. Fetches the day-ahead electricity price from ENTSO-E
3. Solves an MPC optimisation to find the heating power that minimises cost while respecting temperature boundaries
4. Applies the result to the LED via PWM
5. Holds that power level for the full 15-minute interval

---

## Hardware

| Component | Description |
|---|---|
| Raspberry Pi 4B | Main compute unit — runs Python and the MPC |
| DHT11 sensor | Measures indoor temperature |
| LED | Represents the heating load (controlled via PWM) |
| 220Ω resistor | Current limiting resistor for the LED |
| Breadboard + jumper wires | Circuit assembly |

### Wiring

| Component | Pi Pin |
|---|---|
| DHT11 VCC | Pin 1 (3.3V) |
| DHT11 GND | Pin 6 (GND) |
| DHT11 DATA | Pin 7 (GPIO 4) |
| LED anode (+) | Pin 12 (GPIO 18, PWM) |
| LED cathode (−) | GND via 220Ω resistor |

---

## Software

### Requirements

```bash
sudo apt update
pip3 install adafruit-circuitpython-dht RPi.GPIO numpy scipy pandas entsoe-py
```

## The MPC Controller

### What is MPC?

Model Predictive Control is a control strategy that, at each time step:
1. Uses a mathematical model of the system to **predict future behaviour**
2. Solves an **optimisation problem** to find the best sequence of actions over a prediction horizon
3. Applies only the **first action** and repeats the process at the next step (receding horizon)

This allows the controller to anticipate future conditions — like a price spike in two hours — and act proactively rather than reactively.

### Thermal Model

The indoor temperature evolves according to:

```
dT/dt = (1/C) × (p × P_MAX − k × (T_in − T_outdoor))
```

| Parameter | Description | Value |
|---|---|---|
| `C` | Thermal capacitance (how slowly the room heats up) | 10.0 |
| `k` | Heat loss coefficient (how fast heat escapes) | 1.5 |
| `P_MAX` | Maximum heating power | 50.0 |
| `dt` | Time step | 15 min |
| `HORIZON` | Prediction horizon | 12 steps (3 hours) |

`P_MAX = 50` is sized for the worst-case scenario (outdoor temperature of −10°C), ensuring the heater can always maintain `T_MIN` regardless of conditions.

### Optimisation

At each step the MPC minimises:

```
cost = Σ p[i] × P_MAX × price[i] × dt
```

Subject to hard constraints:

```
T_MIN ≤ T[i] ≤ T_MAX   for all i in horizon
0 ≤ p[i] ≤ 1           for all i in horizon
```

Hard constraints are used to guarantee the temperature boundaries are never violated. If the optimiser fails to find a feasible solution, a thermostat fallback ensures safe operation.

---

**Key finding:** MPC savings are highest in mild conditions where the system has scheduling flexibility. In extreme cold, the heater must run continuously to maintain comfort, leaving little room for price-based optimisation — both controllers converge to similar behaviour.

---

## Project Structure

```
├── mpc_controller.py           # main controller — runs on Raspberry Pi
├── MPC_Simulation.ipynb        # simulation notebook — runs on laptop
├── test_get_energy_price.py    # test file for fetching energy price
├── test_led.py                 # test file to testing led
└── README.md                   # this file
```

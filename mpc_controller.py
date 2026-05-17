import numpy as np
import pandas as pd
import adafruit_dht
import board
import RPi.GPIO as GPIO
import time
from scipy.optimize import minimize
from entsoe import EntsoePandasClient
from datetime import datetime, timezone

# ── User-defined settings ──────────────────────────────────────────────────
API_KEY     = "YOUR_ENTSOE_API_KEY"
T_MIN       = 22.0   # minimum allowed temperature (°C)
T_MAX       = 26.0   # maximum allowed temperature (°C)
T_OUTDOOR   = 18.0   # outdoor temperature (°C) — user defined for now

# ── Thermal model parameters ───────────────────────────────────────────────
C     = 10.0   # thermal capacitance
k     = 1.5    # heat loss coefficient
P_MAX = 50.0    # max heating power

# ── MPC settings ───────────────────────────────────────────────────────────
dt      = 15 / 60   # time step: 15 minutes in hours
HORIZON = 4          # look 4 steps ahead = 1 hour

# ── Hardware setup ─────────────────────────────────────────────────────────
DHT_PIN = board.D4
LED_PIN = 18

dht = adafruit_dht.DHT11(DHT_PIN)
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
pwm = GPIO.PWM(LED_PIN, 1000)
pwm.start(0)

# ── Fetch electricity prices from ENTSO-E ──────────────────────────────────
def fetch_prices():
    """
    Fetch today's day-ahead prices from ENTSO-E.
    Returns a DataFrame with 15-min resolution timestamps and prices in €/kWh.
    """
    print("Fetching electricity prices from ENTSO-E...")
    client = EntsoePandasClient(api_key=API_KEY)

    today = pd.Timestamp.now(tz='Europe/Helsinki').normalize()
    tomorrow = today + pd.Timedelta(days=1)

    df = client.query_day_ahead_prices('FI', today, tomorrow).reset_index()
    df.columns = ['timestamp', 'price']
    df['price'] = df['price'] / 1000.0  # €/MWh → €/kWh
    df = df.head(96)  # limit to 24h * 4 = 96 intervals (15 min steps)

    print(f"Fetched {len(df)} price intervals")
    print(f"  Min: {df['price'].min()*100:.2f} c€/kWh  "
          f"Max: {df['price'].max()*100:.2f} c€/kWh  "
          f"Avg: {df['price'].mean()*100:.2f} c€/kWh")
    return df

# ── Get current price from DataFrame ──────────────────────────────────────
def get_current_price_index(prices_df):
    """Find the index in the price DataFrame matching the current time."""
    now = pd.Timestamp.now(tz='Europe/Helsinki').floor('15min')
    matches = prices_df[prices_df['timestamp'] == now]
    if matches.empty:
        print(f"Warning: no price found for {now}, using closest available")
        idx = (prices_df['timestamp'] - now).abs().argmin()
    else:
        idx = matches.index[0]
    return idx

# ── Read temperature from DHT11 ────────────────────────────────────────────
def read_temperature():
    """Read temperature with retries — DHT11 occasionally misreads."""
    for attempt in range(5):
        try:
            temp = dht.temperature
            if temp is not None:
                return float(temp)
        except RuntimeError:
            pass
        time.sleep(2)
    print("Warning: sensor read failed after 5 attempts, using last known value")
    return None

# ── Thermal model (one step forward) ──────────────────────────────────────
def thermal_model(T, p):
    dT = (1 / C) * (p * P_MAX - k * (T - T_OUTDOOR))
    return T + dT * dt

# ── MPC controller ─────────────────────────────────────────────────────────
def run_mpc(T_current, price_forecast):
    """
    Solve the MPC optimisation over the prediction horizon.
    Uses hard constraints to enforce temperature boundaries.
    Returns optimal power for the current 15-min interval (0.0 to 1.0).
    """

    def cost_function(power_sequence):
        T = T_current
        total_cost = 0
        for i, p in enumerate(power_sequence):
            T = thermal_model(T, p)
            total_cost += p * P_MAX * price_forecast[i] * dt
        return total_cost  # no penalty — hard constraints handle it

    # hard constraints: T must stay within [T_MIN, T_MAX] at every step
    constraints = []
    for step in range(HORIZON):
        def make_lower(s):
            def lower(power_sequence):
                T = T_current
                for j in range(s + 1):
                    T = thermal_model(T, power_sequence[j])
                return T - T_MIN  # must be >= 0
            return lower

        def make_upper(s):
            def upper(power_sequence):
                T = T_current
                for j in range(s + 1):
                    T = thermal_model(T, power_sequence[j])
                return T_MAX - T  # must be >= 0
            return upper

        constraints.append({'type': 'ineq', 'fun': make_lower(step)})
        constraints.append({'type': 'ineq', 'fun': make_upper(step)})

    # warm start based on current temperature
    if T_current < T_MIN:
        p0 = np.ones(HORIZON)
    elif T_current > T_MAX:
        p0 = np.zeros(HORIZON)
    else:
        p0 = np.full(HORIZON, 0.3)

    result = minimize(
        cost_function,
        p0,
        method='SLSQP',
        bounds=[(0, 1)] * HORIZON,
        constraints=constraints,
        options={'maxiter': 200, 'ftol': 1e-8}
    )

    # safety fallback: if optimiser failed, use thermostat logic
    if not result.success:
        if T_current < T_MIN:
            return 1.0
        elif T_current > T_MAX:
            return 0.0
        else:
            return 0.3

    return np.clip(result.x[0], 0, 1)

# ── Apply power to LED ─────────────────────────────────────────────────────
def set_led(power_fraction):
    """Set LED brightness via PWM. power_fraction: 0.0 to 1.0."""
    duty_cycle = power_fraction * 100
    pwm.ChangeDutyCycle(duty_cycle)

# ── Main control loop ──────────────────────────────────────────────────────
def main():
    print("=" * 50)
    print("MPC Heating Controller — Raspberry Pi")
    print(f"Target range: {T_MIN}°C – {T_MAX}°C")
    print(f"Outdoor temp: {T_OUTDOOR}°C")
    print(f"Time step: {int(dt*60)} minutes")
    print("=" * 50)

    # fetch prices once at startup
    prices_df = fetch_prices()
    prices_fetch_date = pd.Timestamp.now(tz='Europe/Helsinki').date()

    T_last = None  # last known temperature

    try:
        while True:
            # ── refresh prices if day changed ──────────────────────────
            today = pd.Timestamp.now(tz='Europe/Helsinki').date()
            if today != prices_fetch_date:
                prices_df = fetch_prices()
                prices_fetch_date = today

            # ── read current temperature ───────────────────────────────
            T_current = read_temperature()
            if T_current is None:
                T_current = T_last if T_last is not None else (T_MIN + T_MAX) / 2
            else:
                T_last = T_current

            # ── get price forecast for MPC horizon ─────────────────────
            idx = get_current_price_index(prices_df)
            forecast_end = min(idx + HORIZON, len(prices_df))
            price_forecast = prices_df['price'].values[idx:forecast_end]

            # pad if near end of day
            if len(price_forecast) < HORIZON:
                price_forecast = np.pad(
                    price_forecast,
                    (0, HORIZON - len(price_forecast)),
                    mode='edge'
                )

            # ── run MPC ────────────────────────────────────────────────
            optimal_power = run_mpc(T_current, price_forecast)

            # ── apply to LED ───────────────────────────────────────────
            set_led(optimal_power)

            # ── log current state ──────────────────────────────────────
            current_price = prices_df['price'].values[idx]
            now_str = datetime.now().strftime('%H:%M')
            print(f"\n[{now_str}]")
            print(f"  T_in:    {T_current:.1f}°C")
            print(f"  T_out:   {T_OUTDOOR}°C")
            print(f"  Price:   {current_price*100:.2f} c€/kWh")
            print(f"  Power:   {optimal_power*100:.1f}%")
            print(f"  LED:     {'ON' if optimal_power > 0.05 else 'OFF'} "
                  f"({optimal_power*100:.0f}% brightness)")

            # ── hold this power for the full 15-minute interval ────────
            print(f"  Holding for 15 minutes...")
            time.sleep(15 * 60)

    except KeyboardInterrupt:
        print("\nStopping controller...")
    finally:
        pwm.stop()
        GPIO.cleanup()
        dht.exit()
        print("GPIO cleaned up. Goodbye!")

if __name__ == "__main__":
    main()
import adafruit_dht
import board
import RPi.GPIO as GPIO
import time

# ── Setup ──────────────────────────────────────────────
DHT_PIN = board.D4      # DHT11 data → GPIO 4
LED_PIN = 18            # LED → GPIO 18 (PWM)

dht = adafruit_dht.DHT11(DHT_PIN)

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
pwm = GPIO.PWM(LED_PIN, 1000)  # 1000 Hz frequency
pwm.start(0)

# ── Test loop ──────────────────────────────────────────
print("Testing DHT11 and LED on Raspberry Pi...")
print("Press Ctrl+C to stop\n")

try:
    while True:
        try:
            temperature = dht.temperature
            humidity    = dht.humidity

            print(f"Temperature: {temperature:.1f}°C | Humidity: {humidity}%")

            # scale brightness with temperature (20–30°C → 0–100%)
            brightness = max(0, min(100, (temperature - 20) * 10))
            pwm.ChangeDutyCycle(brightness)
            print(f"LED brightness: {brightness:.0f}%")

        except RuntimeError as e:
            # DHT11 occasionally misreads — just retry
            print(f"Sensor read error (retrying): {e}")

        time.sleep(2)

except KeyboardInterrupt:
    print("\nStopping...")
    pwm.stop()
    GPIO.cleanup()
    dht.exit()
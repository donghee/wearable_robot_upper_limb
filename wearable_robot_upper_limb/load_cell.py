#!/usr/bin/python3

# Use the old GPIO code	
#sudo apt remove python3-rpi.gpio
#sudo apt install python3-rpi-lgpio
#pip install hx711

import RPi.GPIO as GPIO
import time
from hx711 import HX711

print("Wait for calibration...")
measures = []
try:
    hx711 = HX711(
        dout_pin=5,
        pd_sck_pin=6,
        channel='A',
        gain=64
    )

    hx711.reset()   # Before we start, reset the HX711 (not obligate)
    measures = hx711.get_raw_data(times=10)
finally:
    GPIO.cleanup()  # always do a GPIO cleanup in your scripts!

# zero scale
OFFSET = sum(measures)/ len(measures)
print("Calibration fininshed.")
time.sleep(2)

try: 
    hx711 = HX711(
        dout_pin=5,
        pd_sck_pin=6,
        channel='A',
        gain=64
    )

    hx711.reset()   # Before we start, reset the HX711 (not obligate)
    measures = hx711.get_raw_data(times=3)
finally:
    GPIO.cleanup()  # always do a GPIO cleanup in your scripts!

# convert measurement unit value to SI unit(g)
#SCALE_FACTOR = 279 # KNU
SCALE_FACTOR = 100 # DONGHEE
calibrated_measures = [(x-OFFSET)/SCALE_FACTOR for x in measures]
calibrated_measurement = sum(calibrated_measures)/len(calibrated_measures)

print("{}g".format(int(calibrated_measurement)))

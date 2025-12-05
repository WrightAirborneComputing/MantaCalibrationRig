from machine import Pin, ADC
import utime

# Onboard LED (works on Pico & Pico W)
led = Pin("LED", Pin.OUT)

# ADC0 on GPIO26 and ADC1 on GPIO27
adc0 = ADC(26)   # ADC0
adc1 = ADC(27)   # ADC1

while True:
    # Blink LED
    led.toggle()

    # Read raw ADC values
    raw0 = adc0.read_u16()
    raw1 = adc1.read_u16()

    print("[{:d}/{:d}]".format(raw0, raw1))

    utime.sleep(0.1)
# while

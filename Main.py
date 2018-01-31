import RPi.GPIO as IO
import time

IO.setwarnings(False)
IO.setmode(IO.BCM)
IO.setup(6, IO.OUT)

p = IO.PWM(6, IO.OUT)
p.start(0)

while 1:
  for x in rage(50):
    p.ChangeDutyCycle(x)
    time.sleep(0.1)

  for x in rage(50):
    p.ChangeDutyCycle(50-x)
    time.sleep(0.1)

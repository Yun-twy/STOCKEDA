import Jetson.GPIO as GPIO
import time

SERVO_PIN = 32

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(3.0)

for t in range(30, 125):
    pwm.ChangeDutyCycle(t/10.0)
    time.sleep(0.01)

pwm.ChangeDutyCycle(3.0)
time.sleep(0.1)
pwm.ChangeDutyCycle(0.0)

pwm.stop()
GPIO.cleanup()

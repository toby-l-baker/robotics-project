import RPi.GPIO as GPIO
import time

def Setup(*pins):
    servos = []
    GPIO.setmode(GPIO.BOARD)
    # use GPIO.BCM for SOC channel numbers
    # use GPIO.BOARD for printed board numbers
    for pin in pins:
        servo = Servo(pin)
        servos.append(servo)
    return servos

class Servo:

    def __init__(self, pinNum, startAngle=90, frequency=50):
        GPIO.setup(pinNum, GPIO.OUT)
        self.p = GPIO.PWM(pinNum, 50)
        #self.p.start(startAngle*10/18)

    def update_angle(self, angle):
        # Deprecated - do not use
        try:
            self.p.ChangeDutyCycle(angle*10/18)
            #self.p.ChangeDutyCycle(10)
            time.sleep(angle*0.2/60*1.1) # Turning speed from data sheet 
        except:
            print("Exception...")
            self.p.stop()
            GPIO.cleanup()

def main():
    #TODO Control servo between duty cycle 6 (delivery) and 10 (hold package). 7.5 is neutral level
    # Not using angle conversion
    servos = Setup(7)
    print("setup ", len(servos), " servo motors...")
    servos[0].p.start(0)
    while True:
	val = float(input("Val: "))
        servos[0].p.ChangeDutyCycle(val)

if __name__ == "__main__":
    main()

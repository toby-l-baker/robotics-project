import RPi.GPIO as GPIO
import time




def Setup(*pins):
    servos = []
    GPIO.setmode(GPIO.BCM)
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
        self.p.start(startAngle*10/18)
    def update_angle(self, angle):
        try:
            # self.p.ChangeDutyCycle(angle*10/18)
            self.p.ChangeDutyCycle(10)
            time.sleep(angle*0.2/60*1.1) # Turning speed from data sheet 
        except:
            print("Exception...")
            self.p.stop()
            GPIO.cleanup()

def main():
    servos = Setup(17)
    print("setup ", len(servos), " servo motors...")
    servos[0].update_angle(5)
    servos[0].update_angle(9)



if __name__ == "__main__":
    main()
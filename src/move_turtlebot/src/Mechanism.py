import RPi.GPIO as GPIO

class Mechanism:
    """Controls the mechanism to deliver, level and catch packages"""

    def __init__(self, pinNum):
        self.dc_deliver = 6
        self.dc_level = 7.5
        self.dc_catch = 9

        # Initialize GPIO settings
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pinNum, GPIO.OUT)
        self.p = GPIO.PWM(pinNum, 50)

        self.p.start(self.dc_level)

    def deliver(self):
        try:
            print("Mechanism.deliver()")
            self.p.ChangeDutyCycle(self.dc_deliver)
        except Exception as e:
            print("Got exception {}".format(e))
            self.p.stop()
            GPIO.cleanup()

    def level(self):
        try:
            print("Mechanism.level()")
            self.p.ChangeDutyCycle(self.dc_level)
        except Exception as e:
            print("Got exception {}".format(e))
            self.p.stop()
            GPIO.cleanup()

    def catch(self):
        try:
            print("Mechanism.catch()")
            self.p.ChangeDutyCycle(self.dc_catch)
        except Exception as e:
            print("Got exception {}".format(e))
            self.p.stop()
            GPIO.cleanup()

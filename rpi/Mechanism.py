import RPi.GPIO as GPIO

class Mechanism:
    """Controls the mechanism to deliver, level and catch packages"""

    def __init__(self, pinNum):
        self.target = 0 # TODO

        self.dc_deliver = 6
        self.dc_level = 7.5
        self.dc_catch = 9

        # Initialize GPIO settings
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pinNum, GPIO.OUT)
        self.p = GPIO.PWM(pinNum, 50)

        self.p.start(self.dc_level)

    def step(self):
    	pass

    def deliver(self):
        try:
            self.p.ChangeDutyCycle(self.dc_deliver)
        except Exception as e:
            print("Got exception {}".format(e))
            self.p.stop()
            GPIO.cleanup()

    def level(self):
        try:
            self.p.ChangeDutyCycle(self.dc_level)
        except Exception as e:
            print("Got exception {}".format(e))
            self.p.stop()
            GPIO.cleanup()

    def catch(self):
        try:
            self.p.ChangeDutyCycle(self.dc_catch)
        except Exception as e:
            print("Got exception {}".format(e))
            self.p.stop()
            GPIO.cleanup()

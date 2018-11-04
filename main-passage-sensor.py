#Libraries
import RPi.GPIO as GPIO
import time
import requests
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24

DEFAULT_DISTANCE = 200
DEFAULT_TOLERANCE_RANGE = 50

DETECT_TOLERANCE_RANGE = 30
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

distance = None
oldDistance = None
 
try:
    while True:
        oldDistance = distance

        # set Trigger to HIGH
        GPIO.output(GPIO_TRIGGER, True)
     
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)
     
        StartTime = time.time()
        StopTime = time.time()
     
        # save StartTime
        while GPIO.input(GPIO_ECHO) == 0:
            StartTime = time.time()
     
        # save time of arrival
        while GPIO.input(GPIO_ECHO) == 1:
            StopTime = time.time()
     
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2

        print ("Measured Distance = %.1f cm" % dist)

        if (oldDistance != None && ((DEFAULT_DISTANCE - distance) > DEFAULT_TOLERANCE_RANGE) && (abs(distance - oldDistance) > DETECT_TOLERANCE_RANGE)):
            print ("Someone passed")
            r = requests.post("http://ec2-52-14-74-16.us-east-2.compute.amazonaws.com:3000/ws/sensors", json={"type": "passage", "info": "medicine"})
            print(r.status_code, r.reason)
            print(r.text)

        time.sleep(1)

except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
import RPi.GPIO as GPIO
import requests

GPIO.setmode(GPIO.BOARD)

DOOR_SENSOR_PIN = 10

oldIsOpen = None 
isOpen = None
    
# Set up the door sensor pin.
GPIO.setup(DOOR_SENSOR_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP) 

try:
	while True: 
		oldIsOpen = isOpen 
		isOpen = GPIO.input(DOOR_SENSOR_PIN)  
    
		if (isOpen and (isOpen != oldIsOpen)):  
			print("Door was opened!")
			r = requests.post("http://ec2-52-14-74-16.us-east-2.compute.amazonaws.com:3000/ws/sensors", json={"type": "door", "info": "medicine"})
			print(r.status_code, r.reason)
			print(r.text)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
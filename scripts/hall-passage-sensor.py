import RPi.GPIO as GPIO
import requests

PIR_SENSOR_PIN = 8

GPIO.setmode(GPIO.BOARD)

GPIO.setup(PIR_SENSOR_PIN, GPIO.IN)

oldPresence = None 
presence = None

try:
	while True:
		oldPresence = presence 
		presence = GPIO.input(PIR_SENSOR_PIN)
		
		if (presence and presence != oldPresence): 		
			print("Motion Detected")
			r = requests.post("http://ec2-52-14-74-16.us-east-2.compute.amazonaws.com:3000/ws/sensors", json={"type": "passage", "info": "hall"})
			print(r.status_code, r.reason)
			print(r.text)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
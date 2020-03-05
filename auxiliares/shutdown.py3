import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library
from time import sleep
from os import system

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
while True: # Run forever
	if GPIO.input(18) == GPIO.LOW:
		sleep(1)
		if GPIO.input(18) == GPIO.LOW:
			system("shutdown -h now")
	sleep(1)

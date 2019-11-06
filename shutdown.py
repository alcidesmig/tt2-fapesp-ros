# Credits: https://core-electronics.com.au/tutorials/how-to-make-a-safe-shutdown-button-for-raspberry-pi.html

from gpiozero import button #import button from the Pi GPIO library
import time # import time functions
import os #imports OS library for Shutdown control

stopButton = Button(18) # defines the button as an object and chooses GPIO 26

while True: #infinite loop
     if stopButton.is_pressed: #Check to see if button is pressed
        time.sleep(1) # wait for the hold time we want. 
        if stopButton.is_pressed: #check if the user let go of the button
            os.system("sync")
            os.system("shutdown now -h") #shut down the Pi -h is or -r will reset
    time.sleep(1) # wait to loop again so we don’t use the processor too much.

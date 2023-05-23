# A  basic 'free roaming' robot
# uses the front and rear ultrasonic sensors to avoid obstacles
# if an ultrasonic is not fitted it will return -1
#uses the onboard button for a start / stop command via an IRQ.

from PicoAutonomousRobotics import KitronikPicoRobotBuggy
from time import sleep

buggy = KitronikPicoRobotBuggy()
startStop = False

def ButtonHandler():
    global startStop
    if startStop == True:
        startStop = False
    else:
        startStop = True

buggy.setLED(0, buggy.PURPLE)
buggy.setLED(1, buggy.PURPLE)
buggy.setLED(2, buggy.PURPLE)
buggy.setLED(3, buggy.PURPLE)
buggy.show()

while True:
    if startStop == True:
        frontDistance = buggy.getDistance("f")
        rearDistance = buggy.getDistance("r")
        
        if frontDistance > 15:
            # All clear, speed ahead
            buggy.setLED(0, buggy.GREEN)
            buggy.setLED(1, buggy.GREEN)
            buggy.setLED(2, buggy.GREEN)
            buggy.setLED(3, buggy.GREEN)
            buggy.motorOn("l", "f", 85)
            buggy.motorOn("r", "f", 85)
            
        elif frontDistance > 5:
            # Something in the way, but we (probably) have time to miss it
            buggy.setLED(0, buggy.GREEN)
            buggy.setLED(1, buggy.GREEN)
            buggy.setLED(2, buggy.GREEN)
            buggy.setLED(3, buggy.GREEN)
            buggy.motorOff("l")
            buggy.motorOff("r")

            if rearDistance > 15:
                # There is space for us to reverse in a curve and try again
                buggy.setLED(0, buggy.BLUE)
                buggy.setLED(1, buggy.BLUE)
                buggy.setLED(2, buggy.BLUE)
                buggy.setLED(3, buggy.BLUE)
                buggy.motorOn("l", "r", 100)
                buggy.motorOn("r", "r", 50)
                sleep(0.01)
            else:
                # Spin on the spot CW
                buggy.setLED(0, buggy.YELLOW)
                buggy.setLED(1, buggy.YELLOW)
                buggy.setLED(2, buggy.YELLOW)
                buggy.setLED(3, buggy.YELLOW)
                buggy.motorOn("l", "f", 100)
                buggy.motorOn("r", "r", 100)

        else: # Spin on the spot CCW
            buggy.setLED(0, buggy.RED)
            buggy.setLED(1, buggy.RED)
            buggy.setLED(2, buggy.RED)
            buggy.setLED(3, buggy.RED)
            buggy.motorOn("l", "r", 100)
            buggy.motorOn("r", "f", 100)

        buggy.show()

    else: # Motors turned off
        buggy.motorOff("l")
        buggy.motorOff("r")
        buggy.setLED(0, buggy.PURPLE)
        buggy.setLED(1, buggy.PURPLE)
        buggy.setLED(2, buggy.PURPLE)
        buggy.setLED(3, buggy.PURPLE)
        buggy.show()
    
    # No interrupts, so we'll have to poll
    if buggy.button.value:
        ButtonHandler()
        sleep(0.1) # So the button isn't repeatedly triggered

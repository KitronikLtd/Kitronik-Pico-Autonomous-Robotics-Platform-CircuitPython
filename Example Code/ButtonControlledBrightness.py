from PicoAutonomousRobotics import KitronikPicoRobotBuggy
from time import sleep, monotonic

buggy = KitronikPicoRobotBuggy()

brightness = 10
countUp = True

def ButtonHandler():
    global countUp
    global brightness
    if countUp:
        brightness += 5
        if brightness > 99:
            countUp = False
    else:
        brightness -= 5
        if brightness < 1:
            countUp = True
    
    buggy.setBrightness(brightness)
    buggy.show()

def rotateColours():
    # Temporarily  store the first one, then overwrite it, shifting by
    first = buggy.getLED(0)
    buggy.setLED(0, buggy.getLED(1))
    buggy.setLED(1, buggy.getLED(2))
    buggy.setLED(2, buggy.getLED(3))
    buggy.setLED(3, first) # Push the colour that was at 0 back in at this end.

# Set the LEDs initial pattern
buggy.setLED(0, buggy.RED)
buggy.setLED(1, buggy.GREEN)
buggy.setLED(2, buggy.BLUE)
buggy.setLED(3, buggy.PURPLE)
buggy.setBrightness(brightness)
buggy.show()

lastRotate = 0

while True:
    if monotonic() > lastRotate + 0.5: # Every 500 ms
        rotateColours()
        buggy.show()
        lastRotate = monotonic()
        # Don't sleep so we can continue to poll the button

    # No interrupts, so we'll have to poll
    if buggy.button.value:
        ButtonHandler()
        sleep(0.1) # So the button isn't repeatedly triggered

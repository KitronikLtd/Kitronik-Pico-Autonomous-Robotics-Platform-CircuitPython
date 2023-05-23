from PicoAutonomousRobotics import KitronikPicoRobotBuggy

buggy = KitronikPicoRobotBuggy()

while True:
    if buggy.button.value:
        buggy.beepHorn()

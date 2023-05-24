# Kitronik-Pico-Autonomous-Robotics-Platform-CircuitPython
A class and sample code for the Kitronik Autonomous Robotics Platform for the Raspberry Pi Pico. (www.kitronik.co.uk/5335)

On the Autonomous Robotics Platform:  
* Forward is defined as the end away from the power switch / castor  
* Left and Right are defined when facing Forward  
* The Pico should be inserted with the USB connector facing the rear (away from the Pen hole)  

To use, save PicoAutonomousRobotics.py file onto the Pico so it can be imported.

You'll also need to copy the `adafruit_pioasm.mpy` and `adafruit_pixelbuf.mpy` files into the `lib` folder on your Pico. You can get these files from the [Adafruit CircuitPython Bundle](https://circuitpython.org/libraries).

## Import PicoAutonomousRobotics.py and construct an instance:
``` python
from PicoAutonomousRobotics import KitronikPicoRobotBuggy
robot = KitronikPicoRobotBuggy()
```
This will setup the various correct pins for motors / servos / sensors.

## Motors
### Drive a motor:
``` python
robot.motorOn(motor, direction, speed)
```
where:
* motor => "l" or "r" (left or right)
* direction => "f" or "r" (forward or reverse)
* speed => 0 to 100

### Stop a motor:
``` python
    robot.motorOff(motor)
```
where:
* motor => "l" or "r" (left or right)

## Servos
The servo PWM (20ms repeat, on period capped between 500 and 2500us) is driven using the Pico PIO.  
The servos are registered automatically in the initalisation of the class.   
This process sets the PIO PWM active on the servo pin.

### Drive a servo:
``` python
robot.goToPosition(servo, degrees)
```
where:
* servo => the servo to control (0-3)
* degrees => 0-180

``` python
robot.goToPeriod(servo, period)
``` 
where:
* servo => the servo to control (0-3)
* period => 500-2500 in us

## Ultrasonic Sensor
### Read ultrasonic distance:
``` python
robot.getDistance(whichSensor)
```
where:
* whichSensor => "f" or "r" for front or rear sensor

This parameter is defaulted to "f" so the call can be:  
``` python
robot.getDistance()
```

### Setup the units:  
``` python
robot.setMeasurementsTo(units)
```
where:
* units => "inch" for imperial (inch) measurements, "cm" for metric (cm) measuerments

## Line Following
Sensors are marked on the sensor PCB for left, right, and centre. Left is defined as on the left side when looking down on the buggy, facing the front.  
The centre sensor is slightly ahead of the side sensors.

### Read values:
``` python
robot.getRawLFValue(whichSensor):
```
where:
* whichSensor => "c", "l", "r" (centre, left or right sensor)

returns:
* the raw sensor value in the range 0-65535 (low numbers represent dark surfaces)  

The line following sensor can also return **True** or **False**:
``` python
robot.isLFSensorLight(whichSensor):
```
where:
* whichSensor => "l","r", or "c"

returns:  
* **True** when sensor is over a light surface and **False** when over a dark surface


### Set light / dark thresholds:
The light / dark determination is based on the values in "darkThreshold" and "lightThreshold".  
To set the thresholds use:
``` python
robot.setLFDarkValue(darkThreshold, OptionalLeftThreshold = -1, OptionalRightThreshold = -1)
```
``` python
robot.setLFLightValue(lightThreshold, OptionalLeftThreshold = -1, OptionalRightThreshold = -1)
```
Typical values for 'Light' surfaces would be under 20000, and for 'Dark' surfaces over 30000.  
Optional thresholds can set different values for Left and Right sensors in which case the first value sets the Centre threshold.  
If not used all 3 sensors have the same threshold.


## Buzzer
The buzzer is driven with a PWM pin.  
### Sound the buzzer:  
``` python
robot.soundFrequency(frequency)
```
where:
* frequency => 0-3000 (the frequency to sound in Hz)

### Silence the buzzer:
``` python
robot.silence()
```

### Beep the buzzer (like a car horn):
``` python        
robot.beepHorn():
```

## ZIP LEDs
ZIP LEDs have a 2 stage operation...
### Setup ZIP LEDs:  
Set the LEDs with the colour required:  
``` python
robot.setLED(whichLED, whichColour)
```
where:  
* whichLED => 0-3  
* whichColour => tuple of (Red Value, Green Value, Blue Value), or one of the pre-defined colours:

``` python
COLOURS = (BLACK, RED, YELLOW, GREEN, CYAN, BLUE, PURPLE, WHITE)
```

Turn off the LEDs:
``` python
robot.clear(whichLED)
```
where:  
* whichLED => 0-3

Control the brightness:
``` python
robot.setBrightness(value)
```
where:  
* value => 0-100 (brightness value in %)

### Make the changes visible:
``` python
robot.show():
```

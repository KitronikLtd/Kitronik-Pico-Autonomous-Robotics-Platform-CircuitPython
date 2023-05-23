from time import sleep, monotonic
from array import array
from board import GP0, GP3, GP2, GP6, GP7, GP10, GP11, GP14, GP15, GP16, GP17, GP18, GP19, GP20, GP21, GP26, GP27, GP28
from digitalio import DigitalInOut, Direction, Pull
from pwmio import PWMOut
from analogio import AnalogIn
from pulseio import PulseIn
from rp2pio import StateMachine
from adafruit_pioasm import Program
from adafruit_pixelbuf import PixelBuf
from struct import pack

pServo = Program("""
    pull            ;; don't start until first value available
    
.wrap_target
    pull noblock
    mov x, osr      ;; reload OSR with last value
    set pins, 0
    out y, 16       ;; off time
    
loop_off:
    jmp y--, loop_off
    set pins, 1
    out y, 16       ;; on time
    
loop_on:
    jmp y--, loop_on
.wrap
""")

pZip = Program("""
.program ws2812
.side_set 1

.wrap_target
    pull block          side 0
    out y, 32           side 0      ; get count of ws2812 bits

bitloop:
    pull ifempty        side 0      ; drive low
    out x 1             side 0 [5]
    jmp !x do_zero      side 1 [3]  ; drive high and branch depending on bit val
    jmp y--, bitloop    side 1 [4]  ; drive high for a one (long pulse)
    jmp end_sequence    side 0      ; sequence is over

do_zero:
    jmp y--, bitloop    side 0 [4]  ; drive low for a zero (short pulse)

end_sequence:
    pull block          side 0      ; get fresh delay value
    out y, 32           side 0      ; get delay count

wait_reset:
    jmp y--, wait_reset side 0      ; wait until delay elapses
.wrap
""")

class KitronikZIPLEDs(PixelBuf):
    def __init__(self, pin, num_leds, brightness=0.1, auto_write=True):
        byte_count = 3 * num_leds
        bit_count = byte_count * 8
        padding_count = -byte_count % 4
        # backwards, so that dma byteswap corrects it!
        header = pack(">L", bit_count - 1)
        trailer = b"\0" * padding_count + pack(">L", 3840)

        self.sm = StateMachine(pZip.assembled,
                               frequency=12_800_000,
                               first_sideset_pin=pin,
                               out_shift_right=False,
                               auto_pull=False,
                               pull_threshold=32,
                               **pZip.pio_kwargs)

        super().__init__(num_leds,
                         byteorder="GRB",
                         brightness=brightness,
                         auto_write=auto_write,
                         header=header,
                         trailer=trailer)

    def _transmit(self, buf):
        self.sm.background_write(memoryview(buf).cast("L"), swap=True)

class KitronikPicoRobotBuggy:
    # Button for user input:
    button = DigitalInOut(GP0)
    button.direction = Direction.INPUT
    button.pull = Pull.DOWN

    '''
    Motors: controls the motor directions and speed for both motors
    '''
    def _initMotors(self):
        self.motor1Forward = PWMOut(GP20, frequency=100)
        self.motor1Reverse = PWMOut(GP19, frequency=100)
        self.motor2Forward = PWMOut(GP6, frequency=100)
        self.motor2Reverse = PWMOut(GP7, frequency=100)
        # Can't use variable frequency on two pins in the same channel
        self.motorOff("l")
        self.motorOff("r")
        
    def motorOn(self, motor, direction, speed, jumpStart=False):
        # Cap speed to 0-100%
        if speed < 0:
            speed = 0
        elif speed > 100:
            speed = 100

        # Jump start motor by setting to 100% for 20 ms,
        # then dropping to speed specified.
        # Down to jump start the motor when set at low speed
        if not jumpStart and speed > 0.1 and speed < 35:
            self.motorOn(motor, direction, 100, True)
            sleep(0.02)

        # Convert 0-100 to 0-65535
        PWMVal = int(speed * 655.35)
        if motor == "l":
            if direction == "f":
                self.motor1Forward.duty_cycle = PWMVal
                self.motor1Reverse.duty_cycle = 0
            elif direction == "r":
                self.motor1Forward.duty_cycle = 0
                self.motor1Reverse.duty_cycle = PWMVal
            else:
                raise Exception("INVALID DIRECTION") # Harsh, but at least you'll know
        elif motor == "r":
            if direction == "f":
                self.motor2Forward.duty_cycle = PWMVal
                self.motor2Reverse.duty_cycle = 0
            elif direction == "r":
                self.motor2Forward.duty_cycle = 0
                self.motor2Reverse.duty_cycle = PWMVal
            else:
                raise Exception("INVALID DIRECTION") # Harsh, but at least you'll know
        else:
            raise Exception("INVALID MOTOR") # Harsh, but at least you'll know
    
    # To turn off set the speed to 0
    def motorOff(self,motor):
        self.motorOn(motor, "f", 0)
        
    '''
    ServoControl:
    Servo 0 degrees -> pulse of 0.5ms, 180 degrees 2.5ms
    pulse train freq 50hz - 20mS
    1uS is freq of 1000000
    servo pulses range from 500 to 2500usec and overall pulse train is 20000usec repeat.
    servo pins on P.A.R.P. are: Servo 1: 21, Servo2 10, Servo 3 17, Servo 4 11
    '''
    degreesToUS = 2000 / 180
     
    # goToPosition takes a degree position for the serov to goto. 
    # 0degrees->180 degrees is 0->2000us, plus offset of 500uS
    #1 degree ~ 11uS.
    #This function does the sum then calls goToPeriod to actually poke the PIO 
    def goToPosition(self, servo, degrees):
        period = int(degrees * self.degreesToUS + 500)
        self.goToPeriod(servo, period)
    
    def goToPeriod(self, servo, period):
        if servo < 0:
            servo = 0
        if servo > 3:
            servo = 3
        if period < 500:
            period = 500
        if period > 2500:
            period = 2500
        
        self.servos[servo].background_write(memoryview(array('HH', [20_000 - period, period])).cast('L'))
        
    def _initServos(self):
        servoPins = [GP21, GP10, GP17, GP11]
        self.servos = []
        for i in range(len(servoPins)) :
            self.servos.append(StateMachine(pServo.assembled, frequency=1_000_000, first_set_pin=servoPins[i], **pServo.pio_kwargs))
            
    '''
    ZIPLEDS
    We drive the ZIP LEDs using one of the PIO statemachines. 
    '''
    # Define some colour tuples for people to use.    
    BLACK = (0, 0, 0)
    RED = (255, 0, 0)
    YELLOW = (255, 150, 0)
    GREEN = (0, 255, 0)
    CYAN = (0, 255, 255)
    BLUE = (0, 0, 255)
    PURPLE = (180, 0, 255)
    WHITE = (255, 255, 255)
    COLOURS = (BLACK, RED, YELLOW, GREEN, CYAN, BLUE, PURPLE, WHITE)
    
    #sow pushes the current setup of theLEDS to the physical LEDS - it makes them visible.
    def show(self):
        self.ZIPLEDs.show()
    
    def clear(self, whichLED):
        self.setLED(whichLED, self.BLACK)
        
    # Sets the colour of an individual LED. Use show to make change visible
    def setLED(self, whichLED, whichColour):
        if whichLED < 0:
            raise Exception("INVALID LED:", whichLED, " specified")
        elif whichLED > 3:
            raise Exception("INVALID LED:", whichLED, " specified")
        else:
            self.ZIPLEDs[whichLED] = whichColour
    
    # Gets the stored colour of an individual LED, which isnt nessecerily the colour on show if it has been changed, but not 'show'n
    def getLED(self,whichLED):
        if whichLED < 0:
            raise Exception("INVALID LED:", whichLED, " specified")
        elif whichLED > 3:
            raise Exception("INVALID LED:", whichLED, " specified")
        else:
            return self.ZIPLEDs[whichLED]
    
    # Takes 0-100 as a brightness value, brighness is applies in the'show' function
    def setBrightness(self, value):
        # Cap to 0 - 100%
        if value < 0:
            value = 0
        elif value > 100:
            value = 100
        
        self.ZIPLEDs.brightness = value / 100

    '''
    Ultrasonic:
    there are 2 Ultrasonic headers. The front one is the default if not explicitly called wth 'r' for rear
    if we get a timeout (which would be a not fitted sensor, or a range over the sensors maximium the distance returned is -1
    '''
    def getDistance(self, whichSensor="f"):
        if whichSensor == "f":
            trigger = self.triggerFront
            echo = self.echoFront
        else: # Rear
            trigger = self.triggerRear
            echo = self.echoRear
        echo.clear()
        
        trigger.value = True
        sleep(0.00001)
        trigger.value = False
        
        timePassed = -1
        start = monotonic()
        echo.resume()
        
        while not echo and monotonic() < start + 0.1:
            pass
        echo.pause()
        
        if echo:
            timePassed = echo[0]
        
        if timePassed == -1: # Timeout - range equivalent of 5 meters - past the sensors limit or not fitted
            distance = -1
        else:
            distance = (timePassed * self.conversionFactor) / 2
        return distance
       
    def setMeasurementsTo(self, units):
        # 0.0343 cm per microsecond or 0.0135 inches
        if units == "inch":
            self.conversionFactor = 0.0135 # If you ask nicely we can do imperial
        else:
            self.conversionFactor = 0.0343 # cm is default - we are in  metric world.

    '''
    Linefollower: there are 3 LF sensors on the plug in board.        
    gets the raw (0-65535) value of the sensor. 65535 is full dark, 0 would be full brightness.
    in practice the values tend to vary between approx 5000 - 60000
    '''
    def getRawLFValue(self, whichSensor):
        if whichSensor == "c":
            return self.CentreLF.value
        elif whichSensor == "l":
            return self.LeftLF.value
        elif whichSensor == "r":
            return self.RightLF.value
        else:
            raise Exception("INVALID SENSOR") # Harsh, but at least you'll know
    
    '''
    These functions set the thresholds for light/dark sensing to return true / false
    there should be a gap between light and dark thresholds, to give soem deadbanding.
    if specified OptionalLeftThreshold and OptionalRightThreshold give you the ability to
    specify 3 sets of values. If missing then all sensors use the same value.
    initially all sensors are set to 30000  for light and 35000 for dark.
    '''
    def setLFDarkValue(self, darkThreshold, OptionalLeftThreshold=-1, OptionalRightThreshold=-1):
        self.centreDarkVal = darkThreshold
        if OptionalLeftThreshold == -1:
            self.leftDarkVal = darkThreshold
        else:
            self.leftDarkVal = OptionalLeftThreshold
        if OptionalRightThreshold == -1:
            self.rightDarkVal = darkThreshold
        else:
            self.rightDarkVal = OptionalRightThreshold
        
    def setLFLightValue(self,lightThreshold, OptionalLeftThreshold = -1, OptionalRightThreshold = -1):
        self.centreLightVal = lightThreshold
        if(OptionalLeftThreshold == -1):
            self.leftLightVal = lightThreshold
        else:
            self.leftLightVal = OptionalLeftThreshold
        if(OptionalRightThreshold == -1):
            self.rightLightVal = lightThreshold
        else:
            self.rightLightVal = OptionalRightThreshold
 
    '''
    this returns True when sensor is over light and FALSE over Dark.
    Light/Dark is determined by the thresholds.
    This code will throw an exception if the value returned is in the 'gery' area.
    This can happen is you sample half on /off a line for instance.
    Setting the thresholds to the same value will negate this functionality
    '''
    def isLFSensorLight(self, whichSensor):
        if whichSensor == "c":
            sensorVal = self.CentreLF.value
            if sensorVal >= self.centreDarkVal:
                return False
            elif sensorVal < self.centreLightVal:
                return True
            else:
                raise Exception("Sensor value 'Grey'")
        elif whichSensor == "l":
            sensorVal = self.LeftLF.value
            if sensorVal >= self.leftDarkVal:
                return False
            elif sensorVal < self.leftLightVal:
                return True
            else:
                raise Exception("Sensor value 'Grey'")            
        elif whichSensor == "r":
            sensorVal = self.RightLF.value
            if sensorVal >= self.rightDarkVal:
                return False
            elif sensorVal < self.rightLightVal:
                return True
            else:
                raise Exception("Sensor value 'Grey'")
        else:
            raise Exception("INVALID SENSOR") # Harsh, but at least you'll know

    '''
    Buzzer: functions will sound a horn or a required frequency. Option aswell to silence the buzzer
    '''
    def silence(self):
        self.buzzer.duty_cycle = 0 # Silence by setting duty to 0
        
    def soundFrequency(self,frequency):
        if frequency < 0:
            frequency = 0
        elif frequency > 3000:
            frequency = 3000
        
        self.buzzer.frequency = frequency # 1khz. Find out the limits of PWM on the Pico - doesn seem to make a noise past 3080hz
        self.buzzer.duty_cycle = 32767 # 50% duty
        
    def beepHorn(self):
        self.soundFrequency(350)
        sleep(0.3)
        self.silence()

    '''
    initialisation code for using:
    defaults to the standard pins and freq for the kitronik board, but could be overridden
    '''
    def __init__(self):
        self._initMotors()
        self._initServos()
        for i in range(4):
            self.goToPosition(i, 90) # Set the servo outputs to middle of the range.
        
        # Create and start the StateMachine for the ZIPLeds
        self.ZIPLEDs = KitronikZIPLEDs(GP18, 5, brightness=0.2, auto_write=False)
        
        self.triggerFront = DigitalInOut(GP14)
        self.triggerFront.direction = Direction.OUTPUT
        self.echoFront = PulseIn(GP15)
        self.echoFront.pause()
        self.echoFront.clear()
        self.triggerRear = DigitalInOut(GP3)
        self.triggerRear.direction = Direction.OUTPUT
        self.echoRear = PulseIn(GP2)
        self.echoRear.pause()
        self.echoRear.clear()
        
        # Set the measurements to metric by default.
        self.conversionFactor = 0.0343
        self.maxDistanceTimeout = int(2 * 500 / self.conversionFactor) # 500cm is past the 400cm max range by a reasonable amount for a timeout
        
        self.buzzer = PWMOut(GP16, variable_frequency=True)
        self.buzzer.duty_cycle = 0 # Ensure silence by setting duty to 0
        
        # Setup LineFollow Pins
        self.CentreLF = AnalogIn(GP27)
        self.LeftLF = AnalogIn(GP28)
        self.RightLF = AnalogIn(GP26)
        
        # The LF circuit is setup to give a high value when a dark (non reflective) surface is in view,and a low value when a light (reflective) surface is in view.
        # To aid there is a 'is it light or dark' function, and these values set the thresholds for determining that.
        self.centreLightVal = 30000 
        self.centreDarkVal = 35000
        self.leftLightVal = 30000 
        self.leftDarkVal = 35000
        self.rightLightVal = 30000 
        self.rightDarkVal = 35000

#This is stage One of a module to determine spring rate. 
#User input is requested in the console and measurements 
#are made via an adafruit VL6180x TOF sensor

#Created by David C Phillips, University of Wyoming 07APR2019

#import libraries
import time
import RPi.GPIO as GPIO
#import adafruit libraries
import board
import busio
import adafruit_vl6180x #sensor

# Create I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Create sensor instance.
sensor = adafruit_vl6180x.VL6180X(i2c)

#Class to control LEDs indicating measurements
class LED:
    def __init__(self,pin,state): #This is the constructor
        self.__gpio = pin #Stores pin number
        self.__State = state #Stores wether the pin is input or output
        self.__OnTime = self.__OffTime = 0.0 #inititalize to zero
        GPIO.setup(self.__gpio,self.__State) #setting up pin states to input or output
    
    def SetOntime(self,dur): #instance method
        self.__OnTime = self.__OffTime = dur
    
    def TurnOn(self): #instance method to turn on LED
        GPIO.output(self.__gpio,True)
        time.sleep(self.__OnTime)
        
    def TurnOff(self): #instance method to turn off LED
        GPIO.output(self.__gpio,False)
        time.sleep(self.__OffTime)

    def Flash(self, dur): #instance method to flash LED for duration
        onOffTime = dur/10.0
        timeCount = 0.0
    	
        while timeCount <= dur:
            GPIO.output(self.__gpio,True)
            time.sleep(onOffTime)
            GPIO.output(self.__gpio,False)
            time.sleep(onOffTime)
            timeCount = timeCount + 2*onOffTime

def getDistance(): #helper function to get an average distance
        dist = 0
        count = 0
        total = 0
        distArray = [0.0 , 0.0 , 0.0 , 0.0 , 0.0]
        for x in distArray:
            x = sensor.range
            total = total + x
            count += 1
            
        avg = total/count
        return avg

def getSpringConstant(dOne, dTwo, mass): #helper function to calculate the spring constant k

        Force = mass * 9.81 #F = ma, calculating the force applied to the spring
        k = Force / (dTwo - dOne) 

        return k

if __name__== '__main__':
    GPIO.setmode(GPIO.BCM) #Set GPIO pins to broadcom mode 

    LedRed = LED(12, GPIO.OUT) #Instantiate LED class for 
    LedGreen = LED(16, GPIO.OUT)
    
    LedRed.SetOntime(2.0)
    LedGreen.SetOntime(2.0)
    
    mass = 0.0
    # Main loop prints the range and lux every second:
    try:
        while (True):
        	measuring = True
        	measCount = 0
        	distanceOne = 0.0
        	distanceTwo = 0.0
        	deltaDistance = 0.0
        	dummyVar = ''
        	dummyVar = input('Press enter to begin measuring')
        	
        	while(measuring):
        		if measCount > 2:
        		    measuring = False
        		    break
        		else:
        		    LedRed.TurnOn()
        		    mass = float(input('Please enter the mass: ')) #get user input to store mass as a float
        		    LedRed.TurnOff()
        		    print(" ")
        		    print('Distance One is the distance between the sensor and the spring with no weight attached')
        		    dummyVar = input('Press enter to begin measuring distance one')
        		    LedGreen.Flash(3.0)
        		    distanceOne = getDistance() #call the getDistance Function to measure distance
        		    
        		    print("Distance one is measured to be: ", distanceOne," mm")
        		    measCount = measCount+1
        		    print(" ") #creating white space in the console
                    
        		    print('Distance Two is the distance between the sensor and the spring with a weight attached')
        		    dummyVar = input('Press enter to begin measuring distance Two')
        		    LedGreen.Flash(3.0)
        		    distanceTwo = getDistance()
        		    
        		    print("Distance two is measured to be: ", distanceTwo, " mm")
        		    measCount = measCount+1

        		    measuring = False
        		    SpringConstant = getSpringConstant(distanceOne, distanceTwo, mass)
        		    print("The calculated spring constant is ", SpringConstant, "grams/meter")
        		    print("")
        		    
        		    time.sleep(2.0)
        		    print("Enter ctrl+c to exit ")
        		    
        		    print("Repeating Experiment...")
        		    



    except KeyboardInterrupt:
        print("Resetting GPIO")
        GPIO.cleanup()    


#This is stage two of a module to determine spring rate. 
#It improves on stage one by requesting physical input via push buttons
#messages are still printed in the console to help the user.
#measurments are made via an adafruit VL6180x TOF sensor

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
    LedRed.SetOntime(2.0) #set a 2 second duration for led illumination
    LedGreen.SetOntime(2.0)
    pushButton = 23 #establish pin for button
    GPIO.setup(pushButton, GPIO.IN, pull_up_down=GPIO.PUD_UP) #set pushButton pin to input with internal pull up resistor
    
    mass = 0.0
    # Main loop prints the range and lux every second:
    try:
        while (True):
        	measuring = True #variable to store boolean variable for making measurements
        	measCount = 0 #variable to track the number of consecutive measurements made (max number is 2)
        	distanceOne = 0.0 #initialize variable to store the first(unweighted) distance as a float type
        	distanceTwo = 0.0 #initialize variable to store the second (weighted) distance as a float type
        	
        	print('!!!PLEASE CHECK YOUR WIRING BEFORE CONTINUING WITH EXPERIMENT!!! \n') #warning to user
        	input('Press enter to begin measuring') #waiting on user to press enter before continuing with the program
        	
        	while(measuring):
        		if measCount > 2: #this is to catch any glitches that caused the program to make more than two measurments in a row
        		    measuring = False
        		    break
        		else:
        		    LedRed.TurnOn() #illuminate red led while waiting on mass to be entered
        		    mass = float(input('Please enter the mass: ')) #get user input to store mass as a float
        		    LedRed.TurnOff()
        		    print(" ")
        		    print('Distance One is the distance between the sensor and the spring with no weight attached \n')
        		    print('Depress the button to measuring distance one (unweighted) \n')
        		    GPIO.wait_for_edge(23, GPIO.FALLING) #new for stage two, looking for falling edge to trigger program
        		    
        		    LedGreen.Flash(3.0) #flash the led to indicate a measurement is about to made
        		    distanceOne = getDistance() #call the getDistance Function to measure distance
        		    
        		    print("Distance one is measured to be: ", distanceOne," mm \n")
        		    measCount = measCount+1 #increment measCount variable by one
                    
        		    print('Distance two is the distance between the sensor and the spring with a weight attached \n')
        		    print('Depress the button to measuring distance two (weighted) \n')
        		    GPIO.wait_for_edge(23, GPIO.FALLING) #new for stage two, looking for falling edge to trigger program
        		    LedGreen.Flash(3.0) #flash the led to indicate a measurement is about to made
        		    distanceTwo = getDistance() #call getDistance function to take the average distance
        		    
        		    print("Distance two is measured to be: ", distanceTwo, " mm\n") #print message to user to tell them what distance was measured
        		    measCount = measCount+1 #increment measCount variable by one

        		    measuring = False
        		    SpringConstant = getSpringConstant(distanceOne, distanceTwo, mass) #call getSpringConstant to return spring constant
        		    print("The calculated spring constant is ", SpringConstant, "grams/meter \n") #print spring constant value to user
        		    
        		    
        		    print("Enter ctrl+c to exit ")
        		    time.sleep(2.0) #wait two seconds before continuing
        		    print("Repeating Experiment...")
        		    



    except KeyboardInterrupt:
        print("Resetting GPIO")
        GPIO.cleanup()    


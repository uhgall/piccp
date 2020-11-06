#!/usr/bin/python
# -*- coding:utf-8 -*-

import time
import RPi.GPIO as GPIO

import sys
import time
import numpy as np
import itertools
from pipyadc.ADS1256_definitions import *
from pipyadc import ADS1256


# Change this to the local DNS name of your Pi (often raspberrypi.local, if you have changed it) or
# make it blank to connect to localhost.
#PI_HOST = 'klabs.local'

### STEP 1: Initialise ADC object using default configuration:
# (Note1: See ADS1256_default_config.py, see ADS1256 datasheet)
# (Note2: Input buffer on means limited voltage range 0V...3V for 5V supply)
ads = ADS1256() #pi=io.pi(PI_HOST))

### STEP 2: Gain and offset self-calibration:
ads.cal_self()





# ADC = ADS1256.ADS1256()
# ADC.ADS1256_init()

GPIO.setmode(GPIO.BCM)
#GPIO.setmode(GPIO.BOARD)

iopin = 12
GPIO.setup(iopin, GPIO.OUT)
p = GPIO.PWM(iopin, 1000)  # channel=iopin frequency=50Hz


p.start(0)
dcrange = [0,1,2,4,8,16,32,64,100]

# def getvalue(channel):
#     numvalues = 10
#     values = [0]*numvalues
#     for i in range(numvalues):
#     	values[i] = ADC.ADS1256_GetChannalValue(channel)
#     return int(sum(values)/numvalues)


while(1):
    for dc in dcrange:
        p.ChangeDutyCycle(dc)
        time.sleep(1)
        #ADC_Value_0 = ADC.ADS1256_GetChannalValue(0)
        # ADC_Value_0 = getvalue(0)
        # ADC_Value_2 = getvalue(2)
        # ADC_Value_3 = getvalue(3)
        # Voltage2 = ADC_Value_2*5/0x7fffff
        # Voltage3 = ADC_Value_3*5/0x7fffff
        
        v2 = POS_AIN2|NEG_AINCOM
        v3 = POS_AIN3|NEG_AINCOM

        raw_channels = ads.read_sequence([v2,v3])
        voltages     = [i * ads.v_per_digit for i in raw_channels]

        print ("Duty = "+str(dc)+ "   V1 = "+str(voltages[0])+"    V2 = "+str(voltages[1]))
#        print ("3 ADC = %lf"%(ADC_Value_3*5.0/0x7fffff))
#        temp = (ADC_Value_0>>7)*5.0/0xffff
#        print ("DAC :",temp)

    print ("\33["+str(len(dcrange)+1)+"A") # /33 is octal for escape. [ starts sequence. 4A moves up 4 lines.


p.stop()
GPIO.cleanup()
print ("\r\nProgram end     ")
exit()

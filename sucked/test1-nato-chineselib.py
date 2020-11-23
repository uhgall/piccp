#!/usr/bin/python
# -*- coding:utf-8 -*-

import time
import ADS1256
import RPi.GPIO as GPIO

ADC = ADS1256.ADS1256()
ADC.ADS1256_init()

#GPIO.setmode(GPIO.BOARD)
iopin = 12
GPIO.setup(iopin, GPIO.OUT)
p = GPIO.PWM(iopin, 1000)  # channel=iopin frequency=50Hz
p.start(0)
dcrange = [0,1,2,4,8,16,32,64,100]

def getvalue(channel):
    numvalues = 10
    values = [0]*numvalues
    for i in range(numvalues):
    	values[i] = ADC.ADS1256_GetChannalValue(channel)
    return int(sum(values)/numvalues)


while(1):
    for dc in dcrange:
        p.ChangeDutyCycle(dc)
        time.sleep(1)
        #ADC_Value_0 = ADC.ADS1256_GetChannalValue(0)
        ADC_Value_0 = getvalue(0)
        ADC_Value_2 = getvalue(2)
        ADC_Value_3 = getvalue(3)
        Voltage2 = ADC_Value_2*5/0x7fffff
        Voltage3 = ADC_Value_3*5/0x7fffff
        print ("Duty = "+str(dc)+ "   V1 = "+str(Voltage2)+"    V2 = "+str(Voltage3))
#        print ("3 ADC = %lf"%(ADC_Value_3*5.0/0x7fffff))
        temp = (ADC_Value_0>>7)*5.0/0xffff
#        print ("DAC :",temp)

    print ("\33["+str(len(dcrange)+1)+"A") # /33 is octal for escape. [ starts sequence. 4A moves up 4 lines.


p.stop()
GPIO.cleanup()
print ("\r\nProgram end     ")
exit()

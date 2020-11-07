#!/usr/bin/python
# -*- coding:utf-8 -*-

import time
import RPi.GPIO as GPIO

import threading

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

#initialize PWM
GPIO.setmode(GPIO.BCM)


# def read_adc(which):  
#     raw_channels = ads.read_sequence([which])
#     return(ads.v_per_digit * raw_channels[0])

# def read_current():
#     r1 = 0.135 # Ohms
#     u = read_adc(POS_AIN3|NEG_AINCOM)
#     return(u / r1)


# def read_electrode_voltage():
#     vals = ads.read_sequence([POS_AIN2|NEG_AINCOM,POS_AIN3|NEG_AINCOM])
#     return((vals[0] - vals[1]) * ads.v_per_digit)


# def display_ramp():
#     iopin = 12
#     GPIO.setup(iopin, GPIO.OUT) 
#     p = GPIO.PWM(iopin, 1000)  # channel=iopin frequency=50Hz   
#     p.start(0)
#     dcrange = [0,1,2,4,8,16,32,64,100]
#     v2 = POS_AIN2|NEG_AINCOM
#     v3 = POS_AIN3|NEG_AINCOM

#     while(1):
#         for dc in dcrange:
#             p.ChangeDutyCycle(dc)
#             time.sleep(1)
#             print ("Duty = {}, V2 = {}, V3 = {}".format(dc, read_adc(v2),read_adc(v3)))

#         print ("\33["+str(len(dcrange)+1)+"A") # /33 is octal for escape. [ starts sequence. 4A moves up 4 lines.


class IPPCController(threading.Thread) :

    ms_current = None
    ms_voltage = None
    cv_voltage = 0
    cv_dc = 0

    def set_voltage(self,cv_voltage):
        self.cv_voltage = voltage

    def run(self):
        R1 = 0.135 # Ohms
        pwmpin = 12
        GPIO.setup(pwmpin, GPIO.OUT)
        pwm = GPIO.PWM(pwmpin, 1000) 
        self.cv_dc = 50 # start in the middle)
        pwm.start(self.cv_dc)

        while True:
            vals = ads.read_sequence([POS_AIN2|NEG_AINCOM,POS_AIN3|NEG_AINCOM])
            self.ms_voltage = (vals[0] - vals[1]) * ads.v_per_digit
            self.ms_current = vals[1] * ads.v_per_digit / R1

            delta = self.cv_voltage - self.ms_voltage
            dc = self.cv_dc - 20.0 * delta
            if dc > 100:
                self.cv_dc = 100
            if dc < 0:
                self.cv_dc = 0
            else:
                self.cv_dc = dc

            #print("Commanded voltage is {}. Current measured voltage is {}. Setting dc to {}.".format(self.cv_voltage, self.ms_voltage, self.cv_dc))
            pwm.ChangeDutyCycle(self.cv_dc)
            time.sleep(0.001)

    # this would be the proper way to clean up, but the above is an infinite loop... but it doesn't matter for now. 
    #p.stop()

ippc = IPPCController()
ippc.start()

while True:
    for v in [1.8,1.9,2.0,2.2,2.4,2.5]:
        ippc.cv_voltage = v
        time.sleep(10)
        print("Commanded voltage is {:.3f}V. Current measured voltage is {:.3f}V. Current is {:.3f}A. Implies a resistive load of {:.3f} Ohms. dc was {:.2f}%".format(v, ippc.ms_voltage, ippc.ms_current, ippc.ms_voltage / ippc.ms_current, ippc.cv_dc))
        print("dc was {:.2f}%".format(ippc.cv_dc))

GPIO.cleanup()
print ("\r\nProgram end     ")
exit()

#!/usr/bin/python
# -*- coding:utf-8 -*-

import time
import RPi.GPIO as GPIO

import threading

from ostruct import OpenStruct

import PID

import sys
import time
import numpy as np
import itertools
from pipyadc.ADS1256_definitions import *
from pipyadc import ADS1256

from bottle import route, run, template

import plotly.offline as po
import plotly.graph_objs as go

import atexit


# Change this to the local DNS name of your Pi (often raspberrypi.local, if you have changed it) or
# make it blank to connect to localhost.
#PI_HOST = 'klabs.local'
### STEP 1: Initialise ADC object using default configuration:
# (Note1: See ADS1256_default_config.py, see ADS1256 datasheet)
# (Note2: Input buffer on means limited voltage range 0V...3V for 5V supply)
ads = ADS1256() #pi=io.pi(PI_HOST))
ads.drate = DRATE_30000

### STEP 2: Gain and offset self-calibration:
ads.cal_self()

#initialize PWM
GPIO.setmode(GPIO.BCM)

pwmpin = 12
GPIO.setup(pwmpin, GPIO.OUT)
pwm = GPIO.PWM(pwmpin, 100) 
pwm.start(50.0)

def exit_handler():
    pwm.stop()
    GPIO.cleanup()
    print('Stopped PWM, cleaned up GPIO.')
atexit.register(exit_handler)


class IPPCController(threading.Thread) :

    ms_current = None
    ms_voltage = None
    cv_voltage = 2.2
    cv_dc = 200 # start in the middle

    def set_voltage(self,cv_voltage):
        self.cv_voltage = voltage

    def run(self):
        R1 = 0.27 # Ohms
        pwm.ChangeDutyCycle(self.cv_dc)
        repeats = 100

        while True:
            vals = [0,0,0]
            for i in range(repeats):
                vals = [x+y for x,y in zip(vals, ads.read_sequence([POS_AIN2|NEG_AINCOM,POS_AIN3|NEG_AINCOM]))]

            self.ms_voltage = (vals[0] - vals[1]) * ads.v_per_digit / repeats
            self.ms_current = (vals[1] * ads.v_per_digit / repeats) / R1 

            delta = self.cv_voltage - self.ms_voltage
            dc = self.cv_dc - 10.0 * delta
            if dc > 100:
                self.cv_dc = 100
            elif dc < 0:
                self.cv_dc = 0
            else:
                self.cv_dc = dc

            print("Commanded voltage is {}. Current measured voltage is {}. Setting dc to {}.".format(self.cv_voltage, self.ms_voltage, self.cv_dc))
            pwm.ChangeDutyCycle(self.cv_dc)
            time.sleep(0.050)

ippc = IPPCController()
ippc.start()

def read_adc(which,avg_count=100):  
    raw = 0 
    for i in range(avg_count):
        raw = raw + ads.read_sequence([which])[0]
    return(ads.v_per_digit * raw / avg_count)

def read_current():
    r1 = 0.27 # Ohms
    u = read_adc(POS_AIN3|NEG_AINCOM)
    return(u / r1)

def read_electrode_voltage():
    vals = ads.read_sequence([POS_AIN2|NEG_AINCOM,POS_AIN3|NEG_AINCOM])
    return((vals[0] - vals[1]) * ads.v_per_digit)

@route('/')
def home():
    status = "Commanded voltage is {:.3f}V. Current measured voltage is {:.3f}V. Current is {:.3f}A. Implies a resistive load of {:.3f} Ohms. dc was {:.2f}%"
    data = OpenStruct(
        status = status.format(ippc.cv_voltage, ippc.ms_voltage, ippc.ms_current, ippc.ms_voltage / ippc.ms_current, ippc.cv_dc)
        )
    return(template("""
        <h1>{{status}}</h1>
        <h1>Tools</h1>
        <ul>
        <li>
        </li>
        <li>
        <a href="/set_dc?dc=100">Set Duty Cycle to 100</a>
        </li>
        </ul>
        """,data))


run(host='localhost', port=8080, debug=True)




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

from bottle import route, run, template, request

import plotly.offline as po
import plotly.graph_objs as go

import atexit

CIRCUIT_R1 = 0.27 # Ohms; shunt resistor in series to load


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
    cv_dc = 50 # start in the middle

    def set_voltage(self,cv_voltage):
        self.cv_voltage = voltage

    def run(self):
        pwm.ChangeDutyCycle(self.cv_dc)
        repeats = 100
        self.last_time =  time.time()

        self.Kp = -50
        self.Ki = -10
        self.Kd = -5

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        self.lastInput = 0
        self.lastTime = time.time()

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        outMax = 100
        outMin = 0

        while True:
            vals = [0,0,0]
            for i in range(repeats):
                vals = [x+y for x,y in zip(vals, ads.read_sequence([POS_AIN2|NEG_AINCOM,POS_AIN3|NEG_AINCOM]))]

            self.ms_voltage = (vals[0] - vals[1]) * ads.v_per_digit / repeats
            self.ms_current = (vals[1] * ads.v_per_digit / repeats) / CIRCUIT_R1
            self.current_time = time.time()

            Input = self.ms_voltage
            SetPoint = self.cv_voltage

            error = SetPoint - Input
            self.ITerm += (self.Ki * error)
            if (self.ITerm > outMax):
                self.ITerm = outMax
            elif (self.ITerm < outMin):
                self.ITerm = outMin;
            
            dInput = (Input - self.lastInput);
 
            Output = self.Kp * error + self.ITerm - self.Kd * dInput;
            if (Output > outMax):
                Output = outMax;
            elif (Output < outMin): 
                Output = outMin;
 
            self.lastInput = Input;
            self.lastTime = time.time()

            dc = Output

            # error = self.cv_voltage - self.ms_voltage
            # delta_error = error - self.last_error

            # delta_time = self.current_time - self.last_time
            
            # self.PTerm = self.Kp * error
            # self.ITerm += error * delta_time

            # if (self.ITerm < -self.windup_guard):
            #     self.ITerm = -self.windup_guard
            # elif (self.ITerm > self.windup_guard):
            #     self.ITerm = self.windup_guard

            # self.DTerm = delta_error / delta_time

            # # Remember last time and last error for next calculation
            # self.last_time = self.current_time
            # self.last_error = error

            # dc = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

            self.cv_dc = max(min( int(dc), 100 ),0)
            print("After dt={}, cv={}. mv={} dc={} err={}".format(0,self.cv_voltage, self.ms_voltage, dc, error))

            #print("After delta_time={}, commanded voltage is {}. Current measured voltage is {}. Setting dc to {}.".format(0,self.cv_voltage, self.ms_voltage, dc))
            pwm.ChangeDutyCycle(self.cv_dc)
            time.sleep(0.050)

def read_adc(which,avg_count=100):  
    raw = 0 
    for i in range(avg_count):
        raw = raw + ads.read_sequence([which])[0]
    return(ads.v_per_digit * raw / avg_count)

def read_current():
    u = read_adc(POS_AIN3|NEG_AINCOM)
    return(u / CIRCUIT_R1)

def read_electrode_voltage():
    vals = ads.read_sequence([POS_AIN2|NEG_AINCOM,POS_AIN3|NEG_AINCOM])
    return((vals[0] - vals[1]) * ads.v_per_digit)



ippc = IPPCController()
ippc.start()


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
        <a href="/time_series?v=2.5">Set voltage to 2.5V</a>
        </li>
        </ul>
        """,data))


@route('/time_series')
def time_series():
    ippc.cv_voltage = float(request.query.v)

    sample_time = 0.100
    ms_v = []
    cv_dc = []
    x = []
    for t in range(100):
        x.append(t)
        ms_v.append(ippc.ms_voltage)
        cv_dc.append(ippc.cv_dc)
        time.sleep(sample_time)
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=x, y=ms_v,
        mode='lines',
        name="Electrode Voltage")
        )
    # fig.add_trace(go.Scatter(x=x, y=cv_dc,
    #     mode='markers',
    #     name="Duty Cycle")
    #     )
    fig.update_layout(title='Voltage as a function of time',
                   xaxis_title='time (ms)',
                   yaxis_title='Voltage (V)')

    return("<h1>Voltage as a function of time</h1>{}\n".format(fig.to_html()))



run(host='localhost', port=8080, debug=True)




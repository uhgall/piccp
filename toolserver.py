#!/usr/bin/python
# -*- coding:utf-8 -*-

import time
import RPi.GPIO as GPIO

import threading

from ostruct import OpenStruct


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


# Change this to the local DNS name of your Pi (often raspberrypi.local, if you have changed it) or
# make it blank to connect to localhost.
#PI_HOST = 'klabs.local'
### STEP 1: Initialise ADC object using default configuration:
# (Note1: See ADS1256_default_config.py, see ADS1256 datasheet)
# (Note2: Input buffer on means limited voltage range 0V...3V for 5V supply)
ads = ADS1256() #pi=io.pi(PI_HOST))

### STEP 2: Gain and offset self-calibration:
ads.drate = DRATE_30000
#ads.v_ref= 3.3
ads.cal_self()

#ads.pga_gain = 2

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

def read_adc(which,avg_count=1):  
    raw = 0 
    for i in range(avg_count):
        raw = raw + ads.read_sequence([which])[0]
    return(ads.v_per_digit * raw / avg_count)

def read_electrode_voltage():
    vals = ads.read_sequence([POS_AIN2|NEG_AINCOM,POS_AIN3|NEG_AINCOM])
    return((vals[0] - vals[1]) * ads.v_per_digit)

@route('/set_dc')
def set_dc():
    dc = float(request.query.dc)
    pwm.ChangeDutyCycle(dc)

    u2 = read_adc(POS_AIN2|NEG_AINCOM,100)
    u3 = read_adc(POS_AIN3|NEG_AINCOM,100)
    r1 = 0.27 # ohms
    i1 = u3 / r1
    return("Duty cycle set to {:.2f}. U2 = {:.3f}V. U3 = {:.3f}V. Current = {:.3f}A.".format(dc,u2, u3, i1))

v_dc = OpenStruct(
    traces=1,
    delay_ms=10,
    repeat=100)

v_dc.time_estimate = (v_dc.traces-1) * 100 * (v_dc.delay_ms * 0.001 ) + (100 * v_dc.repeat * 2 * 0.00033)
v_dc.info = template("""
    Voltage for a given duty cycle, {{traces}} traces with {{delay_ms}}ms delay between measurements, 
    and taking the average of {{repeat}} samples (estimated measurement time {{time_estimate}} seconds.)
    """,**v_dc)


@route('/')
def home():
    u2 = read_adc(POS_AIN2|NEG_AINCOM,100)
    u3 = read_adc(POS_AIN3|NEG_AINCOM,100)
    r1 = 0.27 # ohms
    i1 = u3 / r1
    v_dc.s = ("U2 = {:.3f}V. U3 = {:.3f}V. Current = {:.3f}A. v_ref={:.3f}V".format(u2, u3, i1,ads.v_ref))

    return(template("""
        <h1>{{s}}</h1>
        <h1>Tools</h1>
        <ul>
        <li>
        <a href="/v_for_given_dc">{{info}}</a>
        </li>
        <li>
        <a href="/set_dc?dc=100">Set Duty Cycle to 100</a>
        </li>
        </ul>
        """,**v_dc))


@route('/v_for_given_dc')
def v_for_given_dc():
    start_time = time.time()
    x = list(range(0,101))
    y = [[] for i in range(v_dc.traces)]
    for dc in x:
        pwm.ChangeDutyCycle(dc)
        print("Duty Cycle set to {:.2f}%".format(dc))
        for i in range(v_dc.traces):    
            if i > 0:
                time.sleep(v_dc.delay_ms*0.001)
            val = read_adc(POS_AIN2|NEG_AINCOM,v_dc.repeat)
            y[i].append(val)
    measurement_time = time.time() - start_time
    fig = go.Figure()
    for i in range(0,v_dc.traces):
        fig.add_trace(go.Scatter(x=x, y=y[i],
            mode='markers',
            marker=dict(size=i*3),
            name="After {}ms".format(i*v_dc.delay_ms)
            ))

    fig.update_layout(title='Voltage as a function of duty cycle',
                   xaxis_title='Duty Cycle (%)',
                   yaxis_title='Voltage (V)')

    return("<h1>Voltage as a function of duty cycle</h1><p>{}.<br /> Actual time {:.3f}.</p>{}\n".format(v_dc.info,measurement_time,fig.to_html()))



run(host='localhost', port=8080, debug=True)




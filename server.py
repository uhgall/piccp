#!/usr/bin/python
# -*- coding:utf-8 -*-

import time
import RPi.GPIO as GPIO

import threading

from ostruct import OpenStruct

# import PID # surprisingly, this didn't work so well.
import sys
import time
import numpy as np
import itertools

import ADS1115
import time

from bottle import route, run, template, request

import plotly.offline as po
import plotly.graph_objs as go

import atexit

from sklearn.preprocessing import PolynomialFeatures 
from sklearn.linear_model import LinearRegression 
from sklearn.pipeline import make_pipeline
from sklearn.linear_model import Ridge
from sklearn import linear_model

class IccpDriverADS1115:

    def __init__(self, 
        circuit_r1 = 0.27,  # Ohms; shunt resistor in series to load
        channel_i = 2,
        channel_v = 3,
        avg_count = 50,
        pwm_pin  = 12,
        ads = None):

        self.circuit_r1 = circuit_r1
        self.channel_i = channel_i
        self.channel_v = channel_v
        self.avg_count = avg_count
        self.pwm_pin = pwm_pin
        self.ads = ads

        #initialize PWM
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, 1200) 
        self.pwm.start(50.0)

    def read_adc(self,channels):  
        raw = [0 for ch in channels]
        for i in range(self.avg_count):
            for r in range(len(channels)):
                ch = channels[r]
                raw[r] = raw[r] + self.ads.readADCSingleEnded(ch)
        return [x / self.avg_count / 1000.0 for x in raw]

    def read_current(self):
        u = read_adc([self.channel_i])
        return(u / self.circuit_r1)

    def read_electrode_voltage(self):
        v0, v1 = self.read_adc([self.channel_i, self.channel_v])
        return(v1-v0)

    def read_current_and_electrode_voltage(self):
        v0, v1 =  self.read_adc([self.channel_i, self.channel_v])
        return [v0 / self.circuit_r1, v1-v0]

    def set_dc(self,cv_dc):
        self.cv_dc = cv_dc
        self.pwm.ChangeDutyCycle(cv_dc)

    def shutdown(self):
        self.pwm.stop()

class ICCPThread(threading.Thread):

    ms_current = None
    ms_voltage = None
    cv_voltage = 2.2
    cv_dc = 50 # start in the middle

    def __init__(self, driver = None):
        self.driver = driver
        self.sample_time = 0.035
        self.cal_start_time = None
        super().__init__()
        self.set_status("Just started.")
        self.set_fixed_voltage(self.cv_voltage)

    def set_fixed_voltage(self,v):
        self.cv_voltage = v
        self.set_status("Requesting fixed Voltage = {:.3f}.".format(v))
        self.mode = "fixed_voltage"

    def set_fixed_dc(self,dc):
        self.cv_dc = dc
        self.driver.set_dc(self.cv_dc)
        self.set_status("Setting fixed DC = {:.0f}.".format(dc))
        self.mode = "fixed_dc"
    
    def set_status(self,s):
        self.status = s
        print(s)

    def calibrate(self):
        self.set_status("Calibration starting.")
        cal_start_time = time.time()
        cal_ms_v = []
        cal_cv_dc = []
        for dc in range(0,100,5):
            cal_cv_dc.append(dc)
            self.driver.set_dc(dc)
            time.sleep(self.sample_time)
            v = self.driver.read_electrode_voltage()
            cal_ms_v.append(v)
            self.set_status("Calibration routine commanded dc = {:.0f} % and measured {:.3f} V".format(dc,v))
        self.set_status("Calibration finished.")

        # not threadsafe here but unlikely to be an issue and not deadly
        self.cal_list_ms_v = cal_ms_v
        self.cal_list_cv_dc = cal_cv_dc
        self.cal_start_time = cal_start_time

    def run(self):

        print("starting thread...")

        while True:
            self.ms_current, self.ms_voltage = self.driver.read_current_and_electrode_voltage()
            if (self.mode == "calibration"):
                v = self.cv_voltage
                dc = self.cv_dc
                self.calibrate()
                self.cv_dc = dc
                self.set_fixed_voltage(v)
                self.set_status("Restarting control loop to keep voltage at {:.3f}V.".format(self.cv_dc))
            elif (self.mode == "fixed_dc"):
                s = "Commanded duty cycle is {:.0f}%. Measured voltage is {:.3f} V. Current is {:.3f} A."
                self.set_status(s.format(self.cv_dc,self.ms_voltage, self.ms_current))
            elif (self.mode == "fixed_voltage"):
                self.driver.set_dc(self.cv_dc)
                delta = self.cv_voltage - self.ms_voltage
                dc = self.cv_dc - 20.0 * delta
                self.cv_dc = max(min(dc,100),0)
                self.driver.set_dc(self.cv_dc)
                s = "Commanded voltage is {:.3f} V. Measured voltage is {:.3f} V. Setting dc to {:.0f} %. Current is {:.3f} A."
                self.set_status(s.format(self.cv_voltage, self.ms_voltage, self.cv_dc,self.ms_current))
            time.sleep(self.sample_time) # TODO - look at last one to determine sleep

driver = IccpDriverADS1115(ads = ADS1115.ADS1115())
iccp = ICCPThread(driver)
iccp.start()

BACK = "<p><a href=\"/\">Back to main page.</a></p>"

@route('/')
def home():
    data = OpenStruct(
        status = iccp.status
        )

    return(template("""
        <h1>Status</h1>
        <li>{{status}}</li>
        <h1>Tools</h1>
        <ul>
        <li><a href="/set_v?v=2.5">Set electrode voltage to 2.5V </a> (or whatever you want, by editing the URL).</li>
        <li><a href="/set_dc?dc=50">Force duty cycle to 50%</a> (or whatever you want), turn of control loop until new electrode voltage is set.</a></li>
        <li><a href="/start_calibration">Start Calibration - Measure voltage as a function of duty cycle.</a></li>
        <li><a href="/v_for_dc_regression_plot">Show Calibration Results (Regression plot).</a></li>
        </ul>
        """,data))




@route('/set_dc')
def set_dc():
    dc = float(request.query.dc)
    iccp.set_fixed_dc(dc)
    return("<h1>Set Fixed duty cycle of ({:.0f}%).</h1>\n<p>{}</p>{}".format(dc,iccp.status,BACK))

@route('/set_v')
def set_v():
    v = float(request.query.v)
    iccp.set_fixed_voltage(v)
    return("<h1>Requesting Voltage {:.3f}%.</h1>\n<p>{}</p>{}".format(v,iccp.status,BACK))
    
@route('/start_calibration')
def start_calibration():
    if iccp.mode == "calibration":
        return("<h1>Calibration already running</h1><p>{}</p>{}".format(iccp.status,BACK))
    iccp.mode = "calibration"
    return("<h1>Requested start of calibration</h1>{}".format(BACK))

@route('/v_for_dc_regression_plot')
def v_for_dc_regression_plot():
    cal_start_time = iccp.cal_start_time
    if cal_start_time == None:
        return(start_calibration())
    list_ms_v = iccp.cal_list_ms_v
    list_cv_dc = iccp.cal_list_cv_dc

    npy = np.array(list_ms_v)
    npx = np.array(list_cv_dc)
    lm=LinearRegression()
    lm.fit(npx.reshape(-1,1),npy.reshape(-1,1))
    res=lm.predict(npx.reshape(-1,1)).reshape(1,-1)

    fig = go.Figure()
    fig.add_trace(go.Scatter(x=list_cv_dc, y=list_ms_v,
        mode='lines',
        name="Electrode Voltage"))
    fig.add_trace(go.Scatter(x=list_cv_dc, y=res[0],
        mode='lines',
        name="Regression fit"))
    fig.update_layout(title='Voltage as a function of duty cycle',
                   xaxis_title='Duty Cycle (%)',
                   yaxis_title='Voltage (V)')
    return("<h1>Voltage as a function of duty cycle, measurement started{}</h1>{}\n".format(cal_start_time, fig.to_html()))

run(host='localhost', port=8080, debug=True)

# fields=['first','second','third']
# with open(r'name', 'a') as f:
#     writer = csv.writer(f)
#     writer.writerow(fields)

def exit_handler():
    iccp.shutdown()
    GPIO.cleanup()
    print('Stopped iccp, cleaned up GPIO.')
atexit.register(exit_handler)


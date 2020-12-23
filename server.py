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



class IccpModuleADS1115:

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

    def status(self):
        return("Current = {:.3f} A, Electrode Voltage = {:.3f}V.".format(*self.read_current_and_electrode_voltage()))

    def shutdown(self):
        self.pwm.stop()


class JobThread(threading.Thread):

    stop_requested = None
    start_time = None
    finish_time = None

    def request_stop(self,s):
        self.stop_requested = s

    def is_still_running(self):
        return self.start_time != None and self.finish_time == None 

    def run(self):
        self.start_time = time.time()
        self.run_job()
        self.finish_time = time.time()

    def uptime(self):
        if self.start_time == None:
            raise(Exception("Not running {}".format(self)))
        else:
            return(time.time()-self.start_time)


class VForDCThread(JobThread) :

    x = None
    ms_v = None
    sample_time = None
    current_x = 0

    def __init__(self, 
        iccp = None,
        sample_time = 0.001):
        self.iccp = iccp
        self.sample_time = sample_time
        super().__init__()

    def run_job(self):
        self.ms_v = []
        self.x = []
        for cv_dc in range(0,100,5):
            print(cv_dc)
            self.current_x = cv_dc
            self.x.append(cv_dc)
            iccp.set_dc(cv_dc)
            time.sleep(self.sample_time)
            self.ms_v.append(iccp.read_electrode_voltage())

# class ICCPThread(JobThread) :

#     ms_current = None
#     ms_voltage = None
#     cv_voltage = 2.2
#     cv_dc = 50 # start in the middle
#     running = False

#     def __init__(self, 
#         iccp = None,
#         sample_time = 0.001, 

#         self.iccp = iccp
#         self.sample_time = sample_time

#     def set_voltage(self,cv_voltage):
#         self.cv_voltage = voltage

#     def run(self):

#         print("starting to track...")

#         self.running = True
#         iccp.set_dc(self.cv_dc)

#         while True:
#             self.ms_voltage, self.ms_current = iccp.read_current_and_electrode_voltage()
#             delta = self.cv_voltage - self.ms_voltage
#             dc = self.cv_dc - 20.0 * delta
#             self.cv_dc = max(min(dc,100),0)
#             print("Commanded voltage is {}. Current measured voltage is {}. Setting dc to {}.".format(self.cv_voltage, self.ms_voltage, self.cv_dc))
#             iccp.set_dc(self.cv_dc)
#             time.sleep(0.010)


iccp = IccpModuleADS1115(ads=ADS1115.ADS1115())

v_for_dc = VForDCThread()
v_for_dc.start()

@route('/')
def home():
    # status = "Commanded voltage is {:.3f}V. Current measured voltage is {:.3f}V. Current is {:.3f}A. Implies a resistive load of {:.3f} Ohms. dc was {:.2f}%"
    data = OpenStruct(
        status = iccp.status() #status.format(iccp.cv_voltage, iccp.ms_voltage, iccp.ms_current, iccp.ms_voltage / iccp.ms_current, iccp.cv_dc)
        )
    return(template("""
        <h1>{{status}}</h1>
        <h1>Tools</h1>
        <ul>
        <li>
        <a href="/v_for_dc_regression_plot">Calibration tool - Voltage as a function of duty cycle, regression plot.</a>
        </li>
        <li><a href="/set_electrode_voltage?v=2.5">Set electrode voltage to 2.5V</a></li>
        <li><a href="/set_dc?dc=50">Set duty cycle to 50%</a></li>
        </ul>
        """,data))


# @route('/time_series')
# def time_series():
#     iccp.cv_voltage = float(request.query.v)

#     sample_time = 0.100
#     ms_v = []
#     cv_dc = []
#     x = []
#     for t in range(100):
#         x.append(t)
#         ms_v.append(iccp.ms_voltage)
#         cv_dc.append(iccp.cv_dc)
#         time.sleep(sample_time)
#     fig = go.Figure()
#     fig.add_trace(go.Scatter(x=x, y=ms_v,
#         mode='lines',
#         name="Electrode Voltage")
#         )

#     fig.update_layout(title='Voltage as a function of time',
#                    xaxis_title='time (ms)',
#                    yaxis_title='Voltage (V)')

#     return("<h1>Voltage as a function of time</h1>{}\n".format(fig.to_html()))


# @route('/set_electrode_voltage')
# def set_electrode_voltage():
#     iccp.cv_voltage = float(request.query.v)
#     iccp.set    
#     sample_time = 0.100
#     ms_v = []
#     cv_dc = []
#     x = []
#     for t in range(100):
#         x.append(t)
#         ms_v.append(iccp.ms_voltage)
#         cv_dc.append(iccp.cv_dc)
#         time.sleep(sample_time)
#     fig = go.Figure()
#     fig.add_trace(go.Scatter(x=x, y=ms_v,
#         mode='lines',
#         name="Electrode Voltage")
#         )

#     fig.update_layout(title='Voltage as a function of time',
#                    xaxis_title='time (ms)',
#                    yaxis_title='Voltage (V)')

#     return("<h1>Voltage as a function of time</h1>{}\n".format(fig.to_html()))


@route('/set_dc')
def set_dc():
    dc = float(request.query.dc)
    iccp.set_dc(dc)
    return("<h1>Duty Cycle set to {:.2f}%.</h1>\n<p>{}</p><p><a href=\"/\">Back to main page.</a></p>".format(dc,iccp.status()))


@route('/v_for_dc_regression_plot')
def v_for_dc_regression_plot():
    global v_for_dc
    if (v_for_dc.start_time == None):
        v_for_dc.start()
        return "Started Calibration. Please reload to see results."

    if (v_for_dc.is_still_running()):
        return("<h1>Already running</h1><p>Not finished yet, have {} points after {:.3f}s. </p>".format(v_for_dc.current_x,time.time() - v_for_dc.start_time))

    npy = np.array(v_for_dc.ms_v)
    npx = np.array(v_for_dc.x)
    lm=LinearRegression()
    lm.fit(npx.reshape(-1,1),npy.reshape(-1,1))
    res=lm.predict(npx.reshape(-1,1)).reshape(1,-1)

    print(res)

    fig = go.Figure()
    fig.add_trace(go.Scatter(x=v_for_dc.x, y=v_for_dc.ms_v,
        mode='lines',
        name="Electrode Voltage"))
    fig.add_trace(go.Scatter(x=v_for_dc.x, y=res[0],
        mode='lines',
        name="Regression fit"))
    fig.update_layout(title='Voltage as a function of duty cycle',
                   xaxis_title='Duty Cycle (%)',
                   yaxis_title='Voltage (V)')
    v_for_dc = VForDCThread()
    return("<h1>Voltage as a function of duty cycle</h1>{}\n".format(fig.to_html()))


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


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




CIRCUIT_R1 = 0.27 # Ohms; shunt resistor in series to load


# Change this to the local DNS name of your Pi (often raspberrypi.local, if you have changed it) or
# make it blank to connect to localhost.
#PI_HOST = 'klabs.local'

ads = ADS1115.ADS1115()    

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


class VForDC(threading.Thread) :

    x = None
    ms_v = None
    sample_time = None
    started = None
    finished = None
    current_x = 0

    def run(self):
        if self.started == None:

            self.started = time.time()

            self.sample_time = 0.001
            trace_count = 1
            self.ms_v = [[] for i in range(trace_count)]
            self.x = []
            for cv_dc in range(0,100,1):
                print(cv_dc)
                self.current_x = cv_dc
                self.x.append(cv_dc)
                pwm.ChangeDutyCycle(cv_dc)
                for t in range(trace_count):
                    time.sleep(self.sample_time)
                    self.ms_v[t].append(read_electrode_voltage())
            self.finished = time.time()
        else:
            print("Won't start again, it's already running for #{}".format(self.started-time.time()))



class ICCPController(threading.Thread) :

    ms_current = None
    ms_voltage = None
    cv_voltage = 2.2
    cv_dc = 50 # start in the middle
    running = False

    def set_voltage(self,cv_voltage):
        self.cv_voltage = voltage

    def run_delta(self):

        self.running = True
        pwm.ChangeDutyCycle(self.cv_dc)

        while True:




            self.ms_voltage = read_electrode_voltage()
            self.ms_current = read_current()
            
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


    def run_pid(self):


        #self.calc_v_for_dc()

        self.running = True
        pwm.ChangeDutyCycle(self.cv_dc)
        repeats = 10
        self.last_time =  time.time()

        self.Kp = -20
        self.Ki = -2
        self.Kd = -1

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
                time.sleep(0.01)
                for ch in range(3):
                    vals[ch] += ads.readADCSingleEnded(channel=ch)
                    
            self.ms_voltage = (vals[1] - vals[0]) / repeats / 1000.0 # in volts, not mv
            self.ms_current = (vals[0] / repeats) / CIRCUIT_R1 / 1000.0
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
            print("cv={:.4f}. mv={:.4f} dc={:.4f} err={:.4f}".format(self.cv_voltage, self.ms_voltage, dc, error))

            #print("After delta_time={}, commanded voltage is {}. Current measured voltage is {}. Setting dc to {}.".format(0,self.cv_voltage, self.ms_voltage, dc))
            pwm.ChangeDutyCycle(self.cv_dc)
            time.sleep(0.050)

    def run(self):
        self.run_delta()


def read_adc(which,avg_count=25):  
    raw = 0 
    for i in range(avg_count):
        raw = raw + ads.readADCSingleEnded(which)
    return(raw / avg_count / 1000.0)

def read_current():
    u = read_adc(0)
    return(u / CIRCUIT_R1)

def read_electrode_voltage():
    return(read_adc(1)-read_adc(0))

iccp = ICCPController()
iccp.start()

v_for_dc = VForDC()
# v_for_dc.start()


@route('/')
def home():
    status = "Commanded voltage is {:.3f}V. Current measured voltage is {:.3f}V. Current is {:.3f}A. Implies a resistive load of {:.3f} Ohms. dc was {:.2f}%"
    data = OpenStruct(
        status = status.format(iccp.cv_voltage, iccp.ms_voltage, iccp.ms_current, iccp.ms_voltage / iccp.ms_current, iccp.cv_dc)
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
    iccp.cv_voltage = float(request.query.v)

    sample_time = 0.100
    ms_v = []
    cv_dc = []
    x = []
    for t in range(100):
        x.append(t)
        ms_v.append(iccp.ms_voltage)
        cv_dc.append(iccp.cv_dc)
        time.sleep(sample_time)
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=x, y=ms_v,
        mode='lines',
        name="Electrode Voltage")
        )

    fig.update_layout(title='Voltage as a function of time',
                   xaxis_title='time (ms)',
                   yaxis_title='Voltage (V)')

    return("<h1>Voltage as a function of time</h1>{}\n".format(fig.to_html()))





@route('/v_for_dc_regression_plot')
def v_for_dc_regression_plot():

    if (v_for_dc.started == None):
        v_for_dc.started = time.time()
        v_for_dc.start()

    if (v_for_dc.started != None and v_for_dc.finished == None ):
        return("<h1>Already running</h1><p>Not finished yet, have {} points after {}s. </p>".format(v_for_dc.current_x,time.time() - v_for_dc.started))


    npy = np.array(v_for_dc.ms_v[0])
    npx = np.array(v_for_dc.x)
 
    # poly = PolynomialFeatures(degree=2)
    # X_ = poly.fit_transform([x])
    # predict_ = poly.fit_transform([x])
    # clf = linear_model.LinearRegression()
    # clf.fit(X_, [y])
    # res = clf.predict(predict_)

    lm=LinearRegression()
    lm.fit(npx.reshape(-1,1),npy.reshape(-1,1))

    # lm.fit([x],[y])

    res=lm.predict(npx.reshape(-1,1)).reshape(1,-1)

    #print(res.tolist())

    fig = go.Figure()
    for t in range(len(v_for_dc.ms_v)):
        fig.add_trace(go.Scatter(x=v_for_dc.x, y=v_for_dc.ms_v[t],
            mode='lines',
            name="Electrode Voltage with #{:.3f}s delay".format(t*v_for_dc.sample_time)
            ))

  
    fig.add_trace(go.Scatter(x=v_for_dc.x, y=res[0],
        mode='lines',
        name="Regression fit".format(t*v_for_dc.sample_time)))

    fig.update_layout(title='Voltage as a function of duty cycle',
                   xaxis_title='Duty Cycle (%)',
                   yaxis_title='Voltage (V)')

    return("<h1>Voltage as a function of duty cycle</h1>{}\n".format(fig.to_html()))



run(host='localhost', port=8080, debug=True)


# fields=['first','second','third']
# with open(r'name', 'a') as f:
#     writer = csv.writer(f)
#     writer.writerow(fields)


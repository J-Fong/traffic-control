"""
Functions to return a control signal for RC car A to slow to a desired velocity
required to avoid collision with RC car B as calculated by a PID system.
"""
import math
import PID
import time
import numpy as np
import matplotlib.pyplot as plt

pi = math.pi

def detect_edge(A_pos, A_poscol, B_poscol, A_velcol, B_velcol, A_angcol, B_angcol, t_col):
    """
    Detects back edge of car to be avoided (car B)
    """
    # Car dimensions
    w = 12.65 # Width in cm
    l = 17.50 # Length in cm
    
    # Find direction of approach towards B and select back corner to calibrate
    # with
    B_angNpos = (B_angcol - A_angcol) % (2 * pi) # Positive B angle in A frame
    B_angNneg = (A_angcol - B_angcol) % (2 * pi) # Negative B angle in A frame
    print(B_angNpos, B_angNneg)
    x_eBN = -l / 2
    if abs(B_angNpos) < abs(B_angNneg):
        y_eBN = w / 2
    else:
        y_eBN = -w / 2
    print(x_eBN, y_eBN)
    x_eB = B_poscol[0] + x_eBN * math.cos(B_angcol) - y_eBN * math.sin(B_angcol) # Back corner x in camera frame
    y_eB = B_poscol[1] + x_eBN * math.sin(B_angcol) + y_eBN * math.cos(B_angcol) # Back corner y in camera frame
    print(x_eB, y_eB)
    """
    sampleTime = 0.01
    pidX = PID.PID(P, I, D)
    pidY = PID.PID(P, I, D)

    pidX.SetPoint = A_poscol[0]
    pidY.SetPoint = A_poscol[1]
    pidX.setSampleTime(sampleTime)
    pidY.setSampleTime(sampleTime)

    feedbackX = A_pos
    feedbackY = 

    feedback_list = []
    time_list = []
    setpoint_list = []

    for i in range(1, END):
        pidX.update(feedback)
        pidY.update(feedback)
        output = pidX.output
        output = pidY.output
        if pidX.SetPoint > 0:
            feedback += (output - (1 / i))
        if pidY.SetPoint > 0:
            feedback += (output - (1 / i))
        if i > 9:
            pidX.SetPoint = 1
        if i > 9:
            pidY.SetPoint = 1
        time.sleep(0.02)

        feedback_list.append(feedback)
        setpoint_list.append(pidX.SetPoint)
        setpoint_list.append(pidY.SetPoint)
        time_list.append(i)
    """
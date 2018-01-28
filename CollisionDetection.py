# -*- coding: utf-8 -*-
import numpy as np
"""
Created on Sat Jan 27 14:31:54 2018

@author: Renz
"""
#DECLARE FUNCTION

def GetAngle(x,y, x_prev, y_prev):
    xVector = x-x_prev
    yVector = y-y_prev
    return np.arctan2(yVector, xVector)
    
def KalmanFilter(t, t_prev, x, y, x_prev, y_prev, v, x_hat, predictedP):
    
    #Noise
    Q = np.array([[0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.1]]) #Process Noise
    R = np.array([[0.01, 0], [0, 0.01]]) #Measurement Noise
    
    #Initial guesses
    if t_prev < 0:
        x_hat = np.array([[x], [y], [0]])
        predictedP = np.array([[0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.1]])
    
    #theta measurements
    xVector = x-x_prev
    yVector = y-y_prev
    theta = np.arctan2(yVector, xVector)
    dt = t-t_prev
        
    #Model
    A = np.array([[1, 0, dt*np.cos(theta)], [0, 1, dt*np.sin(theta)], [0, 0, 1]])
    C = np.array([[1, 0, 0],[0, 1, 0]])
    
    #Prediction
    x_hat = A*x_hat
    predictedP = A*predictedP*A.transpose() + Q
    
    #update
    Knum = np.dot(predictedP,C.transpose())
    Kdenum = np.dot(C,np.dot(predictedP,C.transpose())) + R
    K = np.dot(Knum,np.linalg.inv(Kdenum))
    print(Knum)
    print(Kdenum)
    print(K)
    
    x_hat = x_hat + np.dot(K,(np.array([[x],[y],[v]])-np.dot(C,x_hat)))
    
    predictedP = predictedP - np.dot(K, np.dot(C,predictedP))
    
    return x_hat, predictedP
    
    
    
    
#!/usr/bin/env python

"""
Created on Tue Nov 27 17:23:20 2018
@author: mgreiff
"""

import numpy as np
import scipy
import scipy.linalg as sl
import matplotlib.pyplot as plt
from math import sin, cos, pi

# State vector
#
#     |   x   |
#     |   y   |
# x = | theta |
#     |  phi1 |
#     |  phi2 |
#
# Nonlinear flow field
#
#         | (r/2)*(phi1+phi2)*cos(theta) |
#         | (r/2)*(phi1+phi2)*sin(theta) |
# dx/dt = | (r/(2*d))*(phi1 - phi2)      |
#         |  0                           |
#         |  0                           |
#
# Measurement equation
#
#     |1 0 0  0  0 |
# y = |0 1 0  0  0 |
#     |0 0 0 c1  0 |
#     |0 0 0  0 c2 |


class ekf :
    def __init__(self, ) :

        self.m
        self.h
        self.r
        self.d
        self.m
        


    # Jacobians of the flow
    def get_flow_jacobians(m, h, r, d):
        # Unpack the state
        x, y, t, p1, p2 = m[0,0],m[1,0],m[2,0],m[3,0],m[4,0]
        #Evaluate the drift term
        f = np.array([
            [x + (h*r*cos(t)*(p1 + p2))/2.0],
            [y + (h*r*sin(t)*(p1 + p2))/2.0],
            [   t + (h*r*(p1 - p2))/(2.0*d)],
            [                            p1],
            [                            p2]])
        # Compute continuous time jacobians
        dfdx = np.array([
            [ 1.0, 0, -h*(r*sin(t)*(p1 + p2))/2.0, h*(r*cos(t))/2.0, h*(r*cos(t))/2.0],
            [ 0, 1.0,  h*(r*cos(t)*(p1 + p2))/2.0, h*(r*sin(t))/2.0, h*(r*sin(t))/2.0],
            [ 0,   0,                         1.0,      h*r/(2.0*d),     -h*r/(2.0*d)],
            [ 0,   0,                           0,              1.0,                0],
            [ 0,   0,                           0,                0,            1.0]])
        # Compute hessians of f[0] and f[1], all other f[n]=0
        ddfddx1 = np.array([
            [ 0, 0,                           0,                 0,                 0],
            [ 0, 0,                           0,                 0,                 0],
            [ 0, 0, -(h*r*cos(t)*(p1 + p2))/2.0, -(h*r*sin(t))/2.0, -(h*r*sin(t))/2.0],
            [ 0, 0,           -(h*r*sin(t))/2.0,                 0,                 0],
            [ 0, 0,           -(h*r*sin(t))/2.0,                 0,                 0]])
        ddfddx2 = np.array([
            [ 0, 0,                           0,                 0,                 0],
            [ 0, 0,                           0,                 0,                 0],
            [ 0, 0, -(h*r*sin(t)*(p1 + p2))/2.0,  (h*r*cos(t))/2.0,  (h*r*cos(t))/2.0],
            [ 0, 0,            (h*r*cos(t))/2.0,                 0,                 0],
            [ 0, 0,            (h*r*cos(t))/2.0,                 0,                 0]])
        return f, dfdx, ddfddx1, ddfddx2

    #Measurement jacobians
    # Jacobians of the state
    def get_measurement_jacobians(m, c1, c2):
        # Unpack the state
        x, y, t, p1, p2 = m[0,0], m[1,0], m[2,0], m[3,0], m[4,0]
        #Evaluate the drift term
        hx = np.array([
             [x],
             [y],
             [p1 / c1],
             [p2 / c2]])

        # Compute continuous time jacobians
        dhdx = np.array([
            [ 1.0,   0, 0,      0,      0],
            [   0, 1.0, 0,      0,      0],
            [   0,   0, 0, 1.0/c1,      0],
            [   0,   0, 0,      0, 1.0/c2]])
        return hx, dhdx

    def filter_update(ym, m, P, h,r, d, c1, c2, Q, R):
        # Update the current state estimate
        #
        e1, e2, e3, e4, e5 = np.eye(5)
        e1 = np.reshape(e1,[5,1])
        e2 = np.reshape(e2,[5,1])

        # Prediction
        fx, dfdx, ddfddx1, ddfddx2 = get_flow_jacobians(m, h, r, d)

        Pf1 = np.dot(ddfddx1, P)
        Pf2 = np.dot(ddfddx2, P)
        F11 = 0.5*(np.trace(np.dot(Pf1,Pf1)))*np.dot(e1,e1.T)
        F12 = 0.5*(np.trace(np.dot(Pf1,Pf2)))*np.dot(e1,e2.T)
        F21 = 0.5*(np.trace(np.dot(Pf2,Pf1)))*np.dot(e2,e1.T)
        F22 = 0.5*(np.trace(np.dot(Pf2,Pf2)))*np.dot(e2,e2.T)
        F1  = 0.5*np.trace(Pf1)*e1
        F2  = 0.5*np.trace(Pf2)*e2

        mkf = fx + F1 + F2
        Pkf = np.dot(np.dot(dfdx, P), dfdx.T) + Q + F11 + F21 + F12 + F22

        # Correction
        e1, e2, e3, e4 = np.eye(4)
        e1 = np.reshape(e1,[4,1])
        e2 = np.reshape(e2,[4,1])

        hx, dhdx = get_measurement_jacobians(mkf, c1, c2)

        Ph1 = np.dot(ddfddx1, P)
        Ph2 = np.dot(ddfddx2, P)
        H1  = 0.5*np.trace(Ph1)*e1
        H2  = 0.5*np.trace(Ph2)*e2

        e = ym - (hx + H1 + H2)
        Sk = R + np.dot(np.dot(dhdx,Pkf),dhdx.T)
        Kk = np.dot(np.dot(Pkf, dhdx.T),sl.inv(Sk))
        IKC = (np.eye(5)-np.dot(Kk,dhdx))

        mk = mkf + np.dot(Kk,e)
        Pk = np.dot(np.dot(IKC,Pkf),IKC.T) + np.dot(np.dot(Kk,R),Kk.T)
        return mk, Pk

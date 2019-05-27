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
    def __init__(self, Q, R) :

        self.h = 1/25.0     # Time step (set this yourselves)
        self.r = 0.125        # radius of the wheel (m)
        self.d = 0.235      # half distance between the wheels (m)

        # we moved 5.979 m and accumulated
        #
        #    12408 -(-15120)=2712 pixels on wheel 1
        #    11161 -  13863=-2702 pixels on wheel 2
        #
        # This means that 1 dpixel = 0.0022087181381603253 dmeter and that
        # 1 pixel/s = 0.0022087181381603253 m/s
        # This means that 1 pixel/s = 1/r Hz so a speed in pixels becomes
        self.c1 =  0.0022087181381603253 / 0.1 # Hz in the wheel speeds
        self.c2 = -0.0022087181381603253 / 0.1 # Hz in the wheel speeds
        #where one is negative as the cumulative pixel count seems to indicate that the
        #counter for wheel 2 is inverted

        self.Q = Q
        self.R = R

    # Jacobians of the flow
    def get_flow_jacobians(self, m): #, h, r, d):
        # Unpack the state
        x, y, t, p1, p2 = m[0,0],m[1,0],m[2,0],m[3,0],m[4,0]
        #Evaluate the drift term
        f = np.array([
            [x + (self.h*self.r*cos(t)*(p1 + p2))/2.0],
            [y + (self.h*self.r*sin(t)*(p1 + p2))/2.0],
            [   t + (self.h*self.r*(p1 - p2))/(2.0*self.d)],
            [                            p1],
            [                            p2]])
        # Compute continuous time jacobians
        dfdx = np.array([
            [ 1.0, 0, -self.h*(self.r*sin(t)*(p1 + p2))/2.0, self.h*(self.r*cos(t))/2.0, self.h*(self.r*cos(t))/2.0],
            [ 0, 1.0,  self.h*(self.r*cos(t)*(p1 + p2))/2.0, self.h*(self.r*sin(t))/2.0, self.h*(self.r*sin(t))/2.0],
            [ 0,   0,                         1.0,      self.h*self.r/(2.0*self.d),     -self.h*self.r/(2.0*self.d)],
            [ 0,   0,                           0,              1.0,                0],
            [ 0,   0,                           0,                0,            1.0]])
        # Compute hessians of f[0] and f[1], all other f[n]=0
        ddfddx1 = np.array([
            [ 0, 0,                           0,                 0,                 0],
            [ 0, 0,                           0,                 0,                 0],
            [ 0, 0, -(self.h*self.r*cos(t)*(p1 + p2))/2.0, -(self.h*self.r*sin(t))/2.0, -(self.h*self.r*sin(t))/2.0],
            [ 0, 0,           -(self.h*self.r*sin(t))/2.0,                 0,                 0],
            [ 0, 0,           -(self.h*self.r*sin(t))/2.0,                 0,                 0]])
        ddfddx2 = np.array([
            [ 0, 0,                           0,                 0,                 0],
            [ 0, 0,                           0,                 0,                 0],
            [ 0, 0, -(self.h*self.r*sin(t)*(p1 + p2))/2.0,  (self.h*self.r*cos(t))/2.0,  (self.h*self.r*cos(t))/2.0],
            [ 0, 0,            (self.h*self.r*cos(t))/2.0,                 0,                 0],
            [ 0, 0,            (self.h*self.r*cos(t))/2.0,                 0,                 0]])
        return f, dfdx, ddfddx1, ddfddx2

    #Measurement jacobians
    # Jacobians of the state
    def get_measurement_jacobians(self, m): #, c1, c2):
        # Unpack the state
        x, y, t, p1, p2 = m[0,0], m[1,0], m[2,0], m[3,0], m[4,0]
        #Evaluate the drift term
        hx = np.array([
             [x],
             [y],
             [p1 / self.c1],
             [p2 / self.c2]])

        # Compute continuous time jacobians
        dhdx = np.array([
            [ 1.0,   0, 0,      0,      0],
            [   0, 1.0, 0,      0,      0],
            [   0,   0, 0, 1.0/self.c1,      0],
            [   0,   0, 0,      0, 1.0/self.c2]])
        return hx, dhdx

    def filter_update(self, ym, m, P): # h, r, d, c1, c2, Q, R):
        # Update the current state estimate
        #
        e1, e2, e3, e4, e5 = np.eye(5)
        e1 = np.reshape(e1,[5,1])
        e2 = np.reshape(e2,[5,1])

        # Prediction
        fx, dfdx, ddfddx1, ddfddx2 = self.get_flow_jacobians(m) #, self.h, self.r, self.d)

        Pf1 = np.dot(ddfddx1, P)
        Pf2 = np.dot(ddfddx2, P)
        F11 = 0.5*(np.trace(np.dot(Pf1,Pf1)))*np.dot(e1,e1.T)
        F12 = 0.5*(np.trace(np.dot(Pf1,Pf2)))*np.dot(e1,e2.T)
        F21 = 0.5*(np.trace(np.dot(Pf2,Pf1)))*np.dot(e2,e1.T)
        F22 = 0.5*(np.trace(np.dot(Pf2,Pf2)))*np.dot(e2,e2.T)
        F1  = 0.5*np.trace(Pf1)*e1
        F2  = 0.5*np.trace(Pf2)*e2

        mkf = fx + F1 + F2
        Pkf = np.dot(np.dot(dfdx, P), dfdx.T) + self.Q + F11 + F21 + F12 + F22

        # Correction
        e1, e2, e3, e4 = np.eye(4)
        e1 = np.reshape(e1,[4,1])
        e2 = np.reshape(e2,[4,1])

        hx, dhdx = self.get_measurement_jacobians(mkf) #, self.c1, self.c2)

        Ph1 = np.dot(ddfddx1, P)
        Ph2 = np.dot(ddfddx2, P)
        H1  = 0.5*np.trace(Ph1)*e1
        H2  = 0.5*np.trace(Ph2)*e2

        e = ym - (hx + H1 + H2)
        Sk = self.R + np.dot(np.dot(dhdx,Pkf),dhdx.T)
        Kk = np.dot(np.dot(Pkf, dhdx.T),sl.inv(Sk))
        IKC = (np.eye(5)-np.dot(Kk,dhdx))

        mk = mkf + np.dot(Kk,e)
        Pk = np.dot(np.dot(IKC,Pkf),IKC.T) + np.dot(np.dot(Kk,self.R),Kk.T)
        return mk, Pk

if __name__ == "__main__":

    h = 1/25.0  # Time step (set this yourselves)

    # Setup
    N = 500
    states       = np.zeros((N,5,1))
    measurements = np.zeros((N,4,1))
    times        = np.zeros((N,1,1))

    ## Generate some random data to test the filter
    Q   = np.diag([0.001, 0.001, 0.01, 0.01, 0.01])
    Qsq = h*sl.cholesky(Q)
    R   = np.diag([0.01, 0.01, 1.0, 1.0])
    Rsq = sl.cholesky(R)

    my_ekf = ekf(Q,R)


    for k in range(1,N):

        # Find the drift term
        fx, _, _, _ = my_ekf.get_flow_jacobians(states[k-1]) #, h, r, d)
        # propagate the state with some input on the wheel accelerations
        du = h*np.array([[0],[0],[0],[sin(0.9*times[k-1])],[cos(0.6*times[k-1])]])
        states[k] = fx + du
        hx, _ = my_ekf.get_measurement_jacobians(states[k-1]) #, c1, c2)
        measurements[k] = hx + np.dot(Rsq, np.random.randn(4,1))
        times[k] =  times[k-1] + h



    #Plot results
    plt.figure(1)
    plt.subplot(411)
    plt.title('The true and correpted measurements\nof states used in the estimation')
    plt.plot(times[:,0],measurements[:,0],'b')
    plt.plot(times[:,0],states[:,0],'r')
    plt.legend(['Measured x', 'True x'])
    plt.subplot(412)
    plt.plot(times[:,0],measurements[:,1],'b')
    plt.plot(times[:,0],states[:,1],'r')
    plt.legend(['Measured y', 'True y'])
    plt.subplot(413)
    plt.plot(times[:,0],measurements[:,2],'b')
    plt.legend(['Measured wheel count 1'])
    plt.subplot(414)
    plt.plot(times[:,0],measurements[:,3],'b')
    plt.legend(['Measured wheel count 2'])

    plt.figure(2)
    for ii in range(5):
        plt.subplot(5,1,ii+1)
        plt.plot(times[:,0],states[:,ii],'r')


    ## Run the estimation on the generated data
    m  = np.zeros((N,5,1))
    P  = np.zeros((N,5,5))
    m0 = np.zeros((5,1))+1.0
    P0 = 0.1*np.eye(5)

    for k in range(1,len(times)):

        # Find the drift term
        yk = measurements[k]
        if k==1:
            mk, Pk = my_ekf.filter_update(yk, m0, P0) #, h, r, d, c1, c2, Q, R)
        else:
            mk, Pk = my_ekf.filter_update(yk, m[k-1], P[k-1]) #, h, r, d, c1, c2, Q, R)
        m[k] = mk
        P[k] = Pk


    plt.figure(3)
    for ii in range(5):
        plt.subplot(5,1,ii+1)
        plt.plot(times[:,0],m[:,ii],'b')
        plt.plot(times[:,0],states[:,ii],'r')

    plt.show()

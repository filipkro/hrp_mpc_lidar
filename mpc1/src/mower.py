import solver_interface as si
import numpy as np
import matplotlib.pyplot as plt
from math import atan2


si.set_defaults()
si.setup_indexing()

# Set parameters
#x_0
x_0 = [0.0, 0.0, 0.0]
u_0 = [0.0, 0.0]
si.set_parameters(0,0,0.0)
si.set_parameters(0,1,0.0)
si.set_parameters(0,2,0.0)

#Q
si.set_parameters(1,0,100.0)
si.set_parameters(1,4,100.0)
si.set_parameters(1,8,10.0)

#R
si.set_parameters(2,0,1.0)
si.set_parameters(2,3,1.0)

#u_max
si.set_parameters(5,0,10.0)
si.set_parameters(5,1,2.0)

#deltau_max
si.set_parameters(12,0,1.0)
si.set_parameters(12,1,1.0)

h = 0.1

t = np.linspace(0,200,400/h)

xref = 5*np.cos(t)
# print "xref: "
# print xref



# for i in range(1,100):
# ref.append(float(i))


u1used = [0.0]
u2used = [0.0]


for ind in range(200):
    print x_0
    
    #x_0
    si.set_parameters(0,0,x_0[0])
    si.set_parameters(0,1,x_0[1])
    si.set_parameters(0,2,x_0[2])

    #u_prev
    si.set_parameters(13,0,u_0[0])
    si.set_parameters(13,1,u_0[1])

    A = np.array([[1.0,0.0,-u_0[0]*h*np.sin(x_0[2])], [0.0,1.0,u_0[0]*h*np.cos(x_0[2])], [0.0,0.0,1.0]])
    #A = np.array([[1.0,0.0,0.0], [0.0,1.0,0.0], [0.0,0.0,1.0]])
    for i in range(3):
        for j in range(3):
            si.set_parameters(3,i+3*j,float(A[i,j]))

    B = np.array([[h*np.cos(x_0[2]),-u_0[0]*np.sin(x_0[2])*0.5*h**2], [h*np.sin(x_0[2]), u_0[0]*np.cos(x_0[2])*0.5*h**2], [0,h]])
    #B = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 1.0]])
    for i in range(3):
        for j in range(2):
            si.set_parameters(4,i+3*j,float(B[i,j]))

    #ref
    print "i: " + str(ind)
    # si.set_parameters(6,0,ref[ind])
    # si.set_parameters(6,1,ref[ind])
    # si.set_parameters(6,2,float(0))
    deltax = t[ind+5] - x_0[0]
    deltay = xref[ind+5] - x_0[1]
    theta = atan2(deltay, deltax)
    # theta = 0.0

    

    si.set_parameters(7,0,t[ind+1])
    si.set_parameters(7,1,xref[ind+1])
    si.set_parameters(7,2,theta)

    si.set_parameters(8,0,t[ind+2])
    si.set_parameters(8,1,xref[ind+2])
    si.set_parameters(8,2,theta)

    si.set_parameters(9,0,t[ind+3])
    si.set_parameters(9,1,xref[ind+3])
    si.set_parameters(9,2,theta)

    si.set_parameters(10,0,t[ind+4])
    si.set_parameters(10,1,xref[ind+4])
    si.set_parameters(10,2,theta)

    si.set_parameters(11,0,t[ind+5])
    si.set_parameters(11,1,xref[ind+5])
    si.set_parameters(11,2,theta)

    si.visualize_parameters()

    si.solve()
    pred = np.array(si.get_prediction_horizon())
    x1 = np.array(pred[::3])
    x2 = np.array(pred[1::3])
    x3 = np.array(pred[2::3])
    cont = np.array(si.get_control_horizon())
    u1 = np.array(cont[::2])
    u2 = np.array(cont[1::2])

    controlsig = si.get_control_horizon()
    uu1 = controlsig[::2]
    print "\nuu1: "
    print uu1
    
    print cont
    # print "x1: " +  str(x1)
    # print "x0: " + str(x_0)
    # print x_0[0]
    # print x1[0]
    # print x2[0]
    # x1[0] = x_0[0]
    # x1[1] = x_0[1]
    # x1[2] = x_0[2]
    x_0 = [x1[1], x2[1], x3[1]]
    u_0 = [u1[0], u2[0]]
    print "x1: " +  str(x1)
    print "x0: " + str(x_0)
    print x_0[0]

    xpos.append(x1[1])
    ypos.append(x2[1])
    thetapos.append(x3[1])

    u1used.append(u1[0])
    u2used.append(u2[0])

    si.visualize_parameters()
    si.visualize_variables()

    print x1

    # plt.figure(1)
    # plt.subplot(2,1,1)
    # pred = np.array(si.get_prediction_horizon())
    # plt.plot(x1[0:8],linestyle='-', color='r', linewidth=3)
    # plt.plot(x2[0:6],linestyle='--', color='b', linewidth=3)
    # plt.plot(x3[0:6],linestyle=':', color='g', linewidth=3)
    # plt.plot(xref[ind:ind+5],linestyle='-.', color='y', linewidth=3)
    # plt.legend(('x1','x2','x3'))

    # plt.subplot(2,1,2)
    # cont = np.array(si.get_control_horizon())
    # plt.plot(u1[1:6],linestyle='-', color='r', linewidth=3)
    # plt.plot(u2[1:6],linestyle='--', color='b', linewidth=3)
    # plt.legend(('u1','u2'))
    # plt.show()

#si.set_ref(float(5),float(2))


print "xpos"
print xpos

xn = np.array(xpos)
yn = np.array(ypos)
thtean = np.array(thetapos)

print xn
print yn
print thtean

u1n = np.array(u1used)
u2n = np.array(u2used)

print "u1: "
print u1n
print "u2: "
print u2n

# plt.figure(1)
# plt.subplot(2,1,1)
# plt.plot(xn,linestyle='-', color='r', linewidth=3)
# plt.plot(yn,linestyle='--', color='b', linewidth=3)
# plt.plot(thtean,linestyle=':', color='g', linewidth=3)
# plt.legend(('x1','x2','x3'))

# plt.subplot(2,1,2)
# plt.plot(u1n,linestyle='-', color='r', linewidth=3)
# plt.plot(u2n,linestyle='--', color='b', linewidth=3)
# plt.legend(('u1','u2'))
# plt.show()

plt.figure(1)
plt.subplot(2,1,1)
plt.plot(xpos[0:89],linestyle='-', color='r', linewidth=3)
plt.plot(ypos[0:89],linestyle='--', color='b', linewidth=3)
plt.plot(thetapos[1:89],linestyle=':', color='g', linewidth=3)
plt.plot(xref[1:89],linestyle='-.', color='y', linewidth=3)
plt.legend(('x1','x2','x3'))

plt.subplot(2,1,2)
plt.plot(u1used[0:89],linestyle='-', color='r', linewidth=3)
plt.plot(u2used[0:89],linestyle='--', color='b', linewidth=3)
plt.legend(('u1','u2'))
plt.show()

plt.figure(2)
plt.plot(xpos,ypos, linestyle='-', color='b', linewidth=3)
plt.plot(t,xref, linestyle='--', color='r', linewidth=3)
plt.show()
# plt.figure(1)
# plt.subplot(2,1,1)
# pred = np.array(si.get_prediction_horizon())
# plt.plot(pred[::3],linestyle='-', color='r', linewidth=3)
# plt.plot(pred[1::3],linestyle='--', color='b', linewidth=3)
# plt.plot(pred[2::3],linestyle=':', color='g', linewidth=3)
# plt.legend(('x1','x2','x3'))

# plt.subplot(2,1,2)
# cont = np.array(si.get_control_horizon())
# plt.plot(cont[::2],linestyle='-', color='r', linewidth=3)
# plt.plot(cont[1::2],linestyle='--', color='b', linewidth=3)
# plt.legend(('u1','u2'))
# plt.show()

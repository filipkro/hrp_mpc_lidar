import numpy as np
from math import atan

import matplotlib.pyplot as plt

def sigmoid(x):
    return (1 / (1 + np.exp(-x)))

h = 1
x_len = 6
run_time = 40
xref_long = np.linspace(0, x_len, run_time/h)
yref_long = 0*xref_long

for i in range(8):
    yref_long[i+5] = 0.15*i
    yref_long[34-i] = 0.15*i
yref_long[13:27] = 0.15*8

# for i in range(10):
#     yref_long[i+20] = 0.25*sigmoid(3*(xref_long[i]-x_len/2))

# for i in range(10):
#     yref_long[i+30] = yref_long[i+30-1]

# for i in range(len(xref_long)):
#     yref_long.append(0.25*atan(10*(xref_long[i]-x_len/2)))

# for i in range(len(xref_long)):
#     yref_long.append(-0.25*atan(10*(xref_long[i]-x_len/2)))

# xref_long = np.linspace(0, 2*x_len, 2*run_time/h)
# for i in range(len(xref_long)):
#     yref_long.append(0.25*sigmoid(10*(xref_long[i]-x_len/2)))
# # diff = yref_long[0]

# for i in range(len(xref_long)):
#     yref_long.append(0.25-0.25*sigmoid(10*(xref_long[i]-x_len/2)))

# for i in range(10):
#     yref_long.insert(0,0)

# xref_long = np.linspace(0, 2*x_len+10, (2*run_time+10)/h)
# for i in range(40):
#     yref_long[i] = yref_long[i] - diff 

print np.amin(yref_long)
print yref_long
print len(yref_long)





plt.figure(1)
plt.plot(xref_long,yref_long)
plt.show()
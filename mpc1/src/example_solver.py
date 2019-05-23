import solver_interface as si
import numpy as np
import matplotlib.pyplot as plt

si.set_defaults()
si.setup_indexing()

# Set parameters
#x_0
si.set_parameters(0,0,0.0)
si.set_parameters(0,1,0.0)
si.set_parameters(0,2,0.0)

si.set_parameters(1,0,100.0)
si.set_parameters(1,4,10.0)
si.set_parameters(1,8,0.1)

si.set_parameters(2,0,0.1)
si.set_parameters(2,3,1.0)

h = 0.5
A = np.array([[1.0,0.0,0.0], [0.0,1.0,0.0], [0.0,0.0,1.0]])
for i in range(3):
    for j in range(3):
        si.set_parameters(3,i+3*j,float(A[i,j]))

B = np.array([[h, 0.0], [0.0, h**2/2.0], [0,h]])
for i in range(3):
    for j in range(2):
        si.set_parameters(4,i+3*j,float(B[i,j]))

si.set_parameters(5,0,100.0)

si.set_parameters(6,0,float(1))
si.set_parameters(6,1,float(0.5))
si.set_parameters(6,2,float(0))

si.set_parameters(7,0,float(1))
si.set_parameters(7,1,float(1))
si.set_parameters(7,2,float(0))

#si.set_ref(float(5),float(2))


si.visualize_parameters()


si.solve()


plt.figure(1)
plt.subplot(2,1,1)
pred = np.array(si.get_prediction_horizon())
plt.plot(pred[::3],linestyle='-', color='r', linewidth=3)
plt.plot(pred[1::3],linestyle='--', color='b', linewidth=3)
plt.plot(pred[2::3],linestyle=':', color='g', linewidth=3)
plt.legend(('x1','x2','x3'))

plt.subplot(2,1,2)
cont = np.array(si.get_control_horizon())
plt.plot(cont[::2],linestyle='-', color='r', linewidth=3)
plt.plot(cont[1::2],linestyle='--', color='b', linewidth=3)
plt.legend(('u1','u2'))
plt.show()

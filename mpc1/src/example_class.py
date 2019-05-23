import solver_interface as si
import numpy as np
import matplotlib.pyplot as plt
import time

class CVXGENSolver(object):

    def __init__(self):
        """Return a Customer object whose name is *name* and starting
        balance is *balance*."""
        # Set up solver
        si.set_defaults()
        si.setup_indexing()
        si.load_default_data()

    def load_cvxgen_description(self):
        print("not yet implemented")

        # Execute the dimensions line by line
        exec("m = 2   # inputs.")
        exec("n = 3   # states.")
        exec("T = 30  # horizon.")

        self.parameters = {
            "A"    :{"id":3, "rows": n, "columns": n},
            "B"    :{"id":4, "rows": n, "columns": m},
            "Q"    :{"id":1, "rows": n, "columns": n},
            "R"    :{"id":2, "rows": m, "columns": m},
            "x[0]" :{"id":0, "rows": n, "columns": 1},
            "u_max":{"id":5, "rows": 1, "columns": 1}
        }

        self.variables = {


        }

    def set_parameter(self, parameter, value):
        if not parameter in self.parameters.keys():
            # Check that a valid parameter has been requested
            print("Could not find the parameter %s in the parameter definitions" % str(parameter))
        else:
            identifier = self.parameters[parameter]["id"]
            cols = self.parameters[parameter]["columns"]
            rows = self.parameters[parameter]["rows"]

            # Check that the inpu dimensionality matches
            if len(value.shape) != 2:
                print("The input must be of shape (%i,%i), not %s." % (cols, rows, str(value.shape)))
            elif value.shape[0] != rows or value.shape[1] != cols:
                print("The input must be of shape (%i,%i), not %s." % (cols, rows, str(value.shape)))

            for col in range(cols):
                for row in range(rows):
                    si.set_parameters(identifier, row+rows*col, float(value[row, col]))
                    print (identifier, row,col,row+rows*col, value[row, col])

    def visualize_parameters(self):
        si.visualize_parameters()

    def solve(self):
        si.solve()

    def visualize_variables(self):
        plt.figure(1)
        plt.subplot(2,1,1)
        pred = np.array(si.get_prediction_horizon())
        plt.plot(pred[3::3],linestyle='-', color='r', linewidth=3)
        plt.plot(pred[4::3],linestyle='--', color='b', linewidth=3)
        plt.plot(pred[5::3],linestyle=':', color='g', linewidth=3)
        plt.legend(('x1','x2','x3'))

        plt.subplot(2,1,2)
        cont = np.array(si.get_control_horizon())
        plt.plot(cont[2::2],linestyle='-', color='r', linewidth=3)
        plt.plot(cont[3::2],linestyle='--', color='b', linewidth=3)
        plt.legend(('u1','u2'))
        plt.show()

sol = CVXGENSolver()
sol.load_cvxgen_description()

# Dynamics of a triple integraor with two inputs
h = 0.1
A = np.array([[1.0,h,h**2/2.0], [0.0,1.0,h], [0.0,0.0,1.0]])
B = np.array([[h**3/6.0, h**3/6.0], [h**2/2.0, h**2/2.0], [h,h]]) / 2.0
Q = 100*np.eye(3)
R = 0.1*np.eye(2)
x0 = np.array([[1.0],[2.0],[-5.0]])
u_max =  np.array([[10.0]])

# Set the solver parameters
sol.set_parameter("A", A)
sol.set_parameter("B", B)
sol.set_parameter("Q", Q)
sol.set_parameter("R", R)
sol.set_parameter("x[0]", x0)
sol.set_parameter("u_max", u_max)

# Visualize the set parameters in the solver
sol.visualize_parameters()

# Optimize
sol.solve()

# Visualize state and control horizons
sol.visualize_variables()

# Compute average solution time
t1 = time.time()
N = 1000
for ii in range(N):
    sol.solve()
totalTime = time.time()-t1
print("Solved %i problems in %f [s], average time %f [s]" % (N, totalTime, totalTime/ float(N)))

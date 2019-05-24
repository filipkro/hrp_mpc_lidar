echo "Removing old executables..."
rm -rf *.o
rm -rf *.so

echo "Generating handles..."
#python generate_handle.py

echo "Seting swig interface..."
swig -python solver_interface.i

echo "Compiling position independent libraries..."
gcc -O2 -fPIC -c solver_interface.c cvxgen/solver.c cvxgen/matrix_support.c cvxgen/util.c cvxgen/ldl.c
gcc -O2 -fPIC -c solver_interface_wrap.c -I/usr/include/python2.7
gcc -shared solver.o matrix_support.o ldl.o solver_interface.o util.o solver_interface_wrap.o -o _solver_interface.so

echo "Done!"

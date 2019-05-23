/* solver_interface.i */
%module solver_interface
%{
/* Put header files here or function declarations like below */
#include "cvxgen/solver.h"
#include "solver_interface.h"

extern float * get_prediction_horizon();
extern float * get_control_horizon();
extern float * get_control();

%}

/* From solver.h */
extern void set_defaults(void);
extern void setup_indexing(void);
extern void load_default_data(void);
extern long solve(void);

/* From solver_interface.h */
extern void set_parameters(int id, int index, float val);
extern void visualize_parameters(void);
extern void visualize_variables(void);

/* Typemaps */
%typemap(out) float* get_prediction_horizon {
  int number_of_variables = NUM_VARS_COLS_X*NUM_VARS_ROWS_X;
  int j;
  $result = PyList_New(number_of_variables);
  for (j = 0; j < number_of_variables; j++) {
    PyObject *o = PyFloat_FromDouble((double) $1[j]);
    PyList_SetItem($result,j,o);
  }
}

%typemap(out) float* get_control_horizon {
  int number_of_variables = NUM_VARS_COLS_U*NUM_VARS_ROWS_U;
  int i;
  $result = PyList_New(number_of_variables);
  for (i = 0; i < number_of_variables; i++) {
    PyObject *o = PyFloat_FromDouble((double) $1[i]);
    PyList_SetItem($result,i,o);
  }
}

%typemap(out) float* get_control {
  int number_of_variables = 2;
  int i;
  //$1, $1_dim0, $1_dim1
  $result = PyList_New(number_of_variables);
  for (i = 0; i < number_of_variables; i++) {
    PyObject *o = PyFloat_FromDouble((double) $1[i]);
    PyList_SetItem($result,i,o);
  }
}

%include "solver_interface.c"

#ifndef SOLVER_INTERFACE_H
#define SOLVER_INTERFACE_H

#include "cvxgen/solver.h"
#include <stdio.h>

void set_parameters(int id, int index, float val);
void visualize_parameters(void);
void visualize_variables(void);

#define NUM_VARS_COLS_X 5
#define NUM_VARS_ROWS_X 3
#define NUM_VARS_COLS_U 5
#define NUM_VARS_ROWS_U 2

#endif // MODULE_INTERFACE_H

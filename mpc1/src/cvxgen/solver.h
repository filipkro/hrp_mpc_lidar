/* Produced by CVXGEN, 2019-05-18 09:53:59 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t {
  double r_1[3];
  double Q[9];
  double R[4];
  double r_2[3];
  double r_3[3];
  double r_4[3];
  double r_5[3];
  double u_prev[2];
  double A[9];
  double x_0[3];
  double B[6];
  double u_max[2];
  double deltau_max[2];
  double *r[6];
  double *x[1];
} Params;
typedef struct Vars_t {
  double *x_1; /* 3 rows. */
  double *t_01; /* 2 rows. */
  double *x_2; /* 3 rows. */
  double *t_02; /* 2 rows. */
  double *x_3; /* 3 rows. */
  double *t_03; /* 2 rows. */
  double *x_4; /* 3 rows. */
  double *t_04; /* 2 rows. */
  double *x_5; /* 3 rows. */
  double *t_05; /* 2 rows. */
  double *u_0; /* 2 rows. */
  double *u_1; /* 2 rows. */
  double *u_2; /* 2 rows. */
  double *u_3; /* 2 rows. */
  double *u_4; /* 2 rows. */
  double *u_5; /* 2 rows. */
  double *t_06; /* 1 rows. */
  double *t_07; /* 1 rows. */
  double *t_08; /* 1 rows. */
  double *t_09; /* 1 rows. */
  double *t_10; /* 1 rows. */
  double *t_11; /* 1 rows. */
  double *t_12; /* 2 rows. */
  double *t_13; /* 2 rows. */
  double *t_14; /* 2 rows. */
  double *t_15; /* 2 rows. */
  double *t_16; /* 2 rows. */
  double *t_17; /* 2 rows. */
  double *t_18; /* 2 rows. */
  double *t_19; /* 2 rows. */
  double *t_20; /* 2 rows. */
  double *t_21; /* 2 rows. */
  double *x_6; /* 3 rows. */
  double *x[7];
  double *u[6];
} Vars;
typedef struct Workspace_t {
  double h[90];
  double s_inv[90];
  double s_inv_z[90];
  double b[28];
  double q[66];
  double rhs[274];
  double x[274];
  double *s;
  double *z;
  double *y;
  double lhs_aff[274];
  double lhs_cc[274];
  double buffer[274];
  double buffer2[274];
  double KKT[609];
  double L[561];
  double d[274];
  double v[274];
  double d_inv[274];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_548686778368[1];
  double quad_183018455040[1];
  double quad_19734581248[1];
  double quad_653725216768[1];
  double quad_975577374720[1];
  double quad_942105673728[1];
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_params(void);
void setup_indexed_optvars(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
int main(int argc, char **argv);
void load_default_data(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

#endif

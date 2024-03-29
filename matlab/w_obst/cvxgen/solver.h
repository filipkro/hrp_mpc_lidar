/* Produced by CVXGEN, 2019-04-16 07:32:34 -0400.  */
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
  double r_1[5];
  double Q[25];
  double R[4];
  double r_2[5];
  double r_3[5];
  double r_4[5];
  double r_5[5];
  double r_6[5];
  double r_7[5];
  double r_8[5];
  double r_9[5];
  double r_10[5];
  double r_11[5];
  double Q_final[25];
  double x_0[5];
  double r_0[5];
  double u_prev[2];
  double A[25];
  double B[10];
  double u_max[2];
  double deltau_max[2];
  double dist[2];
  double *r[12];
  double *x[1];
} Params;
typedef struct Vars_t {
  double *x_1; /* 5 rows. */
  double *t_01; /* 2 rows. */
  double *x_2; /* 5 rows. */
  double *t_02; /* 2 rows. */
  double *x_3; /* 5 rows. */
  double *t_03; /* 2 rows. */
  double *x_4; /* 5 rows. */
  double *t_04; /* 2 rows. */
  double *x_5; /* 5 rows. */
  double *t_05; /* 2 rows. */
  double *x_6; /* 5 rows. */
  double *t_06; /* 2 rows. */
  double *x_7; /* 5 rows. */
  double *t_07; /* 2 rows. */
  double *x_8; /* 5 rows. */
  double *t_08; /* 2 rows. */
  double *x_9; /* 5 rows. */
  double *t_09; /* 2 rows. */
  double *x_10; /* 5 rows. */
  double *t_10; /* 2 rows. */
  double *x_11; /* 5 rows. */
  double *u_0; /* 2 rows. */
  double *t_11; /* 1 rows. */
  double *t_12; /* 1 rows. */
  double *u_1; /* 2 rows. */
  double *t_13; /* 1 rows. */
  double *u_2; /* 2 rows. */
  double *t_14; /* 1 rows. */
  double *u_3; /* 2 rows. */
  double *t_15; /* 1 rows. */
  double *u_4; /* 2 rows. */
  double *t_16; /* 1 rows. */
  double *u_5; /* 2 rows. */
  double *t_17; /* 1 rows. */
  double *u_6; /* 2 rows. */
  double *t_18; /* 1 rows. */
  double *u_7; /* 2 rows. */
  double *t_19; /* 1 rows. */
  double *u_8; /* 2 rows. */
  double *t_20; /* 1 rows. */
  double *u_9; /* 2 rows. */
  double *t_21; /* 1 rows. */
  double *u_10; /* 2 rows. */
  double *t_22; /* 1 rows. */
  double *t_23; /* 1 rows. */
  double *t_24; /* 1 rows. */
  double *t_25; /* 1 rows. */
  double *t_26; /* 1 rows. */
  double *t_27; /* 1 rows. */
  double *t_28; /* 1 rows. */
  double *t_29; /* 1 rows. */
  double *t_30; /* 1 rows. */
  double *t_31; /* 1 rows. */
  double *t_32; /* 1 rows. */
  double *t_33; /* 1 rows. */
  double *t_34; /* 1 rows. */
  double *t_35; /* 1 rows. */
  double *t_36; /* 1 rows. */
  double *t_37; /* 1 rows. */
  double *t_38; /* 1 rows. */
  double *t_39; /* 1 rows. */
  double *t_40; /* 1 rows. */
  double *t_41; /* 1 rows. */
  double *t_42; /* 1 rows. */
  double *t_43; /* 1 rows. */
  double *t_44; /* 1 rows. */
  double *t_45; /* 1 rows. */
  double *t_46; /* 1 rows. */
  double *t_47; /* 1 rows. */
  double *t_48; /* 1 rows. */
  double *t_49; /* 1 rows. */
  double *t_50; /* 1 rows. */
  double *t_51; /* 1 rows. */
  double *t_52; /* 1 rows. */
  double *t_53; /* 1 rows. */
  double *t_54; /* 1 rows. */
  double *t_55; /* 1 rows. */
  double *t_56; /* 1 rows. */
  double *t_57; /* 1 rows. */
  double *t_58; /* 1 rows. */
  double *t_59; /* 1 rows. */
  double *t_60; /* 1 rows. */
  double *t_61; /* 1 rows. */
  double *t_62; /* 1 rows. */
  double *t_63; /* 1 rows. */
  double *t_64; /* 1 rows. */
  double *t_65; /* 1 rows. */
  double *t_66; /* 1 rows. */
  double *t_67; /* 1 rows. */
  double *t_68; /* 1 rows. */
  double *t_69; /* 1 rows. */
  double *t_70; /* 1 rows. */
  double *t_71; /* 1 rows. */
  double *t_72; /* 1 rows. */
  double *t_73; /* 1 rows. */
  double *t_74; /* 1 rows. */
  double *x[12];
  double *u[11];
} Vars;
typedef struct Workspace_t {
  double h[192];
  double s_inv[192];
  double s_inv_z[192];
  double b[75];
  double q[161];
  double rhs[620];
  double x[620];
  double *s;
  double *z;
  double *y;
  double lhs_aff[620];
  double lhs_cc[620];
  double buffer[620];
  double buffer2[620];
  double KKT[1608];
  double L[1738];
  double d[620];
  double v[620];
  double d_inv[620];
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
  double quad_744270442496[1];
  double quad_560115118080[1];
  double quad_6032879616[1];
  double quad_611032489984[1];
  double quad_522860462080[1];
  double quad_497797779456[1];
  double quad_871133241344[1];
  double quad_176290516992[1];
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

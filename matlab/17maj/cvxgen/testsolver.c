/* Produced by CVXGEN, 2019-05-18 09:53:59 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.r_1[0] = 0.20319161029830202;
  params.r_1[1] = 0.8325912904724193;
  params.r_1[2] = -0.8363810443482227;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q[0] = 1.510827605197663;
  params.Q[3] = 0;
  params.Q[6] = 0;
  params.Q[1] = 0;
  params.Q[4] = 1.8929469543476547;
  params.Q[7] = 0;
  params.Q[2] = 0;
  params.Q[5] = 0;
  params.Q[8] = 1.896293088933438;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.R[0] = 1.1255853104638363;
  params.R[2] = 0;
  params.R[1] = 0;
  params.R[3] = 1.2072428781381868;
  params.r_2[0] = -1.7941311867966805;
  params.r_2[1] = -0.23676062539745413;
  params.r_2[2] = -1.8804951564857322;
  params.r_3[0] = -0.17266710242115568;
  params.r_3[1] = 0.596576190459043;
  params.r_3[2] = -0.8860508694080989;
  params.r_4[0] = 0.7050196079205251;
  params.r_4[1] = 0.3634512696654033;
  params.r_4[2] = -1.9040724704913385;
  params.r_5[0] = 0.23541635196352795;
  params.r_5[1] = -0.9629902123701384;
  params.r_5[2] = -0.3395952119597214;
  params.u_prev[0] = -0.865899672914725;
  params.u_prev[1] = 0.7725516732519853;
  params.A[0] = -0.23818512931704205;
  params.A[1] = -1.372529046100147;
  params.A[2] = 0.17859607212737894;
  params.A[3] = 1.1212590580454682;
  params.A[4] = -0.774545870495281;
  params.A[5] = -1.1121684642712744;
  params.A[6] = -0.44811496977740495;
  params.A[7] = 1.7455345994417217;
  params.A[8] = 1.9039816898917352;
  params.x_0[0] = 0.6895347036512547;
  params.x_0[1] = 1.6113364341535923;
  params.x_0[2] = 1.383003485172717;
  params.B[0] = -0.48802383468444344;
  params.B[1] = -1.631131964513103;
  params.B[2] = 0.6136436100941447;
  params.B[3] = 0.2313630495538037;
  params.B[4] = -0.5537409477496875;
  params.B[5] = -1.0997819806406723;
  params.u_max[0] = 0.8130398327524972;
  params.u_max[1] = 0.9378804973983381;
  params.deltau_max[0] = 0.5384711565021225;
  params.deltau_max[1] = 0.5835855484508652;
}

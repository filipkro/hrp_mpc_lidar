/* Produced by CVXGEN, 2019-04-16 03:46:29 -0400.  */
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
  params.r_6[0] = -0.865899672914725;
  params.r_6[1] = 0.7725516732519853;
  params.r_6[2] = -0.23818512931704205;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q_final[0] = 1.1568677384749633;
  params.Q_final[3] = 0;
  params.Q_final[6] = 0;
  params.Q_final[1] = 0;
  params.Q_final[4] = 1.5446490180318446;
  params.Q_final[7] = 0;
  params.Q_final[2] = 0;
  params.Q_final[5] = 0;
  params.Q_final[8] = 1.780314764511367;
  params.x_0[0] = -0.774545870495281;
  params.x_0[1] = -1.1121684642712744;
  params.x_0[2] = -0.44811496977740495;
  params.r_0[0] = 1.7455345994417217;
  params.r_0[1] = 1.9039816898917352;
  params.r_0[2] = 0.6895347036512547;
  params.u_prev[0] = 1.6113364341535923;
  params.u_prev[1] = 1.383003485172717;
  params.A[0] = -0.48802383468444344;
  params.A[1] = -1.631131964513103;
  params.A[2] = 0.6136436100941447;
  params.A[3] = 0.2313630495538037;
  params.A[4] = -0.5537409477496875;
  params.A[5] = -1.0997819806406723;
  params.A[6] = -0.3739203344950055;
  params.A[7] = -0.12423900520332376;
  params.A[8] = -0.923057686995755;
  params.B[0] = -0.8328289030982696;
  params.B[1] = -0.16925440270808823;
  params.B[2] = 1.442135651787706;
  params.B[3] = 0.34501161787128565;
  params.B[4] = -0.8660485502711608;
  params.B[5] = -0.8880899735055947;
  params.u_max[0] = 0.9092441510438936;
  params.u_max[1] = 0.41082068920997505;
  params.deltau_max[0] = 0.4027574220861463;
  params.deltau_max[1] = 1.0280701196348838;
}

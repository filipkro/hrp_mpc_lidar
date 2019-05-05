/* Produced by CVXGEN, 2019-04-16 07:32:34 -0400.  */
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
  params.r_1[3] = 0.04331042079065206;
  params.r_1[4] = 1.5717878173906188;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q[0] = 1.896293088933438;
  params.Q[5] = 0;
  params.Q[10] = 0;
  params.Q[15] = 0;
  params.Q[20] = 0;
  params.Q[1] = 0;
  params.Q[6] = 1.1255853104638363;
  params.Q[11] = 0;
  params.Q[16] = 0;
  params.Q[21] = 0;
  params.Q[2] = 0;
  params.Q[7] = 0;
  params.Q[12] = 1.2072428781381868;
  params.Q[17] = 0;
  params.Q[22] = 0;
  params.Q[3] = 0;
  params.Q[8] = 0;
  params.Q[13] = 0;
  params.Q[18] = 1.0514672033008299;
  params.Q[23] = 0;
  params.Q[4] = 0;
  params.Q[9] = 0;
  params.Q[14] = 0;
  params.Q[19] = 0;
  params.Q[24] = 1.4408098436506365;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.R[0] = 1.0298762108785668;
  params.R[2] = 0;
  params.R[1] = 0;
  params.R[3] = 1.456833224394711;
  params.r_2[0] = 0.596576190459043;
  params.r_2[1] = -0.8860508694080989;
  params.r_2[2] = 0.7050196079205251;
  params.r_2[3] = 0.3634512696654033;
  params.r_2[4] = -1.9040724704913385;
  params.r_3[0] = 0.23541635196352795;
  params.r_3[1] = -0.9629902123701384;
  params.r_3[2] = -0.3395952119597214;
  params.r_3[3] = -0.865899672914725;
  params.r_3[4] = 0.7725516732519853;
  params.r_4[0] = -0.23818512931704205;
  params.r_4[1] = -1.372529046100147;
  params.r_4[2] = 0.17859607212737894;
  params.r_4[3] = 1.1212590580454682;
  params.r_4[4] = -0.774545870495281;
  params.r_5[0] = -1.1121684642712744;
  params.r_5[1] = -0.44811496977740495;
  params.r_5[2] = 1.7455345994417217;
  params.r_5[3] = 1.9039816898917352;
  params.r_5[4] = 0.6895347036512547;
  params.r_6[0] = 1.6113364341535923;
  params.r_6[1] = 1.383003485172717;
  params.r_6[2] = -0.48802383468444344;
  params.r_6[3] = -1.631131964513103;
  params.r_6[4] = 0.6136436100941447;
  params.r_7[0] = 0.2313630495538037;
  params.r_7[1] = -0.5537409477496875;
  params.r_7[2] = -1.0997819806406723;
  params.r_7[3] = -0.3739203344950055;
  params.r_7[4] = -0.12423900520332376;
  params.r_8[0] = -0.923057686995755;
  params.r_8[1] = -0.8328289030982696;
  params.r_8[2] = -0.16925440270808823;
  params.r_8[3] = 1.442135651787706;
  params.r_8[4] = 0.34501161787128565;
  params.r_9[0] = -0.8660485502711608;
  params.r_9[1] = -0.8880899735055947;
  params.r_9[2] = -0.1815116979122129;
  params.r_9[3] = -1.17835862158005;
  params.r_9[4] = -1.1944851558277074;
  params.r_10[0] = 0.05614023926976763;
  params.r_10[1] = -1.6510825248767813;
  params.r_10[2] = -0.06565787059365391;
  params.r_10[3] = -0.5512951504486665;
  params.r_10[4] = 0.8307464872626844;
  params.r_11[0] = 0.9869848924080182;
  params.r_11[1] = 0.7643716874230573;
  params.r_11[2] = 0.7567216550196565;
  params.r_11[3] = -0.5055995034042868;
  params.r_11[4] = 0.6725392189410702;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q_final[0] = 1.339848663956818;
  params.Q_final[5] = 0;
  params.Q_final[10] = 0;
  params.Q_final[15] = 0;
  params.Q_final[20] = 0;
  params.Q_final[1] = 0;
  params.Q_final[6] = 1.572793869868875;
  params.Q_final[11] = 0;
  params.Q_final[16] = 0;
  params.Q_final[21] = 0;
  params.Q_final[2] = 0;
  params.Q_final[7] = 0;
  params.Q_final[12] = 1.3258071580648745;
  params.Q_final[17] = 0;
  params.Q_final[22] = 0;
  params.Q_final[3] = 0;
  params.Q_final[8] = 0;
  params.Q_final[13] = 0;
  params.Q_final[18] = 1.445145049263532;
  params.Q_final[23] = 0;
  params.Q_final[4] = 0;
  params.Q_final[9] = 0;
  params.Q_final[14] = 0;
  params.Q_final[19] = 0;
  params.Q_final[24] = 1.0615289308299394;
  params.x_0[0] = -1.0292983112626475;
  params.x_0[1] = 1.8864104246942706;
  params.x_0[2] = -1.077663182579704;
  params.x_0[3] = 0.7659100437893209;
  params.x_0[4] = 0.6019074328549583;
  params.r_0[0] = 0.8957565577499285;
  params.r_0[1] = -0.09964555746227477;
  params.r_0[2] = 0.38665509840745127;
  params.r_0[3] = -1.7321223042686946;
  params.r_0[4] = -1.7097514487110663;
  params.u_prev[0] = -1.2040958948116867;
  params.u_prev[1] = -1.3925560119658358;
  params.A[0] = -1.5995826216742213;
  params.A[1] = -1.4828245415645833;
  params.A[2] = 0.21311092723061398;
  params.A[3] = -1.248740700304487;
  params.A[4] = 1.808404972124833;
  params.A[5] = 0.7264471152297065;
  params.A[6] = 0.16407869343908477;
  params.A[7] = 0.8287224032315907;
  params.A[8] = -0.9444533161899464;
  params.A[9] = 1.7069027370149112;
  params.A[10] = 1.3567722311998827;
  params.A[11] = 0.9052779937121489;
  params.A[12] = -0.07904017565835986;
  params.A[13] = 1.3684127435065871;
  params.A[14] = 0.979009293697437;
  params.A[15] = 0.6413036255984501;
  params.A[16] = 1.6559010680237511;
  params.A[17] = 0.5346622551502991;
  params.A[18] = -0.5362376605895625;
  params.A[19] = 0.2113782926017822;
  params.A[20] = -1.2144776931994525;
  params.A[21] = -1.2317108144255875;
  params.A[22] = 0.9026784957312834;
  params.A[23] = 1.1397468137245244;
  params.A[24] = 1.8883934547350631;
  params.B[0] = 1.4038856681660068;
  params.B[1] = 0.17437730638329096;
  params.B[2] = -1.6408365219077408;
  params.B[3] = -0.04450702153554875;
  params.B[4] = 1.7117453902485025;
  params.B[5] = 1.1504727980139053;
  params.B[6] = -0.05962309578364744;
  params.B[7] = -0.1788825540764547;
  params.B[8] = -1.1280569263625857;
  params.B[9] = -1.2911464767927057;
  params.u_max[0] = 0.14724733843871518;
  params.u_max[1] = 1.784786375174185;
  params.deltau_max[0] = 1.2803532337981178;
  params.deltau_max[1] = 0.2866646349426427;
  params.dist[0] = -0.3434923211351708;
  params.dist[1] = -1.8035643024085055;
}

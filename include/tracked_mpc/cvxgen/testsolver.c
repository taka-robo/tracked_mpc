/* Produced by CVXGEN, 2022-12-07 11:01:32 -0500.  */
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
  params.x_ref_1[0] = 0.20319161029830202;
  params.x_ref_1[1] = 0.8325912904724193;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q[0] = 1.2909047389129444;
  params.Q[2] = 0;
  params.Q[1] = 0;
  params.Q[3] = 1.510827605197663;
  params.u_ref_1[0] = 1.5717878173906188;
  params.u_ref_1[1] = 1.5851723557337523;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.R[0] = 1.1255853104638363;
  params.R[2] = 0;
  params.R[1] = 0;
  params.R[3] = 1.2072428781381868;
  params.x_ref_2[0] = -1.7941311867966805;
  params.x_ref_2[1] = -0.23676062539745413;
  params.u_ref_2[0] = -1.8804951564857322;
  params.u_ref_2[1] = -0.17266710242115568;
  params.x_ref_3[0] = 0.596576190459043;
  params.x_ref_3[1] = -0.8860508694080989;
  params.u_ref_3[0] = 0.7050196079205251;
  params.u_ref_3[1] = 0.3634512696654033;
  params.x_ref_4[0] = -1.9040724704913385;
  params.x_ref_4[1] = 0.23541635196352795;
  params.u_ref_4[0] = -0.9629902123701384;
  params.u_ref_4[1] = -0.3395952119597214;
  params.x_ref_5[0] = -0.865899672914725;
  params.x_ref_5[1] = 0.7725516732519853;
  params.u_ref_5[0] = -0.23818512931704205;
  params.u_ref_5[1] = -1.372529046100147;
  params.x_ref_6[0] = 0.17859607212737894;
  params.x_ref_6[1] = 1.1212590580454682;
  params.u_ref_6[0] = -0.774545870495281;
  params.u_ref_6[1] = -1.1121684642712744;
  params.x_ref_7[0] = -0.44811496977740495;
  params.x_ref_7[1] = 1.7455345994417217;
  params.u_ref_7[0] = 1.9039816898917352;
  params.u_ref_7[1] = 0.6895347036512547;
  params.x_ref_8[0] = 1.6113364341535923;
  params.x_ref_8[1] = 1.383003485172717;
  params.u_ref_8[0] = -0.48802383468444344;
  params.u_ref_8[1] = -1.631131964513103;
  params.x_ref_9[0] = 0.6136436100941447;
  params.x_ref_9[1] = 0.2313630495538037;
  params.u_ref_9[0] = -0.5537409477496875;
  params.u_ref_9[1] = -1.0997819806406723;
  params.x_ref_10[0] = -0.3739203344950055;
  params.x_ref_10[1] = -0.12423900520332376;
  params.u_ref_10[0] = -0.923057686995755;
  params.u_ref_10[1] = -0.8328289030982696;
  params.A[0] = -0.16925440270808823;
  params.A[1] = 1.442135651787706;
  params.A[2] = 0.34501161787128565;
  params.A[3] = -0.8660485502711608;
  params.x_0[0] = -0.8880899735055947;
  params.x_0[1] = -0.1815116979122129;
  params.B[0] = -1.17835862158005;
  params.B[1] = -1.1944851558277074;
  params.B[2] = 0.05614023926976763;
  params.B[3] = -1.6510825248767813;
  params.u_0[0] = -0.06565787059365391;
  params.u_0[1] = -0.5512951504486665;
  params.w_0[0] = 0.8307464872626844;
  params.w_0[1] = 0.9869848924080182;
  params.w_1[0] = 0.7643716874230573;
  params.w_1[1] = 0.7567216550196565;
  params.w_2[0] = -0.5055995034042868;
  params.w_2[1] = 0.6725392189410702;
  params.w_3[0] = -0.6406053441727284;
  params.w_3[1] = 0.29117547947550015;
  params.w_4[0] = -0.6967713677405021;
  params.w_4[1] = -0.21941980294587182;
  params.w_5[0] = -1.753884276680243;
  params.w_5[1] = -1.0292983112626475;
  params.w_6[0] = 1.8864104246942706;
  params.w_6[1] = -1.077663182579704;
  params.w_7[0] = 0.7659100437893209;
  params.w_7[1] = 0.6019074328549583;
  params.w_8[0] = 0.8957565577499285;
  params.w_8[1] = -0.09964555746227477;
  params.w_9[0] = 0.38665509840745127;
  params.w_9[1] = -1.7321223042686946;
  params.w_10[0] = -1.7097514487110663;
  params.w_10[1] = -1.2040958948116867;
  params.u_max[0] = 0.3037219940170821;
  params.u_max[1] = 0.20020868916288936;
}

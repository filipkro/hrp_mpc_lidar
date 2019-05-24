#include "solver_interface.h"

#define CONTROL_DIM 2
#define STATE_DIM 3
#define HORIZON 5

Vars vars;
Params params;
Workspace work;
Settings settings;

void load_default_data(void) {
  settings.verbose = 0;

  params.x_0[0] = 0.0;
  params.x_0[1] = 0.0;
  params.x_0[2] = 0.0;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q[0] = 1.0;
  params.Q[3] = 0;
  params.Q[6] = 0;
  params.Q[1] = 0;
  params.Q[4] = 1.0;
  params.Q[7] = 0;
  params.Q[2] = 0;
  params.Q[5] = 0;
  params.Q[8] = 1.0;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.R[0] = 1.0;
  params.R[2] = 0;
  params.R[1] = 0;
  params.R[3] = 1.0;
  params.A[0] = -1.7941311867966805;
  params.A[1] = -0.23676062539745413;
  params.A[2] = -1.8804951564857322;
  params.A[3] = -0.17266710242115568;
  params.A[4] = 0.596576190459043;
  params.A[5] = -0.8860508694080989;
  params.A[6] = 0.7050196079205251;
  params.A[7] = 0.3634512696654033;
  params.A[8] = -1.9040724704913385;
  params.B[0] = 0.23541635196352795;
  params.B[1] = -0.9629902123701384;
  params.B[2] = -0.3395952119597214;
  params.B[3] = -0.865899672914725;
  params.B[4] = 0.7725516732519853;
  params.B[5] = -0.23818512931704205;
  params.u_max[0] = 5;
  params.u_max[1] = 3.14;
  params.deltau_max[0] = 3;
  params.deltau_max[1] = 1;
}

void visualize_parameters(void){
  printmatrix("x_0", params.x_0, 3, 1, 0);
  printmatrix("Q", params.Q, 3, 3, 0);
  printmatrix("R", params.R, 2, 2, 0);
  printmatrix("A", params.A, 3, 3, 0);
  printmatrix("B", params.B, 3, 2, 0);
  printmatrix("u_max", params.u_max, 2, 1, 0);
  // printmatrix("r_4", params.r_4, 3, 1, 0);
  // printmatrix("r_5", params.r_5, 3, 1, 0);
}

void visualize_variables(void){
  printmatrix("x_0", vars.x_1, 3, 1, 0);
  printmatrix("x_1", vars.x_2, 3, 1, 0);
  printmatrix("u_0", vars.u_0, 2, 1, 0);
  printmatrix("u_1", vars.u_1, 2, 1, 0);
  printmatrix("u_2", vars.u_2, 2, 1, 0);
}

void set_parameters(int id, int index, float val){
  if (id == 0){
    params.x_0[index] = val;
  } else if (id == 1){
    params.Q[index] = val;
  } else if (id == 2){
    params.R[index] = val;
  } else if (id == 3){
    params.A[index] = val;
  } else if (id == 4){
    params.B[index] = val;
  } else if (id == 5){
    params.u_max[index] = val;
 // } else if (id == 6){
   // params.r_0[index] = val;
  } else if (id == 7){
    params.r_1[index] = val;
  } else if (id==8){
    params.r_2[index] = val;
  } else if (id == 9){
    params.r_3[index] = val;
  } else if (id == 10){
    params.r_4[index] = val;
  } else if (id == 11){
    params.r_5[index] = val;
  } else if (id == 12){
    params.deltau_max[index] = val;
  } else if (id == 13){
    params.u_prev[index] = val;
  }
  params.A[0] = 1.0;
}


void set_ref(float x, float y){
  setup_indexing();
  for (int i = 0; i < HORIZON+2; ++i){
    params.r[0][i] = x;
    params.r[1][i] = y;
    params.r[2][i] = (float)0;
  }
}

float* get_prediction_horizon() {
  static float vars_x[NUM_VARS_COLS_X*NUM_VARS_ROWS_X];
  for (int col = 1; col < NUM_VARS_COLS_X+1; col++){
    for (int row = 0; row < NUM_VARS_ROWS_X; row++){
      vars_x[col*NUM_VARS_ROWS_X + row] = (float)vars.x[col][row];
    }
  }
  return vars_x;
}

float* get_control_horizon() {
  static float vars_u[NUM_VARS_COLS_U*NUM_VARS_ROWS_U];
  for (int col = 0; col < NUM_VARS_COLS_U; col++){
    for (int row = 0; row < NUM_VARS_ROWS_U; row++){
      vars_u[col*NUM_VARS_ROWS_U + row] = (float)vars.u[col][row];
    }
  }
  return vars_u;
}
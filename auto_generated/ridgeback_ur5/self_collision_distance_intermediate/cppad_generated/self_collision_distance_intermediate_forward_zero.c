#include <math.h>
#include <stdio.h>

typedef struct Array {
    void* data;
    unsigned long size;
    int sparse;
    const unsigned long* idx;
    unsigned long nnz;
} Array;

struct LangCAtomicFun {
    void* libModel;
    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);
    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

void self_collision_distance_intermediate_forward_zero(double const *const * in,
                                                       double*const * out,
                                                       struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[20];

   v[0] = sin(x[2]);
   v[1] = cos(x[3]);
   v[2] = sin(x[3]);
   v[3] = -1 * v[1] + -1.22464679914735e-16 * v[2];
   v[4] = cos(x[2]);
   v[5] = 1.22464679914735e-16 * v[1] + -1 * v[2];
   v[6] = v[0] * v[3] + v[4] * v[5];
   v[7] = cos(x[4]);
   v[2] = 0 - v[2];
   v[8] = -1 * v[2] + -1.22464679914735e-16 * v[1];
   v[2] = 1.22464679914735e-16 * v[2] + -1 * v[1];
   v[1] = v[0] * v[8] + v[4] * v[2];
   v[9] = sin(x[4]);
   v[10] = -2.05103489747671e-10 * v[9];
   v[11] = v[6] * v[7] + v[1] * v[10];
   v[12] = 0 - v[0];
   v[5] = v[4] * v[3] + v[12] * v[5];
   v[2] = v[4] * v[8] + v[12] * v[2];
   v[10] = v[5] * v[7] + v[2] * v[10];
   v[8] = cos(x[5]);
   v[3] = 0 - v[9];
   v[13] = -2.05103489747671e-10 * v[7];
   v[5] = v[5] * v[3] + v[2] * v[13];
   v[14] = sin(x[5]);
   v[15] = v[10] * v[8] + v[5] * v[14];
   v[16] = 0 - v[14];
   v[13] = v[6] * v[3] + v[1] * v[13];
   v[3] = v[11] * v[16] + v[13] * v[8];
   v[6] = v[9] * v[16] + v[7] * v[8];
   v[1] = -1 * v[1];
   v[2] = -1 * v[2];
   v[7] = v[9] * v[8] + v[7] * v[14];
   if( v[15] > v[3] ) {
      v[17] = v[6] - v[1];
   } else {
      v[17] = v[2] - v[7];
   }
   if( v[15] > v[3] ) {
      v[18] = 1.0000000002051 + v[15] - v[3];
   } else {
      v[18] = 1.0000000002051 + v[3] - v[15];
   }
   v[19] = 0.5 / sqrt(v[18]);
   v[17] = v[17] * v[19];
   if( v[15] > v[3] ) {
      v[7] = v[2] + v[7];
   } else {
      v[7] = v[6] + v[1];
   }
   v[7] = v[7] * v[19];
   v[13] = v[11] * v[8] + v[13] * v[14] + v[10] * v[16] + v[5] * v[8];
   if( v[15] > v[3] ) {
      v[16] = v[18];
   } else {
      v[16] = v[13];
   }
   v[16] = v[16] * v[19];
   v[14] = v[7] * x[12] - v[16] * x[14];
   v[14] = v[14] + v[14];
   if( v[15] > v[3] ) {
      v[13] = v[13];
   } else {
      v[13] = v[18];
   }
   v[13] = v[13] * v[19];
   v[19] = v[13] * x[14] - v[7] * x[13];
   v[19] = v[19] + v[19];
   v[18] = v[16] * x[13] - v[13] * x[12];
   v[18] = v[18] + v[18];
   v[12] = v[0] - v[12];
   if( v[4] < 0 - v[4] ) {
      v[0] = 2. - v[4] - v[4];
   } else {
      v[0] = 2. + v[4] + v[4];
   }
   if( v[4] < 0 - v[4] ) {
      v[3] = v[12];
   } else {
      v[3] = v[0];
   }
   v[15] = 0.5 / sqrt(v[0]);
   v[3] = v[3] * v[15];
   if( v[4] < 0 - v[4] ) {
      v[0] = v[0];
   } else {
      v[0] = v[12];
   }
   v[0] = v[0] * v[15];
   v[15] = v[0] * x[9];
   v[15] = v[15] + v[15];
   v[12] = 0 - v[0] * x[10];
   v[12] = v[12] + v[12];
   v[11] = -0.425 * v[11] + x[1] + v[17] * v[14] + x[13] + v[7] * v[19] - v[16] * v[18] - v[3] * v[15] - x[10] - v[0] * v[12] - x[1];
   v[12] = -0.425 * v[10] + x[0] + v[17] * v[19] + x[12] + v[13] * v[18] + v[0] * v[15] - v[7] * v[14] - v[3] * v[12] - x[9] - x[0];
   v[18] = 0.369159 + -0.425 * v[9] + v[17] * v[18] + x[14] + v[16] * v[14] - v[13] * v[19] - x[11];
   y[0] = x[15] * sqrt(v[11] * v[11] + v[12] * v[12] + v[18] * v[18]) - 0.1;
}


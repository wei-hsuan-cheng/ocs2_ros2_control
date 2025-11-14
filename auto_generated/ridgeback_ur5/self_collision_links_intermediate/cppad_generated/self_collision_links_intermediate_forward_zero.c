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

void self_collision_links_intermediate_forward_zero(double const *const * in,
                                                    double*const * out,
                                                    struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[17];

   v[0] = cos(x[2]);
   v[1] = sin(x[2]);
   v[2] = 0 - v[1];
   v[3] = v[1] - v[2];
   if( v[0] < 0 - v[0] ) {
      v[4] = 2. - v[0] - v[0];
   } else {
      v[4] = 2. + v[0] + v[0];
   }
   if( v[0] < 0 - v[0] ) {
      v[5] = v[3];
   } else {
      v[5] = v[4];
   }
   v[6] = 0.5 / sqrt(v[4]);
   v[5] = v[5] * v[6];
   if( v[0] < 0 - v[0] ) {
      v[4] = v[4];
   } else {
      v[4] = v[3];
   }
   v[4] = 0 - v[4] * v[6];
   v[6] = 0 - x[1];
   v[3] = 0 - v[4] * v[6];
   v[3] = v[3] + v[3];
   v[7] = 0 - x[0];
   v[8] = 0 - v[4] * x[10];
   v[8] = v[8] + v[8];
   v[9] = v[4] * v[7];
   v[9] = v[9] + v[9];
   v[10] = v[4] * x[9];
   v[10] = v[10] + v[10];
   y[0] = v[5] * v[3] + v[7] + v[5] * v[8] + x[9] - v[4] * v[9] - v[4] * v[10];
   y[1] = v[5] * v[9] + v[6] + v[4] * v[3] + v[5] * v[10] + x[10] + v[4] * v[8];
   v[10] = cos(x[3]);
   v[9] = sin(x[3]);
   v[8] = -1 * v[10] + -1.22464679914735e-16 * v[9];
   v[3] = 1.22464679914735e-16 * v[10] + -1 * v[9];
   v[6] = v[0] * v[8] + v[2] * v[3];
   v[4] = cos(x[4]);
   v[9] = 0 - v[9];
   v[5] = -1 * v[9] + -1.22464679914735e-16 * v[10];
   v[9] = 1.22464679914735e-16 * v[9] + -1 * v[10];
   v[2] = v[0] * v[5] + v[2] * v[9];
   v[10] = sin(x[4]);
   v[7] = -2.05103489747671e-10 * v[10];
   v[11] = v[6] * v[4] + v[2] * v[7];
   v[12] = cos(x[5]);
   v[13] = 0 - v[10];
   v[14] = -2.05103489747671e-10 * v[4];
   v[6] = v[6] * v[13] + v[2] * v[14];
   v[15] = sin(x[5]);
   v[16] = v[11] * v[12] + v[6] * v[15];
   v[3] = v[1] * v[8] + v[0] * v[3];
   v[9] = v[1] * v[5] + v[0] * v[9];
   v[7] = v[3] * v[4] + v[9] * v[7];
   v[5] = 0 - v[15];
   v[3] = v[3] * v[13] + v[9] * v[14];
   v[14] = v[7] * v[5] + v[3] * v[12];
   v[13] = v[10] * v[5] + v[4] * v[12];
   v[9] = -1 * v[9];
   v[2] = -1 * v[2];
   v[4] = v[10] * v[12] + v[4] * v[15];
   if( v[16] > v[14] ) {
      v[1] = v[13] - v[9];
   } else {
      v[1] = v[2] - v[4];
   }
   if( v[16] > v[14] ) {
      v[0] = 1.0000000002051 + v[16] - v[14];
   } else {
      v[0] = 1.0000000002051 + v[14] - v[16];
   }
   v[8] = 0.5 / sqrt(v[0]);
   v[1] = v[1] * v[8];
   v[3] = v[7] * v[12] + v[3] * v[15] + v[11] * v[5] + v[6] * v[12];
   if( v[16] > v[14] ) {
      v[5] = v[3];
   } else {
      v[5] = v[0];
   }
   v[5] = 0 - v[5] * v[8];
   v[10] = -0.369159 - -0.425 * v[10];
   if( v[16] > v[14] ) {
      v[4] = v[2] + v[4];
   } else {
      v[4] = v[13] + v[9];
   }
   v[4] = 0 - v[4] * v[8];
   v[7] = 0 - -0.425 * v[7] - x[1];
   v[2] = v[5] * v[10] - v[4] * v[7];
   v[2] = v[2] + v[2];
   v[11] = 0 - -0.425 * v[11] - x[0];
   if( v[16] > v[14] ) {
      v[3] = v[0];
   } else {
      v[3] = v[3];
   }
   v[3] = 0 - v[3] * v[8];
   v[8] = v[3] * v[7] - v[5] * v[11];
   v[8] = v[8] + v[8];
   v[0] = v[5] * x[14] - v[4] * x[13];
   v[0] = v[0] + v[0];
   v[14] = v[3] * x[13] - v[5] * x[12];
   v[14] = v[14] + v[14];
   v[16] = v[4] * v[11] - v[3] * v[10];
   v[16] = v[16] + v[16];
   v[9] = v[4] * x[12] - v[3] * x[14];
   v[9] = v[9] + v[9];
   y[3] = v[1] * v[2] + v[11] + v[5] * v[8] + v[1] * v[0] + x[12] + v[5] * v[14] - v[4] * v[16] - v[4] * v[9];
   y[4] = v[1] * v[16] + v[7] + v[4] * v[2] + v[1] * v[9] + x[13] + v[4] * v[0] - v[3] * v[8] - v[3] * v[14];
   y[5] = v[1] * v[8] + v[10] + v[3] * v[16] + v[1] * v[14] + x[14] + v[3] * v[9] - v[5] * v[2] - v[5] * v[0];
   if( x[15] > 0 ) {
      y[6] = 1;
   } else {
      y[6] = -1;
   }
   // dependent variables without operations
   y[2] = x[11];
}


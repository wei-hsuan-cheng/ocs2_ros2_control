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

void end_effector_kinematics_position_forward_zero(double const *const * in,
                                                   double*const * out,
                                                   struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[25];

   v[0] = cos(x[2]);
   v[1] = cos(x[3]);
   v[2] = sin(x[3]);
   v[3] = -1 * v[1] + -1.22464679914735e-16 * v[2];
   v[4] = sin(x[2]);
   v[5] = 0 - v[4];
   v[6] = 1.22464679914735e-16 * v[1] + -1 * v[2];
   v[7] = v[0] * v[3] + v[5] * v[6];
   v[8] = cos(x[4]);
   v[2] = 0 - v[2];
   v[9] = -1 * v[2] + -1.22464679914735e-16 * v[1];
   v[2] = 1.22464679914735e-16 * v[2] + -1 * v[1];
   v[5] = v[0] * v[9] + v[5] * v[2];
   v[1] = sin(x[4]);
   v[10] = -2.05103489747671e-10 * v[1];
   v[11] = v[7] * v[8] + v[5] * v[10];
   v[12] = -1 * v[5];
   v[13] = cos(x[5]);
   v[14] = 0 - v[1];
   v[15] = -2.05103489747671e-10 * v[8];
   v[5] = v[7] * v[14] + v[5] * v[15];
   v[7] = sin(x[5]);
   v[16] = v[11] * v[13] + v[5] * v[7];
   v[17] = sin(x[6]);
   v[18] = 0 - v[17];
   v[19] = 0 - v[7];
   v[5] = v[11] * v[19] + v[5] * v[13];
   v[20] = cos(x[6]);
   v[21] = v[16] * v[18] + v[5] * v[20];
   v[22] = cos(x[7]);
   v[23] = -2.05103489747671e-10 * v[22];
   v[24] = 0 - sin(x[7]);
   y[0] = -0.425 * v[11] + x[0] + 0.10915 * v[12] + -0.39225 * v[16] + -1.94130395089761e-11 * v[12] + -0.09465 * v[21] + -1.68800121668118e-11 * (-1 * v[21] + -2.05103489747671e-10 * v[12]) + 0.0823 * (v[12] * v[22] + v[21] * v[23] + (v[16] * v[20] + v[5] * v[17]) * v[24]);
   v[6] = v[4] * v[3] + v[0] * v[6];
   v[2] = v[4] * v[9] + v[0] * v[2];
   v[10] = v[6] * v[8] + v[2] * v[10];
   v[9] = -1 * v[2];
   v[2] = v[6] * v[14] + v[2] * v[15];
   v[6] = v[10] * v[13] + v[2] * v[7];
   v[2] = v[10] * v[19] + v[2] * v[13];
   v[15] = v[6] * v[18] + v[2] * v[20];
   y[1] = -0.425 * v[10] + x[1] + 0.10915 * v[9] + -0.39225 * v[6] + -1.94130395089761e-11 * v[9] + -0.09465 * v[15] + -1.68800121668118e-11 * (-1 * v[15] + -2.05103489747671e-10 * v[9]) + 0.0823 * (v[9] * v[22] + v[15] * v[23] + (v[6] * v[20] + v[2] * v[17]) * v[24]);
   v[7] = v[1] * v[13] + v[8] * v[7];
   v[19] = v[1] * v[19] + v[8] * v[13];
   v[18] = v[7] * v[18] + v[19] * v[20];
   y[2] = 0.369158999977613 + -0.425 * v[1] + -0.39225 * v[7] + -0.09465 * v[18] + -1.68800121668118e-11 * (4.2067441506673e-20 + -1 * v[18]) + 0.0823 * (-2.05103489747671e-10 * v[22] + v[18] * v[23] + (v[7] * v[20] + v[19] * v[17]) * v[24]);
}


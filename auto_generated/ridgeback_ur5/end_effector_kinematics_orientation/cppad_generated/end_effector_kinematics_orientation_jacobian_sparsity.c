void end_effector_kinematics_orientation_jacobian_sparsity(unsigned long const** row,
                                                           unsigned long const** col,
                                                           unsigned long* nnz) {
   static unsigned long const rows[21] = {0,0,0,0,0,0,0,1,1,1,1,1,1,1,2,2,2,2,2,2,2};
   static unsigned long const cols[21] = {2,3,4,5,6,7,8,2,3,4,5,6,7,8,2,3,4,5,6,7,8};
   *row = rows;
   *col = cols;
   *nnz = 21;
}

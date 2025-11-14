void end_effector_kinematics_position_jacobian_sparsity(unsigned long const** row,
                                                        unsigned long const** col,
                                                        unsigned long* nnz) {
   static unsigned long const rows[18] = {0,0,0,0,0,0,0,1,1,1,1,1,1,1,2,2,2,2};
   static unsigned long const cols[18] = {0,2,3,4,5,6,7,1,2,3,4,5,6,7,4,5,6,7};
   *row = rows;
   *col = cols;
   *nnz = 18;
}

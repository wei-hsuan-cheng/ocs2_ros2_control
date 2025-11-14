void self_collision_links_intermediate_jacobian_sparsity(unsigned long const** row,
                                                         unsigned long const** col,
                                                         unsigned long* nnz) {
   static unsigned long const rows[27] = {0,0,0,1,1,1,2,2,2,3,3,3,3,3,3,4,4,4,4,4,4,5,5,5,5,5,5};
   static unsigned long const cols[27] = {0,1,2,0,1,2,0,1,2,0,1,2,3,4,5,0,1,2,3,4,5,0,1,2,3,4,5};
   *row = rows;
   *col = cols;
   *nnz = 27;
}

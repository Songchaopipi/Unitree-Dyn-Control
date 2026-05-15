void g1_kinodynamics_flow_q_info(const char** baseName,
                                 unsigned long* m,
                                 unsigned long* n,
                                 unsigned int* indCount,
                                 unsigned int* depCount) {
   *baseName = "double  d";
   *m = 70;
   *n = 111;
   *depCount = 1; // number of dependent array variables
   *indCount = 1; // number of independent array variables
}


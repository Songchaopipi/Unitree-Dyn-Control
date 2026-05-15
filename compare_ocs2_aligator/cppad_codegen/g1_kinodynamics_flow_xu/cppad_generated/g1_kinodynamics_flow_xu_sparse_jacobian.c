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

void g1_kinodynamics_flow_xu_sparse_jacobian__1(double const *const * in, double*const * out, struct LangCAtomicFun atomicFun, double* v, double* array, double* sarray, unsigned long* idx);
void g1_kinodynamics_flow_xu_sparse_jacobian__2(double const *const * in, double*const * out, struct LangCAtomicFun atomicFun, double* v, double* array, double* sarray, unsigned long* idx);
void g1_kinodynamics_flow_xu_sparse_jacobian__3(double const *const * in, double*const * out, struct LangCAtomicFun atomicFun, double* v, double* array, double* sarray, unsigned long* idx);

void g1_kinodynamics_flow_xu_sparse_jacobian(double const *const * in,
                                             double*const * out,
                                             struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[7138];
   double array[0];
   double sarray[0];
   unsigned long idx[0];

   g1_kinodynamics_flow_xu_sparse_jacobian__1(in, out, atomicFun, v, array, sarray, idx);
   g1_kinodynamics_flow_xu_sparse_jacobian__2(in, out, atomicFun, v, array, sarray, idx);
   g1_kinodynamics_flow_xu_sparse_jacobian__3(in, out, atomicFun, v, array, sarray, idx);
   // variable duplicates: 18
   jac[114] = jac[108];
   jac[115] = jac[109];
   jac[116] = jac[110];
   jac[225] = jac[219];
   jac[226] = jac[220];
   jac[227] = jac[221];
   jac[336] = jac[330];
   jac[337] = jac[331];
   jac[338] = jac[332];
   jac[447] = jac[441];
   jac[448] = jac[442];
   jac[449] = jac[443];
   jac[558] = jac[552];
   jac[559] = jac[553];
   jac[560] = jac[554];
   jac[669] = jac[663];
   jac[670] = jac[664];
   jac[671] = jac[665];
   // dependent variables without operations
   jac[0] = 1;
   jac[1] = 1;
   jac[2] = 1;
   jac[3] = 1;
   jac[4] = 1;
   jac[5] = 1;
   jac[6] = 1;
   jac[7] = 1;
   jac[8] = 1;
   jac[9] = 1;
   jac[10] = 1;
   jac[11] = 1;
   jac[12] = 1;
   jac[13] = 1;
   jac[14] = 1;
   jac[15] = 1;
   jac[16] = 1;
   jac[17] = 1;
   jac[18] = 1;
   jac[19] = 1;
   jac[20] = 1;
   jac[21] = 1;
   jac[22] = 1;
   jac[23] = 1;
   jac[24] = 1;
   jac[25] = 1;
   jac[26] = 1;
   jac[27] = 1;
   jac[28] = 1;
   jac[29] = 1;
   jac[30] = 1;
   jac[31] = 1;
   jac[32] = 1;
   jac[33] = 1;
   jac[34] = 1;
   jac[701] = 1;
   jac[702] = 1;
   jac[703] = 1;
   jac[704] = 1;
   jac[705] = 1;
   jac[706] = 1;
   jac[707] = 1;
   jac[708] = 1;
   jac[709] = 1;
   jac[710] = 1;
   jac[711] = 1;
   jac[712] = 1;
   jac[713] = 1;
   jac[714] = 1;
   jac[715] = 1;
   jac[716] = 1;
   jac[717] = 1;
   jac[718] = 1;
   jac[719] = 1;
   jac[720] = 1;
   jac[721] = 1;
   jac[722] = 1;
   jac[723] = 1;
   jac[724] = 1;
   jac[725] = 1;
   jac[726] = 1;
   jac[727] = 1;
   jac[728] = 1;
   jac[729] = 1;
}


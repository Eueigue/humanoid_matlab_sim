/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

int cineq7_max_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq7_max_v_func_alloc_mem(void);
int cineq7_max_v_func_init_mem(int mem);
void cineq7_max_v_func_free_mem(int mem);
int cineq7_max_v_func_checkout(void);
void cineq7_max_v_func_release(int mem);
void cineq7_max_v_func_incref(void);
void cineq7_max_v_func_decref(void);
casadi_int cineq7_max_v_func_n_out(void);
casadi_int cineq7_max_v_func_n_in(void);
casadi_real cineq7_max_v_func_default_in(casadi_int i);
const char* cineq7_max_v_func_name_in(casadi_int i);
const char* cineq7_max_v_func_name_out(casadi_int i);
const casadi_int* cineq7_max_v_func_sparsity_in(casadi_int i);
const casadi_int* cineq7_max_v_func_sparsity_out(casadi_int i);
int cineq7_max_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#ifdef __cplusplus
} /* extern "C" */
#endif

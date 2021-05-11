/*
mpcc : A fast customized optimization solver.

Copyright (C) 2013-2021 EMBOTECH AG [info@embotech.com]. All rights reserved.


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/

/* Generated by FORCESPRO v4.2.1 on Sunday, May 9, 2021 at 5:05:34 AM */
#ifndef mpcc_H
#define mpcc_H

#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif


#ifndef SOLVER_STANDARD_TYPES
#define SOLVER_STANDARD_TYPES

typedef signed char solver_int8_signed;
typedef unsigned char solver_int8_unsigned;
typedef char solver_int8_default;
typedef signed short int solver_int16_signed;
typedef unsigned short int solver_int16_unsigned;
typedef short int solver_int16_default;
typedef signed int solver_int32_signed;
typedef unsigned int solver_int32_unsigned;
typedef int solver_int32_default;
typedef signed long long int solver_int64_signed;
typedef unsigned long long int solver_int64_unsigned;
typedef long long int solver_int64_default;

#endif


/* DATA TYPE ------------------------------------------------------------*/
typedef double mpcc_float;


typedef double mpccinterface_float;

/* SOLVER SETTINGS ------------------------------------------------------*/

/* MISRA-C compliance */
#ifndef MISRA_C_mpcc
#define MISRA_C_mpcc (0)
#endif

/* restrict code */
#ifndef RESTRICT_CODE_mpcc
#define RESTRICT_CODE_mpcc (0)
#endif

/* print level */
#ifndef SET_PRINTLEVEL_mpcc
#define SET_PRINTLEVEL_mpcc    (1)
#endif

/* timing */
#ifndef SET_TIMING_mpcc
#define SET_TIMING_mpcc    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define SET_MAXIT_mpcc         (200)	

/* scaling factor of line search (affine direction) */
#define SET_LS_SCALE_AFF_mpcc  (mpcc_float)(0.9)      

/* scaling factor of line search (combined direction) */
#define SET_LS_SCALE_mpcc      (mpcc_float)(0.95)  

/* minimum required step size in each iteration */
#define SET_LS_MINSTEP_mpcc    (mpcc_float)(1E-08)

/* maximum step size (combined direction) */
#define SET_LS_MAXSTEP_mpcc    (mpcc_float)(0.995)

/* desired relative duality gap */
#define SET_ACC_RDGAP_mpcc     (mpcc_float)(0.0001)

/* desired maximum residual on equality constraints */
#define SET_ACC_RESEQ_mpcc     (mpcc_float)(1E-06)

/* desired maximum residual on inequality constraints */
#define SET_ACC_RESINEQ_mpcc   (mpcc_float)(1E-06)

/* desired maximum violation of complementarity */
#define SET_ACC_KKTCOMPL_mpcc  (mpcc_float)(1E-06)

/* desired maximum violation of stationarity (only checked if value is > 0) */
#define SET_ACC_KKTSTAT_mpcc  (mpcc_float)(-1)

/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define OPTIMAL_mpcc      (1)

/* maximum number of iterations has been reached */
#define MAXITREACHED_mpcc (0)

/* solver has stopped due to a timeout */
#define TIMEOUT_mpcc   (2)

/* no progress in line search possible */
#define NOPROGRESS_mpcc   (-7)

/* fatal internal error - nans occurring */
#define NAN_mpcc  (-10)

/* invalid values in parameters */
#define PARAM_VALUE_ERROR_mpcc   (-11)

/* too small timeout given */
#define INVALID_TIMEOUT_mpcc   (-12)

/* licensing error - solver not valid on this machine */
#define LICENSE_ERROR_mpcc  (-100)


/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct mpcc_params
{
    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H01[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H02[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H03[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H04[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H05[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H06[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H07[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H08[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H09[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H10[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H11[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H12[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H13[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H14[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H15[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H16[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H17[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H18[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H19[169];

    /* matrix of size [13 x 13] (column major format) */
    mpcc_float H20[169];

    /* vector of size 13 */
    mpcc_float f01[13];

    /* vector of size 13 */
    mpcc_float f02[13];

    /* vector of size 13 */
    mpcc_float f03[13];

    /* vector of size 13 */
    mpcc_float f04[13];

    /* vector of size 13 */
    mpcc_float f05[13];

    /* vector of size 13 */
    mpcc_float f06[13];

    /* vector of size 13 */
    mpcc_float f07[13];

    /* vector of size 13 */
    mpcc_float f08[13];

    /* vector of size 13 */
    mpcc_float f09[13];

    /* vector of size 13 */
    mpcc_float f10[13];

    /* vector of size 13 */
    mpcc_float f11[13];

    /* vector of size 13 */
    mpcc_float f12[13];

    /* vector of size 13 */
    mpcc_float f13[13];

    /* vector of size 13 */
    mpcc_float f14[13];

    /* vector of size 13 */
    mpcc_float f15[13];

    /* vector of size 13 */
    mpcc_float f16[13];

    /* vector of size 13 */
    mpcc_float f17[13];

    /* vector of size 13 */
    mpcc_float f18[13];

    /* vector of size 13 */
    mpcc_float f19[13];

    /* vector of size 13 */
    mpcc_float f20[13];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A01[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A02[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A03[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A04[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A05[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A06[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A07[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A08[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A09[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A10[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A11[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A12[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A13[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A14[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A15[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A16[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A17[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A18[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A19[26];

    /* matrix of size [2 x 13] (column major format) */
    mpcc_float A20[26];

    /* vector of size 2 */
    mpcc_float b01[2];

    /* vector of size 2 */
    mpcc_float b02[2];

    /* vector of size 2 */
    mpcc_float b03[2];

    /* vector of size 2 */
    mpcc_float b04[2];

    /* vector of size 2 */
    mpcc_float b05[2];

    /* vector of size 2 */
    mpcc_float b06[2];

    /* vector of size 2 */
    mpcc_float b07[2];

    /* vector of size 2 */
    mpcc_float b08[2];

    /* vector of size 2 */
    mpcc_float b09[2];

    /* vector of size 2 */
    mpcc_float b10[2];

    /* vector of size 2 */
    mpcc_float b11[2];

    /* vector of size 2 */
    mpcc_float b12[2];

    /* vector of size 2 */
    mpcc_float b13[2];

    /* vector of size 2 */
    mpcc_float b14[2];

    /* vector of size 2 */
    mpcc_float b15[2];

    /* vector of size 2 */
    mpcc_float b16[2];

    /* vector of size 2 */
    mpcc_float b17[2];

    /* vector of size 2 */
    mpcc_float b18[2];

    /* vector of size 2 */
    mpcc_float b19[2];

    /* vector of size 2 */
    mpcc_float b20[2];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C01[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C02[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C03[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C04[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C05[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C06[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C07[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C08[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C09[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C10[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C11[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C12[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C13[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C14[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C15[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C16[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C17[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C18[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float C19[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D01[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D02[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D03[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D04[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D05[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D06[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D07[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D08[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D09[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D10[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D11[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D12[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D13[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D14[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D15[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D16[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D17[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D18[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D19[130];

    /* matrix of size [10 x 13] (column major format) */
    mpcc_float D20[130];

    /* vector of size 10 */
    mpcc_float c02[10];

    /* vector of size 10 */
    mpcc_float c03[10];

    /* vector of size 10 */
    mpcc_float c04[10];

    /* vector of size 10 */
    mpcc_float c05[10];

    /* vector of size 10 */
    mpcc_float c06[10];

    /* vector of size 10 */
    mpcc_float c07[10];

    /* vector of size 10 */
    mpcc_float c08[10];

    /* vector of size 10 */
    mpcc_float c09[10];

    /* vector of size 10 */
    mpcc_float c10[10];

    /* vector of size 10 */
    mpcc_float c11[10];

    /* vector of size 10 */
    mpcc_float c12[10];

    /* vector of size 10 */
    mpcc_float c13[10];

    /* vector of size 10 */
    mpcc_float c14[10];

    /* vector of size 10 */
    mpcc_float c15[10];

    /* vector of size 10 */
    mpcc_float c16[10];

    /* vector of size 10 */
    mpcc_float c17[10];

    /* vector of size 10 */
    mpcc_float c18[10];

    /* vector of size 10 */
    mpcc_float c19[10];

    /* vector of size 10 */
    mpcc_float c20[10];

    /* vector of size 10 */
    mpcc_float minusA_times_x0[10];

} mpcc_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct mpcc_output
{
    /* vector of size 260 */
    mpcc_float u0[260];

} mpcc_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct mpcc_info
{
    /* iteration number */
    solver_int32_default it;

	/* number of iterations needed to optimality (branch-and-bound) */
	solver_int32_default it2opt;
	
    /* inf-norm of equality constraint residuals */
    mpcc_float res_eq;
	
    /* inf-norm of inequality constraint residuals */
    mpcc_float res_ineq;

    /* primal objective */
    mpcc_float pobj;	
	
    /* dual objective */
    mpcc_float dobj;	

    /* duality gap := pobj - dobj */
    mpcc_float dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    mpcc_float rdgap;		

	/* infinity norm of gradient of Lagrangian*/
	mpcc_float gradient_lag_norm;

    /* duality measure */
    mpcc_float mu;

	/* duality measure (after affine step) */
    mpcc_float mu_aff;
	
    /* centering parameter */
    mpcc_float sigma;
	
    /* number of backtracking line search steps (affine direction) */
    solver_int32_default lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    solver_int32_default lsit_cc;
    
    /* step size (affine direction) */
    mpcc_float step_aff;
    
    /* step size (combined direction) */
    mpcc_float step_cc;    

	/* solvertime */
	mpcc_float solvetime;   

} mpcc_info;









/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* Time of Solver Generation: (UTC) Sunday, May 9, 2021 5:05:35 AM */
/* User License expires on: (UTC) Saturday, August 28, 2021 10:00:00 PM (approx.) (at the time of code generation) */
/* Solver Static License expires on: (UTC) Saturday, August 28, 2021 10:00:00 PM (approx.) */
/* Solver Generation Request Id: 123197e3-9d5c-41da-be16-c6b1be40f659 */
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif
extern solver_int32_default mpcc_solve(mpcc_params *params, mpcc_output *output, mpcc_info *info, FILE *fs);


#ifdef __cplusplus
}
#endif

#endif

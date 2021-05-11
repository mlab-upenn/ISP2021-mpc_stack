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


#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME mpcc_simulinkBlock

#include "simstruc.h"



/* include FORCESPRO functions and defs */
#include "../include/mpcc.h" 

/* SYSTEM INCLUDES FOR TIMING ------------------------------------------ */


#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

typedef mpccinterface_float mpccnmpc_float;





/*====================*
 * S-function methods *
 *====================*/
/* Function: mdlInitializeSizes =========================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{

    DECL_AND_INIT_DIMSINFO(inputDimsInfo);
    DECL_AND_INIT_DIMSINFO(outputDimsInfo);
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) 
	{
		return; /* Parameter mismatch will be reported by Simulink */
    }

	/* initialize size of continuous and discrete states to zero */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

	/* initialize input ports - there are 139 in total */
    if (!ssSetNumInputPorts(S, 139)) return;
    	
	/* Input Port 0 */
    ssSetInputPortMatrixDimensions(S,  0, 13, 13);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 13, 13);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 13, 13);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 13, 13);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 3, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/	
	/* Input Port 4 */
    ssSetInputPortMatrixDimensions(S,  4, 13, 13);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 4, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 4, 1); /*direct input signal access*/	
	/* Input Port 5 */
    ssSetInputPortMatrixDimensions(S,  5, 13, 13);
    ssSetInputPortDataType(S, 5, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 5, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 5, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 5, 1); /*direct input signal access*/	
	/* Input Port 6 */
    ssSetInputPortMatrixDimensions(S,  6, 13, 13);
    ssSetInputPortDataType(S, 6, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 6, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 6, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 6, 1); /*direct input signal access*/	
	/* Input Port 7 */
    ssSetInputPortMatrixDimensions(S,  7, 13, 13);
    ssSetInputPortDataType(S, 7, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 7, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 7, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 7, 1); /*direct input signal access*/	
	/* Input Port 8 */
    ssSetInputPortMatrixDimensions(S,  8, 13, 13);
    ssSetInputPortDataType(S, 8, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 8, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 8, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 8, 1); /*direct input signal access*/	
	/* Input Port 9 */
    ssSetInputPortMatrixDimensions(S,  9, 13, 13);
    ssSetInputPortDataType(S, 9, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 9, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 9, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 9, 1); /*direct input signal access*/	
	/* Input Port 10 */
    ssSetInputPortMatrixDimensions(S,  10, 13, 13);
    ssSetInputPortDataType(S, 10, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 10, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 10, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 10, 1); /*direct input signal access*/	
	/* Input Port 11 */
    ssSetInputPortMatrixDimensions(S,  11, 13, 13);
    ssSetInputPortDataType(S, 11, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 11, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 11, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 11, 1); /*direct input signal access*/	
	/* Input Port 12 */
    ssSetInputPortMatrixDimensions(S,  12, 13, 13);
    ssSetInputPortDataType(S, 12, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 12, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 12, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 12, 1); /*direct input signal access*/	
	/* Input Port 13 */
    ssSetInputPortMatrixDimensions(S,  13, 13, 13);
    ssSetInputPortDataType(S, 13, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 13, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 13, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 13, 1); /*direct input signal access*/	
	/* Input Port 14 */
    ssSetInputPortMatrixDimensions(S,  14, 13, 13);
    ssSetInputPortDataType(S, 14, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 14, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 14, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 14, 1); /*direct input signal access*/	
	/* Input Port 15 */
    ssSetInputPortMatrixDimensions(S,  15, 13, 13);
    ssSetInputPortDataType(S, 15, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 15, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 15, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 15, 1); /*direct input signal access*/	
	/* Input Port 16 */
    ssSetInputPortMatrixDimensions(S,  16, 13, 13);
    ssSetInputPortDataType(S, 16, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 16, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 16, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 16, 1); /*direct input signal access*/	
	/* Input Port 17 */
    ssSetInputPortMatrixDimensions(S,  17, 13, 13);
    ssSetInputPortDataType(S, 17, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 17, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 17, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 17, 1); /*direct input signal access*/	
	/* Input Port 18 */
    ssSetInputPortMatrixDimensions(S,  18, 13, 13);
    ssSetInputPortDataType(S, 18, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 18, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 18, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 18, 1); /*direct input signal access*/	
	/* Input Port 19 */
    ssSetInputPortMatrixDimensions(S,  19, 13, 13);
    ssSetInputPortDataType(S, 19, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 19, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 19, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 19, 1); /*direct input signal access*/	
	/* Input Port 20 */
    ssSetInputPortMatrixDimensions(S,  20, 13, 1);
    ssSetInputPortDataType(S, 20, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 20, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 20, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 20, 1); /*direct input signal access*/	
	/* Input Port 21 */
    ssSetInputPortMatrixDimensions(S,  21, 13, 1);
    ssSetInputPortDataType(S, 21, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 21, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 21, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 21, 1); /*direct input signal access*/	
	/* Input Port 22 */
    ssSetInputPortMatrixDimensions(S,  22, 13, 1);
    ssSetInputPortDataType(S, 22, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 22, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 22, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 22, 1); /*direct input signal access*/	
	/* Input Port 23 */
    ssSetInputPortMatrixDimensions(S,  23, 13, 1);
    ssSetInputPortDataType(S, 23, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 23, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 23, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 23, 1); /*direct input signal access*/	
	/* Input Port 24 */
    ssSetInputPortMatrixDimensions(S,  24, 13, 1);
    ssSetInputPortDataType(S, 24, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 24, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 24, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 24, 1); /*direct input signal access*/	
	/* Input Port 25 */
    ssSetInputPortMatrixDimensions(S,  25, 13, 1);
    ssSetInputPortDataType(S, 25, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 25, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 25, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 25, 1); /*direct input signal access*/	
	/* Input Port 26 */
    ssSetInputPortMatrixDimensions(S,  26, 13, 1);
    ssSetInputPortDataType(S, 26, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 26, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 26, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 26, 1); /*direct input signal access*/	
	/* Input Port 27 */
    ssSetInputPortMatrixDimensions(S,  27, 13, 1);
    ssSetInputPortDataType(S, 27, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 27, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 27, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 27, 1); /*direct input signal access*/	
	/* Input Port 28 */
    ssSetInputPortMatrixDimensions(S,  28, 13, 1);
    ssSetInputPortDataType(S, 28, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 28, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 28, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 28, 1); /*direct input signal access*/	
	/* Input Port 29 */
    ssSetInputPortMatrixDimensions(S,  29, 13, 1);
    ssSetInputPortDataType(S, 29, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 29, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 29, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 29, 1); /*direct input signal access*/	
	/* Input Port 30 */
    ssSetInputPortMatrixDimensions(S,  30, 13, 1);
    ssSetInputPortDataType(S, 30, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 30, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 30, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 30, 1); /*direct input signal access*/	
	/* Input Port 31 */
    ssSetInputPortMatrixDimensions(S,  31, 13, 1);
    ssSetInputPortDataType(S, 31, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 31, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 31, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 31, 1); /*direct input signal access*/	
	/* Input Port 32 */
    ssSetInputPortMatrixDimensions(S,  32, 13, 1);
    ssSetInputPortDataType(S, 32, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 32, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 32, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 32, 1); /*direct input signal access*/	
	/* Input Port 33 */
    ssSetInputPortMatrixDimensions(S,  33, 13, 1);
    ssSetInputPortDataType(S, 33, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 33, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 33, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 33, 1); /*direct input signal access*/	
	/* Input Port 34 */
    ssSetInputPortMatrixDimensions(S,  34, 13, 1);
    ssSetInputPortDataType(S, 34, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 34, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 34, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 34, 1); /*direct input signal access*/	
	/* Input Port 35 */
    ssSetInputPortMatrixDimensions(S,  35, 13, 1);
    ssSetInputPortDataType(S, 35, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 35, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 35, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 35, 1); /*direct input signal access*/	
	/* Input Port 36 */
    ssSetInputPortMatrixDimensions(S,  36, 13, 1);
    ssSetInputPortDataType(S, 36, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 36, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 36, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 36, 1); /*direct input signal access*/	
	/* Input Port 37 */
    ssSetInputPortMatrixDimensions(S,  37, 13, 1);
    ssSetInputPortDataType(S, 37, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 37, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 37, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 37, 1); /*direct input signal access*/	
	/* Input Port 38 */
    ssSetInputPortMatrixDimensions(S,  38, 13, 1);
    ssSetInputPortDataType(S, 38, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 38, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 38, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 38, 1); /*direct input signal access*/	
	/* Input Port 39 */
    ssSetInputPortMatrixDimensions(S,  39, 13, 1);
    ssSetInputPortDataType(S, 39, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 39, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 39, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 39, 1); /*direct input signal access*/	
	/* Input Port 40 */
    ssSetInputPortMatrixDimensions(S,  40, 2, 13);
    ssSetInputPortDataType(S, 40, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 40, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 40, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 40, 1); /*direct input signal access*/	
	/* Input Port 41 */
    ssSetInputPortMatrixDimensions(S,  41, 2, 13);
    ssSetInputPortDataType(S, 41, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 41, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 41, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 41, 1); /*direct input signal access*/	
	/* Input Port 42 */
    ssSetInputPortMatrixDimensions(S,  42, 2, 13);
    ssSetInputPortDataType(S, 42, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 42, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 42, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 42, 1); /*direct input signal access*/	
	/* Input Port 43 */
    ssSetInputPortMatrixDimensions(S,  43, 2, 13);
    ssSetInputPortDataType(S, 43, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 43, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 43, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 43, 1); /*direct input signal access*/	
	/* Input Port 44 */
    ssSetInputPortMatrixDimensions(S,  44, 2, 13);
    ssSetInputPortDataType(S, 44, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 44, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 44, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 44, 1); /*direct input signal access*/	
	/* Input Port 45 */
    ssSetInputPortMatrixDimensions(S,  45, 2, 13);
    ssSetInputPortDataType(S, 45, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 45, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 45, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 45, 1); /*direct input signal access*/	
	/* Input Port 46 */
    ssSetInputPortMatrixDimensions(S,  46, 2, 13);
    ssSetInputPortDataType(S, 46, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 46, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 46, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 46, 1); /*direct input signal access*/	
	/* Input Port 47 */
    ssSetInputPortMatrixDimensions(S,  47, 2, 13);
    ssSetInputPortDataType(S, 47, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 47, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 47, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 47, 1); /*direct input signal access*/	
	/* Input Port 48 */
    ssSetInputPortMatrixDimensions(S,  48, 2, 13);
    ssSetInputPortDataType(S, 48, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 48, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 48, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 48, 1); /*direct input signal access*/	
	/* Input Port 49 */
    ssSetInputPortMatrixDimensions(S,  49, 2, 13);
    ssSetInputPortDataType(S, 49, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 49, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 49, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 49, 1); /*direct input signal access*/	
	/* Input Port 50 */
    ssSetInputPortMatrixDimensions(S,  50, 2, 13);
    ssSetInputPortDataType(S, 50, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 50, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 50, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 50, 1); /*direct input signal access*/	
	/* Input Port 51 */
    ssSetInputPortMatrixDimensions(S,  51, 2, 13);
    ssSetInputPortDataType(S, 51, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 51, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 51, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 51, 1); /*direct input signal access*/	
	/* Input Port 52 */
    ssSetInputPortMatrixDimensions(S,  52, 2, 13);
    ssSetInputPortDataType(S, 52, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 52, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 52, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 52, 1); /*direct input signal access*/	
	/* Input Port 53 */
    ssSetInputPortMatrixDimensions(S,  53, 2, 13);
    ssSetInputPortDataType(S, 53, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 53, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 53, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 53, 1); /*direct input signal access*/	
	/* Input Port 54 */
    ssSetInputPortMatrixDimensions(S,  54, 2, 13);
    ssSetInputPortDataType(S, 54, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 54, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 54, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 54, 1); /*direct input signal access*/	
	/* Input Port 55 */
    ssSetInputPortMatrixDimensions(S,  55, 2, 13);
    ssSetInputPortDataType(S, 55, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 55, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 55, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 55, 1); /*direct input signal access*/	
	/* Input Port 56 */
    ssSetInputPortMatrixDimensions(S,  56, 2, 13);
    ssSetInputPortDataType(S, 56, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 56, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 56, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 56, 1); /*direct input signal access*/	
	/* Input Port 57 */
    ssSetInputPortMatrixDimensions(S,  57, 2, 13);
    ssSetInputPortDataType(S, 57, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 57, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 57, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 57, 1); /*direct input signal access*/	
	/* Input Port 58 */
    ssSetInputPortMatrixDimensions(S,  58, 2, 13);
    ssSetInputPortDataType(S, 58, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 58, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 58, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 58, 1); /*direct input signal access*/	
	/* Input Port 59 */
    ssSetInputPortMatrixDimensions(S,  59, 2, 13);
    ssSetInputPortDataType(S, 59, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 59, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 59, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 59, 1); /*direct input signal access*/	
	/* Input Port 60 */
    ssSetInputPortMatrixDimensions(S,  60, 2, 1);
    ssSetInputPortDataType(S, 60, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 60, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 60, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 60, 1); /*direct input signal access*/	
	/* Input Port 61 */
    ssSetInputPortMatrixDimensions(S,  61, 2, 1);
    ssSetInputPortDataType(S, 61, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 61, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 61, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 61, 1); /*direct input signal access*/	
	/* Input Port 62 */
    ssSetInputPortMatrixDimensions(S,  62, 2, 1);
    ssSetInputPortDataType(S, 62, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 62, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 62, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 62, 1); /*direct input signal access*/	
	/* Input Port 63 */
    ssSetInputPortMatrixDimensions(S,  63, 2, 1);
    ssSetInputPortDataType(S, 63, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 63, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 63, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 63, 1); /*direct input signal access*/	
	/* Input Port 64 */
    ssSetInputPortMatrixDimensions(S,  64, 2, 1);
    ssSetInputPortDataType(S, 64, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 64, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 64, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 64, 1); /*direct input signal access*/	
	/* Input Port 65 */
    ssSetInputPortMatrixDimensions(S,  65, 2, 1);
    ssSetInputPortDataType(S, 65, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 65, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 65, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 65, 1); /*direct input signal access*/	
	/* Input Port 66 */
    ssSetInputPortMatrixDimensions(S,  66, 2, 1);
    ssSetInputPortDataType(S, 66, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 66, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 66, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 66, 1); /*direct input signal access*/	
	/* Input Port 67 */
    ssSetInputPortMatrixDimensions(S,  67, 2, 1);
    ssSetInputPortDataType(S, 67, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 67, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 67, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 67, 1); /*direct input signal access*/	
	/* Input Port 68 */
    ssSetInputPortMatrixDimensions(S,  68, 2, 1);
    ssSetInputPortDataType(S, 68, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 68, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 68, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 68, 1); /*direct input signal access*/	
	/* Input Port 69 */
    ssSetInputPortMatrixDimensions(S,  69, 2, 1);
    ssSetInputPortDataType(S, 69, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 69, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 69, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 69, 1); /*direct input signal access*/	
	/* Input Port 70 */
    ssSetInputPortMatrixDimensions(S,  70, 2, 1);
    ssSetInputPortDataType(S, 70, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 70, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 70, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 70, 1); /*direct input signal access*/	
	/* Input Port 71 */
    ssSetInputPortMatrixDimensions(S,  71, 2, 1);
    ssSetInputPortDataType(S, 71, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 71, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 71, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 71, 1); /*direct input signal access*/	
	/* Input Port 72 */
    ssSetInputPortMatrixDimensions(S,  72, 2, 1);
    ssSetInputPortDataType(S, 72, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 72, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 72, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 72, 1); /*direct input signal access*/	
	/* Input Port 73 */
    ssSetInputPortMatrixDimensions(S,  73, 2, 1);
    ssSetInputPortDataType(S, 73, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 73, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 73, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 73, 1); /*direct input signal access*/	
	/* Input Port 74 */
    ssSetInputPortMatrixDimensions(S,  74, 2, 1);
    ssSetInputPortDataType(S, 74, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 74, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 74, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 74, 1); /*direct input signal access*/	
	/* Input Port 75 */
    ssSetInputPortMatrixDimensions(S,  75, 2, 1);
    ssSetInputPortDataType(S, 75, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 75, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 75, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 75, 1); /*direct input signal access*/	
	/* Input Port 76 */
    ssSetInputPortMatrixDimensions(S,  76, 2, 1);
    ssSetInputPortDataType(S, 76, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 76, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 76, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 76, 1); /*direct input signal access*/	
	/* Input Port 77 */
    ssSetInputPortMatrixDimensions(S,  77, 2, 1);
    ssSetInputPortDataType(S, 77, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 77, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 77, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 77, 1); /*direct input signal access*/	
	/* Input Port 78 */
    ssSetInputPortMatrixDimensions(S,  78, 2, 1);
    ssSetInputPortDataType(S, 78, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 78, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 78, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 78, 1); /*direct input signal access*/	
	/* Input Port 79 */
    ssSetInputPortMatrixDimensions(S,  79, 2, 1);
    ssSetInputPortDataType(S, 79, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 79, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 79, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 79, 1); /*direct input signal access*/	
	/* Input Port 80 */
    ssSetInputPortMatrixDimensions(S,  80, 10, 13);
    ssSetInputPortDataType(S, 80, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 80, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 80, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 80, 1); /*direct input signal access*/	
	/* Input Port 81 */
    ssSetInputPortMatrixDimensions(S,  81, 10, 13);
    ssSetInputPortDataType(S, 81, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 81, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 81, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 81, 1); /*direct input signal access*/	
	/* Input Port 82 */
    ssSetInputPortMatrixDimensions(S,  82, 10, 13);
    ssSetInputPortDataType(S, 82, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 82, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 82, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 82, 1); /*direct input signal access*/	
	/* Input Port 83 */
    ssSetInputPortMatrixDimensions(S,  83, 10, 13);
    ssSetInputPortDataType(S, 83, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 83, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 83, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 83, 1); /*direct input signal access*/	
	/* Input Port 84 */
    ssSetInputPortMatrixDimensions(S,  84, 10, 13);
    ssSetInputPortDataType(S, 84, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 84, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 84, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 84, 1); /*direct input signal access*/	
	/* Input Port 85 */
    ssSetInputPortMatrixDimensions(S,  85, 10, 13);
    ssSetInputPortDataType(S, 85, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 85, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 85, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 85, 1); /*direct input signal access*/	
	/* Input Port 86 */
    ssSetInputPortMatrixDimensions(S,  86, 10, 13);
    ssSetInputPortDataType(S, 86, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 86, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 86, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 86, 1); /*direct input signal access*/	
	/* Input Port 87 */
    ssSetInputPortMatrixDimensions(S,  87, 10, 13);
    ssSetInputPortDataType(S, 87, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 87, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 87, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 87, 1); /*direct input signal access*/	
	/* Input Port 88 */
    ssSetInputPortMatrixDimensions(S,  88, 10, 13);
    ssSetInputPortDataType(S, 88, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 88, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 88, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 88, 1); /*direct input signal access*/	
	/* Input Port 89 */
    ssSetInputPortMatrixDimensions(S,  89, 10, 13);
    ssSetInputPortDataType(S, 89, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 89, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 89, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 89, 1); /*direct input signal access*/	
	/* Input Port 90 */
    ssSetInputPortMatrixDimensions(S,  90, 10, 13);
    ssSetInputPortDataType(S, 90, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 90, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 90, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 90, 1); /*direct input signal access*/	
	/* Input Port 91 */
    ssSetInputPortMatrixDimensions(S,  91, 10, 13);
    ssSetInputPortDataType(S, 91, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 91, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 91, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 91, 1); /*direct input signal access*/	
	/* Input Port 92 */
    ssSetInputPortMatrixDimensions(S,  92, 10, 13);
    ssSetInputPortDataType(S, 92, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 92, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 92, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 92, 1); /*direct input signal access*/	
	/* Input Port 93 */
    ssSetInputPortMatrixDimensions(S,  93, 10, 13);
    ssSetInputPortDataType(S, 93, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 93, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 93, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 93, 1); /*direct input signal access*/	
	/* Input Port 94 */
    ssSetInputPortMatrixDimensions(S,  94, 10, 13);
    ssSetInputPortDataType(S, 94, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 94, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 94, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 94, 1); /*direct input signal access*/	
	/* Input Port 95 */
    ssSetInputPortMatrixDimensions(S,  95, 10, 13);
    ssSetInputPortDataType(S, 95, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 95, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 95, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 95, 1); /*direct input signal access*/	
	/* Input Port 96 */
    ssSetInputPortMatrixDimensions(S,  96, 10, 13);
    ssSetInputPortDataType(S, 96, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 96, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 96, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 96, 1); /*direct input signal access*/	
	/* Input Port 97 */
    ssSetInputPortMatrixDimensions(S,  97, 10, 13);
    ssSetInputPortDataType(S, 97, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 97, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 97, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 97, 1); /*direct input signal access*/	
	/* Input Port 98 */
    ssSetInputPortMatrixDimensions(S,  98, 10, 13);
    ssSetInputPortDataType(S, 98, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 98, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 98, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 98, 1); /*direct input signal access*/	
	/* Input Port 99 */
    ssSetInputPortMatrixDimensions(S,  99, 10, 13);
    ssSetInputPortDataType(S, 99, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 99, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 99, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 99, 1); /*direct input signal access*/	
	/* Input Port 100 */
    ssSetInputPortMatrixDimensions(S,  100, 10, 13);
    ssSetInputPortDataType(S, 100, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 100, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 100, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 100, 1); /*direct input signal access*/	
	/* Input Port 101 */
    ssSetInputPortMatrixDimensions(S,  101, 10, 13);
    ssSetInputPortDataType(S, 101, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 101, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 101, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 101, 1); /*direct input signal access*/	
	/* Input Port 102 */
    ssSetInputPortMatrixDimensions(S,  102, 10, 13);
    ssSetInputPortDataType(S, 102, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 102, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 102, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 102, 1); /*direct input signal access*/	
	/* Input Port 103 */
    ssSetInputPortMatrixDimensions(S,  103, 10, 13);
    ssSetInputPortDataType(S, 103, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 103, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 103, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 103, 1); /*direct input signal access*/	
	/* Input Port 104 */
    ssSetInputPortMatrixDimensions(S,  104, 10, 13);
    ssSetInputPortDataType(S, 104, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 104, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 104, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 104, 1); /*direct input signal access*/	
	/* Input Port 105 */
    ssSetInputPortMatrixDimensions(S,  105, 10, 13);
    ssSetInputPortDataType(S, 105, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 105, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 105, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 105, 1); /*direct input signal access*/	
	/* Input Port 106 */
    ssSetInputPortMatrixDimensions(S,  106, 10, 13);
    ssSetInputPortDataType(S, 106, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 106, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 106, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 106, 1); /*direct input signal access*/	
	/* Input Port 107 */
    ssSetInputPortMatrixDimensions(S,  107, 10, 13);
    ssSetInputPortDataType(S, 107, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 107, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 107, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 107, 1); /*direct input signal access*/	
	/* Input Port 108 */
    ssSetInputPortMatrixDimensions(S,  108, 10, 13);
    ssSetInputPortDataType(S, 108, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 108, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 108, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 108, 1); /*direct input signal access*/	
	/* Input Port 109 */
    ssSetInputPortMatrixDimensions(S,  109, 10, 13);
    ssSetInputPortDataType(S, 109, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 109, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 109, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 109, 1); /*direct input signal access*/	
	/* Input Port 110 */
    ssSetInputPortMatrixDimensions(S,  110, 10, 13);
    ssSetInputPortDataType(S, 110, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 110, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 110, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 110, 1); /*direct input signal access*/	
	/* Input Port 111 */
    ssSetInputPortMatrixDimensions(S,  111, 10, 13);
    ssSetInputPortDataType(S, 111, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 111, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 111, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 111, 1); /*direct input signal access*/	
	/* Input Port 112 */
    ssSetInputPortMatrixDimensions(S,  112, 10, 13);
    ssSetInputPortDataType(S, 112, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 112, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 112, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 112, 1); /*direct input signal access*/	
	/* Input Port 113 */
    ssSetInputPortMatrixDimensions(S,  113, 10, 13);
    ssSetInputPortDataType(S, 113, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 113, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 113, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 113, 1); /*direct input signal access*/	
	/* Input Port 114 */
    ssSetInputPortMatrixDimensions(S,  114, 10, 13);
    ssSetInputPortDataType(S, 114, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 114, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 114, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 114, 1); /*direct input signal access*/	
	/* Input Port 115 */
    ssSetInputPortMatrixDimensions(S,  115, 10, 13);
    ssSetInputPortDataType(S, 115, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 115, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 115, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 115, 1); /*direct input signal access*/	
	/* Input Port 116 */
    ssSetInputPortMatrixDimensions(S,  116, 10, 13);
    ssSetInputPortDataType(S, 116, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 116, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 116, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 116, 1); /*direct input signal access*/	
	/* Input Port 117 */
    ssSetInputPortMatrixDimensions(S,  117, 10, 13);
    ssSetInputPortDataType(S, 117, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 117, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 117, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 117, 1); /*direct input signal access*/	
	/* Input Port 118 */
    ssSetInputPortMatrixDimensions(S,  118, 10, 13);
    ssSetInputPortDataType(S, 118, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 118, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 118, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 118, 1); /*direct input signal access*/	
	/* Input Port 119 */
    ssSetInputPortMatrixDimensions(S,  119, 10, 1);
    ssSetInputPortDataType(S, 119, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 119, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 119, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 119, 1); /*direct input signal access*/	
	/* Input Port 120 */
    ssSetInputPortMatrixDimensions(S,  120, 10, 1);
    ssSetInputPortDataType(S, 120, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 120, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 120, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 120, 1); /*direct input signal access*/	
	/* Input Port 121 */
    ssSetInputPortMatrixDimensions(S,  121, 10, 1);
    ssSetInputPortDataType(S, 121, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 121, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 121, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 121, 1); /*direct input signal access*/	
	/* Input Port 122 */
    ssSetInputPortMatrixDimensions(S,  122, 10, 1);
    ssSetInputPortDataType(S, 122, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 122, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 122, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 122, 1); /*direct input signal access*/	
	/* Input Port 123 */
    ssSetInputPortMatrixDimensions(S,  123, 10, 1);
    ssSetInputPortDataType(S, 123, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 123, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 123, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 123, 1); /*direct input signal access*/	
	/* Input Port 124 */
    ssSetInputPortMatrixDimensions(S,  124, 10, 1);
    ssSetInputPortDataType(S, 124, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 124, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 124, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 124, 1); /*direct input signal access*/	
	/* Input Port 125 */
    ssSetInputPortMatrixDimensions(S,  125, 10, 1);
    ssSetInputPortDataType(S, 125, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 125, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 125, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 125, 1); /*direct input signal access*/	
	/* Input Port 126 */
    ssSetInputPortMatrixDimensions(S,  126, 10, 1);
    ssSetInputPortDataType(S, 126, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 126, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 126, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 126, 1); /*direct input signal access*/	
	/* Input Port 127 */
    ssSetInputPortMatrixDimensions(S,  127, 10, 1);
    ssSetInputPortDataType(S, 127, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 127, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 127, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 127, 1); /*direct input signal access*/	
	/* Input Port 128 */
    ssSetInputPortMatrixDimensions(S,  128, 10, 1);
    ssSetInputPortDataType(S, 128, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 128, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 128, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 128, 1); /*direct input signal access*/	
	/* Input Port 129 */
    ssSetInputPortMatrixDimensions(S,  129, 10, 1);
    ssSetInputPortDataType(S, 129, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 129, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 129, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 129, 1); /*direct input signal access*/	
	/* Input Port 130 */
    ssSetInputPortMatrixDimensions(S,  130, 10, 1);
    ssSetInputPortDataType(S, 130, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 130, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 130, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 130, 1); /*direct input signal access*/	
	/* Input Port 131 */
    ssSetInputPortMatrixDimensions(S,  131, 10, 1);
    ssSetInputPortDataType(S, 131, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 131, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 131, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 131, 1); /*direct input signal access*/	
	/* Input Port 132 */
    ssSetInputPortMatrixDimensions(S,  132, 10, 1);
    ssSetInputPortDataType(S, 132, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 132, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 132, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 132, 1); /*direct input signal access*/	
	/* Input Port 133 */
    ssSetInputPortMatrixDimensions(S,  133, 10, 1);
    ssSetInputPortDataType(S, 133, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 133, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 133, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 133, 1); /*direct input signal access*/	
	/* Input Port 134 */
    ssSetInputPortMatrixDimensions(S,  134, 10, 1);
    ssSetInputPortDataType(S, 134, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 134, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 134, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 134, 1); /*direct input signal access*/	
	/* Input Port 135 */
    ssSetInputPortMatrixDimensions(S,  135, 10, 1);
    ssSetInputPortDataType(S, 135, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 135, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 135, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 135, 1); /*direct input signal access*/	
	/* Input Port 136 */
    ssSetInputPortMatrixDimensions(S,  136, 10, 1);
    ssSetInputPortDataType(S, 136, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 136, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 136, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 136, 1); /*direct input signal access*/	
	/* Input Port 137 */
    ssSetInputPortMatrixDimensions(S,  137, 10, 1);
    ssSetInputPortDataType(S, 137, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 137, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 137, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 137, 1); /*direct input signal access*/	
	/* Input Port 138 */
    ssSetInputPortMatrixDimensions(S,  138, 10, 1);
    ssSetInputPortDataType(S, 138, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 138, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 138, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 138, 1); /*direct input signal access*/ 


	/* initialize output ports - there are 1 in total */
    if (!ssSetNumOutputPorts(S, 1)) return;    
		
	/* Output Port 0 */
    ssSetOutputPortMatrixDimensions(S,  0, 260, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */


	/* set sampling time */
    ssSetNumSampleTimes(S, 1);

	/* set internal memory of block */
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
	/* SS_OPTION_USE_TLC_WITH_ACCELERATOR removed */ 
	/* SS_OPTION_USE_TLC_WITH_ACCELERATOR removed */ 
    /* ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
		             SS_OPTION_WORKS_WITH_CODE_REUSE)); */
	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE );

	
}

#if defined(MATLAB_MEX_FILE)
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
static void mdlSetInputPortDimensionInfo(SimStruct        *S, 
                                         int_T            port,
                                         const DimsInfo_T *dimsInfo)
{
    if(!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif

#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
#if defined(MDL_SET_OUTPUT_PORT_DIMENSION_INFO)
static void mdlSetOutputPortDimensionInfo(SimStruct        *S, 
                                          int_T            port, 
                                          const DimsInfo_T *dimsInfo)
{
    if (!ssSetOutputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif
# define MDL_SET_INPUT_PORT_FRAME_DATA
static void mdlSetInputPortFrameData(SimStruct  *S, 
                                     int_T      port,
                                     Frame_T    frameData)
{
    ssSetInputPortFrameData(S, port, frameData);
}
/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_SET_INPUT_PORT_DATA_TYPE
static void mdlSetInputPortDataType(SimStruct *S, solver_int32_default port, DTypeId dType)
{
    ssSetInputPortDataType( S, 0, dType);
}
#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct *S, solver_int32_default port, DTypeId dType)
{
    ssSetOutputPortDataType(S, 0, dType);
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct *S)
{
    ssSetInputPortDataType( S, 0, SS_DOUBLE);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
}





/* Function: mdlOutputs =======================================================
 *
*/
static void mdlOutputs(SimStruct *S, int_T tid)
{
	solver_int32_default i, j, k;
	
	/* file pointer for printing */
	FILE *fp = NULL;

	/* Simulink data */
	const real_T *H01 = (const real_T*) ssGetInputPortSignal(S,0);
	const real_T *H02 = (const real_T*) ssGetInputPortSignal(S,1);
	const real_T *H03 = (const real_T*) ssGetInputPortSignal(S,2);
	const real_T *H04 = (const real_T*) ssGetInputPortSignal(S,3);
	const real_T *H05 = (const real_T*) ssGetInputPortSignal(S,4);
	const real_T *H06 = (const real_T*) ssGetInputPortSignal(S,5);
	const real_T *H07 = (const real_T*) ssGetInputPortSignal(S,6);
	const real_T *H08 = (const real_T*) ssGetInputPortSignal(S,7);
	const real_T *H09 = (const real_T*) ssGetInputPortSignal(S,8);
	const real_T *H10 = (const real_T*) ssGetInputPortSignal(S,9);
	const real_T *H11 = (const real_T*) ssGetInputPortSignal(S,10);
	const real_T *H12 = (const real_T*) ssGetInputPortSignal(S,11);
	const real_T *H13 = (const real_T*) ssGetInputPortSignal(S,12);
	const real_T *H14 = (const real_T*) ssGetInputPortSignal(S,13);
	const real_T *H15 = (const real_T*) ssGetInputPortSignal(S,14);
	const real_T *H16 = (const real_T*) ssGetInputPortSignal(S,15);
	const real_T *H17 = (const real_T*) ssGetInputPortSignal(S,16);
	const real_T *H18 = (const real_T*) ssGetInputPortSignal(S,17);
	const real_T *H19 = (const real_T*) ssGetInputPortSignal(S,18);
	const real_T *H20 = (const real_T*) ssGetInputPortSignal(S,19);
	const real_T *f01 = (const real_T*) ssGetInputPortSignal(S,20);
	const real_T *f02 = (const real_T*) ssGetInputPortSignal(S,21);
	const real_T *f03 = (const real_T*) ssGetInputPortSignal(S,22);
	const real_T *f04 = (const real_T*) ssGetInputPortSignal(S,23);
	const real_T *f05 = (const real_T*) ssGetInputPortSignal(S,24);
	const real_T *f06 = (const real_T*) ssGetInputPortSignal(S,25);
	const real_T *f07 = (const real_T*) ssGetInputPortSignal(S,26);
	const real_T *f08 = (const real_T*) ssGetInputPortSignal(S,27);
	const real_T *f09 = (const real_T*) ssGetInputPortSignal(S,28);
	const real_T *f10 = (const real_T*) ssGetInputPortSignal(S,29);
	const real_T *f11 = (const real_T*) ssGetInputPortSignal(S,30);
	const real_T *f12 = (const real_T*) ssGetInputPortSignal(S,31);
	const real_T *f13 = (const real_T*) ssGetInputPortSignal(S,32);
	const real_T *f14 = (const real_T*) ssGetInputPortSignal(S,33);
	const real_T *f15 = (const real_T*) ssGetInputPortSignal(S,34);
	const real_T *f16 = (const real_T*) ssGetInputPortSignal(S,35);
	const real_T *f17 = (const real_T*) ssGetInputPortSignal(S,36);
	const real_T *f18 = (const real_T*) ssGetInputPortSignal(S,37);
	const real_T *f19 = (const real_T*) ssGetInputPortSignal(S,38);
	const real_T *f20 = (const real_T*) ssGetInputPortSignal(S,39);
	const real_T *A01 = (const real_T*) ssGetInputPortSignal(S,40);
	const real_T *A02 = (const real_T*) ssGetInputPortSignal(S,41);
	const real_T *A03 = (const real_T*) ssGetInputPortSignal(S,42);
	const real_T *A04 = (const real_T*) ssGetInputPortSignal(S,43);
	const real_T *A05 = (const real_T*) ssGetInputPortSignal(S,44);
	const real_T *A06 = (const real_T*) ssGetInputPortSignal(S,45);
	const real_T *A07 = (const real_T*) ssGetInputPortSignal(S,46);
	const real_T *A08 = (const real_T*) ssGetInputPortSignal(S,47);
	const real_T *A09 = (const real_T*) ssGetInputPortSignal(S,48);
	const real_T *A10 = (const real_T*) ssGetInputPortSignal(S,49);
	const real_T *A11 = (const real_T*) ssGetInputPortSignal(S,50);
	const real_T *A12 = (const real_T*) ssGetInputPortSignal(S,51);
	const real_T *A13 = (const real_T*) ssGetInputPortSignal(S,52);
	const real_T *A14 = (const real_T*) ssGetInputPortSignal(S,53);
	const real_T *A15 = (const real_T*) ssGetInputPortSignal(S,54);
	const real_T *A16 = (const real_T*) ssGetInputPortSignal(S,55);
	const real_T *A17 = (const real_T*) ssGetInputPortSignal(S,56);
	const real_T *A18 = (const real_T*) ssGetInputPortSignal(S,57);
	const real_T *A19 = (const real_T*) ssGetInputPortSignal(S,58);
	const real_T *A20 = (const real_T*) ssGetInputPortSignal(S,59);
	const real_T *b01 = (const real_T*) ssGetInputPortSignal(S,60);
	const real_T *b02 = (const real_T*) ssGetInputPortSignal(S,61);
	const real_T *b03 = (const real_T*) ssGetInputPortSignal(S,62);
	const real_T *b04 = (const real_T*) ssGetInputPortSignal(S,63);
	const real_T *b05 = (const real_T*) ssGetInputPortSignal(S,64);
	const real_T *b06 = (const real_T*) ssGetInputPortSignal(S,65);
	const real_T *b07 = (const real_T*) ssGetInputPortSignal(S,66);
	const real_T *b08 = (const real_T*) ssGetInputPortSignal(S,67);
	const real_T *b09 = (const real_T*) ssGetInputPortSignal(S,68);
	const real_T *b10 = (const real_T*) ssGetInputPortSignal(S,69);
	const real_T *b11 = (const real_T*) ssGetInputPortSignal(S,70);
	const real_T *b12 = (const real_T*) ssGetInputPortSignal(S,71);
	const real_T *b13 = (const real_T*) ssGetInputPortSignal(S,72);
	const real_T *b14 = (const real_T*) ssGetInputPortSignal(S,73);
	const real_T *b15 = (const real_T*) ssGetInputPortSignal(S,74);
	const real_T *b16 = (const real_T*) ssGetInputPortSignal(S,75);
	const real_T *b17 = (const real_T*) ssGetInputPortSignal(S,76);
	const real_T *b18 = (const real_T*) ssGetInputPortSignal(S,77);
	const real_T *b19 = (const real_T*) ssGetInputPortSignal(S,78);
	const real_T *b20 = (const real_T*) ssGetInputPortSignal(S,79);
	const real_T *C01 = (const real_T*) ssGetInputPortSignal(S,80);
	const real_T *C02 = (const real_T*) ssGetInputPortSignal(S,81);
	const real_T *C03 = (const real_T*) ssGetInputPortSignal(S,82);
	const real_T *C04 = (const real_T*) ssGetInputPortSignal(S,83);
	const real_T *C05 = (const real_T*) ssGetInputPortSignal(S,84);
	const real_T *C06 = (const real_T*) ssGetInputPortSignal(S,85);
	const real_T *C07 = (const real_T*) ssGetInputPortSignal(S,86);
	const real_T *C08 = (const real_T*) ssGetInputPortSignal(S,87);
	const real_T *C09 = (const real_T*) ssGetInputPortSignal(S,88);
	const real_T *C10 = (const real_T*) ssGetInputPortSignal(S,89);
	const real_T *C11 = (const real_T*) ssGetInputPortSignal(S,90);
	const real_T *C12 = (const real_T*) ssGetInputPortSignal(S,91);
	const real_T *C13 = (const real_T*) ssGetInputPortSignal(S,92);
	const real_T *C14 = (const real_T*) ssGetInputPortSignal(S,93);
	const real_T *C15 = (const real_T*) ssGetInputPortSignal(S,94);
	const real_T *C16 = (const real_T*) ssGetInputPortSignal(S,95);
	const real_T *C17 = (const real_T*) ssGetInputPortSignal(S,96);
	const real_T *C18 = (const real_T*) ssGetInputPortSignal(S,97);
	const real_T *C19 = (const real_T*) ssGetInputPortSignal(S,98);
	const real_T *D01 = (const real_T*) ssGetInputPortSignal(S,99);
	const real_T *D02 = (const real_T*) ssGetInputPortSignal(S,100);
	const real_T *D03 = (const real_T*) ssGetInputPortSignal(S,101);
	const real_T *D04 = (const real_T*) ssGetInputPortSignal(S,102);
	const real_T *D05 = (const real_T*) ssGetInputPortSignal(S,103);
	const real_T *D06 = (const real_T*) ssGetInputPortSignal(S,104);
	const real_T *D07 = (const real_T*) ssGetInputPortSignal(S,105);
	const real_T *D08 = (const real_T*) ssGetInputPortSignal(S,106);
	const real_T *D09 = (const real_T*) ssGetInputPortSignal(S,107);
	const real_T *D10 = (const real_T*) ssGetInputPortSignal(S,108);
	const real_T *D11 = (const real_T*) ssGetInputPortSignal(S,109);
	const real_T *D12 = (const real_T*) ssGetInputPortSignal(S,110);
	const real_T *D13 = (const real_T*) ssGetInputPortSignal(S,111);
	const real_T *D14 = (const real_T*) ssGetInputPortSignal(S,112);
	const real_T *D15 = (const real_T*) ssGetInputPortSignal(S,113);
	const real_T *D16 = (const real_T*) ssGetInputPortSignal(S,114);
	const real_T *D17 = (const real_T*) ssGetInputPortSignal(S,115);
	const real_T *D18 = (const real_T*) ssGetInputPortSignal(S,116);
	const real_T *D19 = (const real_T*) ssGetInputPortSignal(S,117);
	const real_T *D20 = (const real_T*) ssGetInputPortSignal(S,118);
	const real_T *c02 = (const real_T*) ssGetInputPortSignal(S,119);
	const real_T *c03 = (const real_T*) ssGetInputPortSignal(S,120);
	const real_T *c04 = (const real_T*) ssGetInputPortSignal(S,121);
	const real_T *c05 = (const real_T*) ssGetInputPortSignal(S,122);
	const real_T *c06 = (const real_T*) ssGetInputPortSignal(S,123);
	const real_T *c07 = (const real_T*) ssGetInputPortSignal(S,124);
	const real_T *c08 = (const real_T*) ssGetInputPortSignal(S,125);
	const real_T *c09 = (const real_T*) ssGetInputPortSignal(S,126);
	const real_T *c10 = (const real_T*) ssGetInputPortSignal(S,127);
	const real_T *c11 = (const real_T*) ssGetInputPortSignal(S,128);
	const real_T *c12 = (const real_T*) ssGetInputPortSignal(S,129);
	const real_T *c13 = (const real_T*) ssGetInputPortSignal(S,130);
	const real_T *c14 = (const real_T*) ssGetInputPortSignal(S,131);
	const real_T *c15 = (const real_T*) ssGetInputPortSignal(S,132);
	const real_T *c16 = (const real_T*) ssGetInputPortSignal(S,133);
	const real_T *c17 = (const real_T*) ssGetInputPortSignal(S,134);
	const real_T *c18 = (const real_T*) ssGetInputPortSignal(S,135);
	const real_T *c19 = (const real_T*) ssGetInputPortSignal(S,136);
	const real_T *c20 = (const real_T*) ssGetInputPortSignal(S,137);
	const real_T *minusA_times_x0 = (const real_T*) ssGetInputPortSignal(S,138);
	
    real_T *u0 = (real_T*) ssGetOutputPortSignal(S,0);
	
	

	/* Solver data */
	static mpcc_params params;
	static mpcc_output output;
	static mpcc_info info;	
	solver_int32_default exitflag;

	/* Extra NMPC data */
	

	/* Copy inputs */
	for( i=0; i<169; i++)
	{ 
		params.H01[i] = (double) H01[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H02[i] = (double) H02[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H03[i] = (double) H03[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H04[i] = (double) H04[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H05[i] = (double) H05[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H06[i] = (double) H06[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H07[i] = (double) H07[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H08[i] = (double) H08[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H09[i] = (double) H09[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H10[i] = (double) H10[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H11[i] = (double) H11[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H12[i] = (double) H12[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H13[i] = (double) H13[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H14[i] = (double) H14[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H15[i] = (double) H15[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H16[i] = (double) H16[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H17[i] = (double) H17[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H18[i] = (double) H18[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H19[i] = (double) H19[i]; 
	}

	for( i=0; i<169; i++)
	{ 
		params.H20[i] = (double) H20[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f01[i] = (double) f01[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f02[i] = (double) f02[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f03[i] = (double) f03[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f04[i] = (double) f04[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f05[i] = (double) f05[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f06[i] = (double) f06[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f07[i] = (double) f07[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f08[i] = (double) f08[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f09[i] = (double) f09[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f10[i] = (double) f10[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f11[i] = (double) f11[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f12[i] = (double) f12[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f13[i] = (double) f13[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f14[i] = (double) f14[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f15[i] = (double) f15[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f16[i] = (double) f16[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f17[i] = (double) f17[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f18[i] = (double) f18[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f19[i] = (double) f19[i]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f20[i] = (double) f20[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A01[i] = (double) A01[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A02[i] = (double) A02[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A03[i] = (double) A03[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A04[i] = (double) A04[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A05[i] = (double) A05[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A06[i] = (double) A06[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A07[i] = (double) A07[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A08[i] = (double) A08[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A09[i] = (double) A09[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A10[i] = (double) A10[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A11[i] = (double) A11[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A12[i] = (double) A12[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A13[i] = (double) A13[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A14[i] = (double) A14[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A15[i] = (double) A15[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A16[i] = (double) A16[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A17[i] = (double) A17[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A18[i] = (double) A18[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A19[i] = (double) A19[i]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A20[i] = (double) A20[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b01[i] = (double) b01[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b02[i] = (double) b02[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b03[i] = (double) b03[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b04[i] = (double) b04[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b05[i] = (double) b05[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b06[i] = (double) b06[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b07[i] = (double) b07[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b08[i] = (double) b08[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b09[i] = (double) b09[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b10[i] = (double) b10[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b11[i] = (double) b11[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b12[i] = (double) b12[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b13[i] = (double) b13[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b14[i] = (double) b14[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b15[i] = (double) b15[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b16[i] = (double) b16[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b17[i] = (double) b17[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b18[i] = (double) b18[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b19[i] = (double) b19[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b20[i] = (double) b20[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C01[i] = (double) C01[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C02[i] = (double) C02[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C03[i] = (double) C03[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C04[i] = (double) C04[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C05[i] = (double) C05[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C06[i] = (double) C06[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C07[i] = (double) C07[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C08[i] = (double) C08[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C09[i] = (double) C09[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C10[i] = (double) C10[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C11[i] = (double) C11[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C12[i] = (double) C12[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C13[i] = (double) C13[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C14[i] = (double) C14[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C15[i] = (double) C15[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C16[i] = (double) C16[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C17[i] = (double) C17[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C18[i] = (double) C18[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.C19[i] = (double) C19[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D01[i] = (double) D01[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D02[i] = (double) D02[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D03[i] = (double) D03[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D04[i] = (double) D04[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D05[i] = (double) D05[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D06[i] = (double) D06[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D07[i] = (double) D07[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D08[i] = (double) D08[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D09[i] = (double) D09[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D10[i] = (double) D10[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D11[i] = (double) D11[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D12[i] = (double) D12[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D13[i] = (double) D13[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D14[i] = (double) D14[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D15[i] = (double) D15[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D16[i] = (double) D16[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D17[i] = (double) D17[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D18[i] = (double) D18[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D19[i] = (double) D19[i]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D20[i] = (double) D20[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c02[i] = (double) c02[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c03[i] = (double) c03[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c04[i] = (double) c04[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c05[i] = (double) c05[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c06[i] = (double) c06[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c07[i] = (double) c07[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c08[i] = (double) c08[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c09[i] = (double) c09[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c10[i] = (double) c10[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c11[i] = (double) c11[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c12[i] = (double) c12[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c13[i] = (double) c13[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c14[i] = (double) c14[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c15[i] = (double) c15[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c16[i] = (double) c16[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c17[i] = (double) c17[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c18[i] = (double) c18[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c19[i] = (double) c19[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c20[i] = (double) c20[i]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.minusA_times_x0[i] = (double) minusA_times_x0[i]; 
	}

	

	

    #if SET_PRINTLEVEL_mpcc > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) 
		{
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* Call solver */
	exitflag = mpcc_solve(&params, &output, &info, fp );

	#if SET_PRINTLEVEL_mpcc > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			ssPrintf("%c",i);
		}
		fclose(fp);
	#endif

	

	/* Copy outputs */
	for( i=0; i<260; i++)
	{ 
		u0[i] = (real_T) output.u0[i]; 
	}

	
}





/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
}
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif



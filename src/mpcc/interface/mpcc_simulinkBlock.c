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

	/* initialize input ports - there are 349 in total */
    if (!ssSetNumInputPorts(S, 349)) return;
    	
	/* Input Port 0 */
    ssSetInputPortMatrixDimensions(S,  0, 14, 14);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 14, 14);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 14, 14);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 14, 14);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 3, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/	
	/* Input Port 4 */
    ssSetInputPortMatrixDimensions(S,  4, 14, 14);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 4, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 4, 1); /*direct input signal access*/	
	/* Input Port 5 */
    ssSetInputPortMatrixDimensions(S,  5, 14, 14);
    ssSetInputPortDataType(S, 5, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 5, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 5, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 5, 1); /*direct input signal access*/	
	/* Input Port 6 */
    ssSetInputPortMatrixDimensions(S,  6, 14, 14);
    ssSetInputPortDataType(S, 6, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 6, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 6, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 6, 1); /*direct input signal access*/	
	/* Input Port 7 */
    ssSetInputPortMatrixDimensions(S,  7, 14, 14);
    ssSetInputPortDataType(S, 7, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 7, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 7, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 7, 1); /*direct input signal access*/	
	/* Input Port 8 */
    ssSetInputPortMatrixDimensions(S,  8, 14, 14);
    ssSetInputPortDataType(S, 8, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 8, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 8, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 8, 1); /*direct input signal access*/	
	/* Input Port 9 */
    ssSetInputPortMatrixDimensions(S,  9, 14, 14);
    ssSetInputPortDataType(S, 9, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 9, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 9, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 9, 1); /*direct input signal access*/	
	/* Input Port 10 */
    ssSetInputPortMatrixDimensions(S,  10, 14, 14);
    ssSetInputPortDataType(S, 10, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 10, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 10, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 10, 1); /*direct input signal access*/	
	/* Input Port 11 */
    ssSetInputPortMatrixDimensions(S,  11, 14, 14);
    ssSetInputPortDataType(S, 11, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 11, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 11, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 11, 1); /*direct input signal access*/	
	/* Input Port 12 */
    ssSetInputPortMatrixDimensions(S,  12, 14, 14);
    ssSetInputPortDataType(S, 12, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 12, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 12, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 12, 1); /*direct input signal access*/	
	/* Input Port 13 */
    ssSetInputPortMatrixDimensions(S,  13, 14, 14);
    ssSetInputPortDataType(S, 13, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 13, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 13, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 13, 1); /*direct input signal access*/	
	/* Input Port 14 */
    ssSetInputPortMatrixDimensions(S,  14, 14, 14);
    ssSetInputPortDataType(S, 14, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 14, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 14, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 14, 1); /*direct input signal access*/	
	/* Input Port 15 */
    ssSetInputPortMatrixDimensions(S,  15, 14, 14);
    ssSetInputPortDataType(S, 15, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 15, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 15, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 15, 1); /*direct input signal access*/	
	/* Input Port 16 */
    ssSetInputPortMatrixDimensions(S,  16, 14, 14);
    ssSetInputPortDataType(S, 16, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 16, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 16, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 16, 1); /*direct input signal access*/	
	/* Input Port 17 */
    ssSetInputPortMatrixDimensions(S,  17, 14, 14);
    ssSetInputPortDataType(S, 17, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 17, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 17, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 17, 1); /*direct input signal access*/	
	/* Input Port 18 */
    ssSetInputPortMatrixDimensions(S,  18, 14, 14);
    ssSetInputPortDataType(S, 18, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 18, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 18, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 18, 1); /*direct input signal access*/	
	/* Input Port 19 */
    ssSetInputPortMatrixDimensions(S,  19, 14, 14);
    ssSetInputPortDataType(S, 19, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 19, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 19, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 19, 1); /*direct input signal access*/	
	/* Input Port 20 */
    ssSetInputPortMatrixDimensions(S,  20, 14, 14);
    ssSetInputPortDataType(S, 20, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 20, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 20, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 20, 1); /*direct input signal access*/	
	/* Input Port 21 */
    ssSetInputPortMatrixDimensions(S,  21, 14, 14);
    ssSetInputPortDataType(S, 21, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 21, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 21, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 21, 1); /*direct input signal access*/	
	/* Input Port 22 */
    ssSetInputPortMatrixDimensions(S,  22, 14, 14);
    ssSetInputPortDataType(S, 22, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 22, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 22, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 22, 1); /*direct input signal access*/	
	/* Input Port 23 */
    ssSetInputPortMatrixDimensions(S,  23, 14, 14);
    ssSetInputPortDataType(S, 23, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 23, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 23, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 23, 1); /*direct input signal access*/	
	/* Input Port 24 */
    ssSetInputPortMatrixDimensions(S,  24, 14, 14);
    ssSetInputPortDataType(S, 24, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 24, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 24, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 24, 1); /*direct input signal access*/	
	/* Input Port 25 */
    ssSetInputPortMatrixDimensions(S,  25, 14, 14);
    ssSetInputPortDataType(S, 25, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 25, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 25, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 25, 1); /*direct input signal access*/	
	/* Input Port 26 */
    ssSetInputPortMatrixDimensions(S,  26, 14, 14);
    ssSetInputPortDataType(S, 26, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 26, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 26, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 26, 1); /*direct input signal access*/	
	/* Input Port 27 */
    ssSetInputPortMatrixDimensions(S,  27, 14, 14);
    ssSetInputPortDataType(S, 27, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 27, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 27, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 27, 1); /*direct input signal access*/	
	/* Input Port 28 */
    ssSetInputPortMatrixDimensions(S,  28, 14, 14);
    ssSetInputPortDataType(S, 28, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 28, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 28, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 28, 1); /*direct input signal access*/	
	/* Input Port 29 */
    ssSetInputPortMatrixDimensions(S,  29, 14, 14);
    ssSetInputPortDataType(S, 29, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 29, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 29, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 29, 1); /*direct input signal access*/	
	/* Input Port 30 */
    ssSetInputPortMatrixDimensions(S,  30, 14, 14);
    ssSetInputPortDataType(S, 30, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 30, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 30, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 30, 1); /*direct input signal access*/	
	/* Input Port 31 */
    ssSetInputPortMatrixDimensions(S,  31, 14, 14);
    ssSetInputPortDataType(S, 31, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 31, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 31, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 31, 1); /*direct input signal access*/	
	/* Input Port 32 */
    ssSetInputPortMatrixDimensions(S,  32, 14, 14);
    ssSetInputPortDataType(S, 32, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 32, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 32, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 32, 1); /*direct input signal access*/	
	/* Input Port 33 */
    ssSetInputPortMatrixDimensions(S,  33, 14, 14);
    ssSetInputPortDataType(S, 33, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 33, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 33, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 33, 1); /*direct input signal access*/	
	/* Input Port 34 */
    ssSetInputPortMatrixDimensions(S,  34, 14, 14);
    ssSetInputPortDataType(S, 34, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 34, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 34, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 34, 1); /*direct input signal access*/	
	/* Input Port 35 */
    ssSetInputPortMatrixDimensions(S,  35, 14, 14);
    ssSetInputPortDataType(S, 35, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 35, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 35, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 35, 1); /*direct input signal access*/	
	/* Input Port 36 */
    ssSetInputPortMatrixDimensions(S,  36, 14, 14);
    ssSetInputPortDataType(S, 36, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 36, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 36, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 36, 1); /*direct input signal access*/	
	/* Input Port 37 */
    ssSetInputPortMatrixDimensions(S,  37, 14, 14);
    ssSetInputPortDataType(S, 37, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 37, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 37, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 37, 1); /*direct input signal access*/	
	/* Input Port 38 */
    ssSetInputPortMatrixDimensions(S,  38, 14, 14);
    ssSetInputPortDataType(S, 38, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 38, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 38, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 38, 1); /*direct input signal access*/	
	/* Input Port 39 */
    ssSetInputPortMatrixDimensions(S,  39, 14, 14);
    ssSetInputPortDataType(S, 39, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 39, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 39, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 39, 1); /*direct input signal access*/	
	/* Input Port 40 */
    ssSetInputPortMatrixDimensions(S,  40, 14, 14);
    ssSetInputPortDataType(S, 40, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 40, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 40, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 40, 1); /*direct input signal access*/	
	/* Input Port 41 */
    ssSetInputPortMatrixDimensions(S,  41, 14, 14);
    ssSetInputPortDataType(S, 41, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 41, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 41, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 41, 1); /*direct input signal access*/	
	/* Input Port 42 */
    ssSetInputPortMatrixDimensions(S,  42, 14, 14);
    ssSetInputPortDataType(S, 42, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 42, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 42, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 42, 1); /*direct input signal access*/	
	/* Input Port 43 */
    ssSetInputPortMatrixDimensions(S,  43, 14, 14);
    ssSetInputPortDataType(S, 43, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 43, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 43, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 43, 1); /*direct input signal access*/	
	/* Input Port 44 */
    ssSetInputPortMatrixDimensions(S,  44, 14, 14);
    ssSetInputPortDataType(S, 44, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 44, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 44, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 44, 1); /*direct input signal access*/	
	/* Input Port 45 */
    ssSetInputPortMatrixDimensions(S,  45, 14, 14);
    ssSetInputPortDataType(S, 45, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 45, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 45, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 45, 1); /*direct input signal access*/	
	/* Input Port 46 */
    ssSetInputPortMatrixDimensions(S,  46, 14, 14);
    ssSetInputPortDataType(S, 46, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 46, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 46, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 46, 1); /*direct input signal access*/	
	/* Input Port 47 */
    ssSetInputPortMatrixDimensions(S,  47, 14, 14);
    ssSetInputPortDataType(S, 47, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 47, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 47, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 47, 1); /*direct input signal access*/	
	/* Input Port 48 */
    ssSetInputPortMatrixDimensions(S,  48, 14, 14);
    ssSetInputPortDataType(S, 48, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 48, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 48, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 48, 1); /*direct input signal access*/	
	/* Input Port 49 */
    ssSetInputPortMatrixDimensions(S,  49, 14, 14);
    ssSetInputPortDataType(S, 49, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 49, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 49, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 49, 1); /*direct input signal access*/	
	/* Input Port 50 */
    ssSetInputPortMatrixDimensions(S,  50, 14, 1);
    ssSetInputPortDataType(S, 50, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 50, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 50, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 50, 1); /*direct input signal access*/	
	/* Input Port 51 */
    ssSetInputPortMatrixDimensions(S,  51, 14, 1);
    ssSetInputPortDataType(S, 51, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 51, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 51, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 51, 1); /*direct input signal access*/	
	/* Input Port 52 */
    ssSetInputPortMatrixDimensions(S,  52, 14, 1);
    ssSetInputPortDataType(S, 52, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 52, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 52, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 52, 1); /*direct input signal access*/	
	/* Input Port 53 */
    ssSetInputPortMatrixDimensions(S,  53, 14, 1);
    ssSetInputPortDataType(S, 53, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 53, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 53, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 53, 1); /*direct input signal access*/	
	/* Input Port 54 */
    ssSetInputPortMatrixDimensions(S,  54, 14, 1);
    ssSetInputPortDataType(S, 54, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 54, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 54, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 54, 1); /*direct input signal access*/	
	/* Input Port 55 */
    ssSetInputPortMatrixDimensions(S,  55, 14, 1);
    ssSetInputPortDataType(S, 55, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 55, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 55, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 55, 1); /*direct input signal access*/	
	/* Input Port 56 */
    ssSetInputPortMatrixDimensions(S,  56, 14, 1);
    ssSetInputPortDataType(S, 56, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 56, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 56, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 56, 1); /*direct input signal access*/	
	/* Input Port 57 */
    ssSetInputPortMatrixDimensions(S,  57, 14, 1);
    ssSetInputPortDataType(S, 57, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 57, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 57, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 57, 1); /*direct input signal access*/	
	/* Input Port 58 */
    ssSetInputPortMatrixDimensions(S,  58, 14, 1);
    ssSetInputPortDataType(S, 58, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 58, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 58, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 58, 1); /*direct input signal access*/	
	/* Input Port 59 */
    ssSetInputPortMatrixDimensions(S,  59, 14, 1);
    ssSetInputPortDataType(S, 59, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 59, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 59, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 59, 1); /*direct input signal access*/	
	/* Input Port 60 */
    ssSetInputPortMatrixDimensions(S,  60, 14, 1);
    ssSetInputPortDataType(S, 60, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 60, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 60, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 60, 1); /*direct input signal access*/	
	/* Input Port 61 */
    ssSetInputPortMatrixDimensions(S,  61, 14, 1);
    ssSetInputPortDataType(S, 61, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 61, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 61, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 61, 1); /*direct input signal access*/	
	/* Input Port 62 */
    ssSetInputPortMatrixDimensions(S,  62, 14, 1);
    ssSetInputPortDataType(S, 62, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 62, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 62, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 62, 1); /*direct input signal access*/	
	/* Input Port 63 */
    ssSetInputPortMatrixDimensions(S,  63, 14, 1);
    ssSetInputPortDataType(S, 63, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 63, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 63, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 63, 1); /*direct input signal access*/	
	/* Input Port 64 */
    ssSetInputPortMatrixDimensions(S,  64, 14, 1);
    ssSetInputPortDataType(S, 64, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 64, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 64, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 64, 1); /*direct input signal access*/	
	/* Input Port 65 */
    ssSetInputPortMatrixDimensions(S,  65, 14, 1);
    ssSetInputPortDataType(S, 65, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 65, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 65, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 65, 1); /*direct input signal access*/	
	/* Input Port 66 */
    ssSetInputPortMatrixDimensions(S,  66, 14, 1);
    ssSetInputPortDataType(S, 66, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 66, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 66, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 66, 1); /*direct input signal access*/	
	/* Input Port 67 */
    ssSetInputPortMatrixDimensions(S,  67, 14, 1);
    ssSetInputPortDataType(S, 67, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 67, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 67, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 67, 1); /*direct input signal access*/	
	/* Input Port 68 */
    ssSetInputPortMatrixDimensions(S,  68, 14, 1);
    ssSetInputPortDataType(S, 68, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 68, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 68, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 68, 1); /*direct input signal access*/	
	/* Input Port 69 */
    ssSetInputPortMatrixDimensions(S,  69, 14, 1);
    ssSetInputPortDataType(S, 69, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 69, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 69, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 69, 1); /*direct input signal access*/	
	/* Input Port 70 */
    ssSetInputPortMatrixDimensions(S,  70, 14, 1);
    ssSetInputPortDataType(S, 70, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 70, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 70, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 70, 1); /*direct input signal access*/	
	/* Input Port 71 */
    ssSetInputPortMatrixDimensions(S,  71, 14, 1);
    ssSetInputPortDataType(S, 71, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 71, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 71, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 71, 1); /*direct input signal access*/	
	/* Input Port 72 */
    ssSetInputPortMatrixDimensions(S,  72, 14, 1);
    ssSetInputPortDataType(S, 72, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 72, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 72, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 72, 1); /*direct input signal access*/	
	/* Input Port 73 */
    ssSetInputPortMatrixDimensions(S,  73, 14, 1);
    ssSetInputPortDataType(S, 73, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 73, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 73, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 73, 1); /*direct input signal access*/	
	/* Input Port 74 */
    ssSetInputPortMatrixDimensions(S,  74, 14, 1);
    ssSetInputPortDataType(S, 74, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 74, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 74, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 74, 1); /*direct input signal access*/	
	/* Input Port 75 */
    ssSetInputPortMatrixDimensions(S,  75, 14, 1);
    ssSetInputPortDataType(S, 75, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 75, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 75, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 75, 1); /*direct input signal access*/	
	/* Input Port 76 */
    ssSetInputPortMatrixDimensions(S,  76, 14, 1);
    ssSetInputPortDataType(S, 76, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 76, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 76, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 76, 1); /*direct input signal access*/	
	/* Input Port 77 */
    ssSetInputPortMatrixDimensions(S,  77, 14, 1);
    ssSetInputPortDataType(S, 77, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 77, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 77, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 77, 1); /*direct input signal access*/	
	/* Input Port 78 */
    ssSetInputPortMatrixDimensions(S,  78, 14, 1);
    ssSetInputPortDataType(S, 78, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 78, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 78, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 78, 1); /*direct input signal access*/	
	/* Input Port 79 */
    ssSetInputPortMatrixDimensions(S,  79, 14, 1);
    ssSetInputPortDataType(S, 79, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 79, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 79, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 79, 1); /*direct input signal access*/	
	/* Input Port 80 */
    ssSetInputPortMatrixDimensions(S,  80, 14, 1);
    ssSetInputPortDataType(S, 80, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 80, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 80, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 80, 1); /*direct input signal access*/	
	/* Input Port 81 */
    ssSetInputPortMatrixDimensions(S,  81, 14, 1);
    ssSetInputPortDataType(S, 81, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 81, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 81, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 81, 1); /*direct input signal access*/	
	/* Input Port 82 */
    ssSetInputPortMatrixDimensions(S,  82, 14, 1);
    ssSetInputPortDataType(S, 82, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 82, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 82, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 82, 1); /*direct input signal access*/	
	/* Input Port 83 */
    ssSetInputPortMatrixDimensions(S,  83, 14, 1);
    ssSetInputPortDataType(S, 83, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 83, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 83, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 83, 1); /*direct input signal access*/	
	/* Input Port 84 */
    ssSetInputPortMatrixDimensions(S,  84, 14, 1);
    ssSetInputPortDataType(S, 84, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 84, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 84, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 84, 1); /*direct input signal access*/	
	/* Input Port 85 */
    ssSetInputPortMatrixDimensions(S,  85, 14, 1);
    ssSetInputPortDataType(S, 85, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 85, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 85, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 85, 1); /*direct input signal access*/	
	/* Input Port 86 */
    ssSetInputPortMatrixDimensions(S,  86, 14, 1);
    ssSetInputPortDataType(S, 86, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 86, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 86, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 86, 1); /*direct input signal access*/	
	/* Input Port 87 */
    ssSetInputPortMatrixDimensions(S,  87, 14, 1);
    ssSetInputPortDataType(S, 87, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 87, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 87, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 87, 1); /*direct input signal access*/	
	/* Input Port 88 */
    ssSetInputPortMatrixDimensions(S,  88, 14, 1);
    ssSetInputPortDataType(S, 88, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 88, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 88, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 88, 1); /*direct input signal access*/	
	/* Input Port 89 */
    ssSetInputPortMatrixDimensions(S,  89, 14, 1);
    ssSetInputPortDataType(S, 89, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 89, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 89, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 89, 1); /*direct input signal access*/	
	/* Input Port 90 */
    ssSetInputPortMatrixDimensions(S,  90, 14, 1);
    ssSetInputPortDataType(S, 90, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 90, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 90, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 90, 1); /*direct input signal access*/	
	/* Input Port 91 */
    ssSetInputPortMatrixDimensions(S,  91, 14, 1);
    ssSetInputPortDataType(S, 91, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 91, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 91, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 91, 1); /*direct input signal access*/	
	/* Input Port 92 */
    ssSetInputPortMatrixDimensions(S,  92, 14, 1);
    ssSetInputPortDataType(S, 92, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 92, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 92, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 92, 1); /*direct input signal access*/	
	/* Input Port 93 */
    ssSetInputPortMatrixDimensions(S,  93, 14, 1);
    ssSetInputPortDataType(S, 93, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 93, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 93, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 93, 1); /*direct input signal access*/	
	/* Input Port 94 */
    ssSetInputPortMatrixDimensions(S,  94, 14, 1);
    ssSetInputPortDataType(S, 94, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 94, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 94, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 94, 1); /*direct input signal access*/	
	/* Input Port 95 */
    ssSetInputPortMatrixDimensions(S,  95, 14, 1);
    ssSetInputPortDataType(S, 95, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 95, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 95, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 95, 1); /*direct input signal access*/	
	/* Input Port 96 */
    ssSetInputPortMatrixDimensions(S,  96, 14, 1);
    ssSetInputPortDataType(S, 96, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 96, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 96, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 96, 1); /*direct input signal access*/	
	/* Input Port 97 */
    ssSetInputPortMatrixDimensions(S,  97, 14, 1);
    ssSetInputPortDataType(S, 97, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 97, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 97, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 97, 1); /*direct input signal access*/	
	/* Input Port 98 */
    ssSetInputPortMatrixDimensions(S,  98, 14, 1);
    ssSetInputPortDataType(S, 98, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 98, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 98, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 98, 1); /*direct input signal access*/	
	/* Input Port 99 */
    ssSetInputPortMatrixDimensions(S,  99, 14, 1);
    ssSetInputPortDataType(S, 99, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 99, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 99, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 99, 1); /*direct input signal access*/	
	/* Input Port 100 */
    ssSetInputPortMatrixDimensions(S,  100, 2, 14);
    ssSetInputPortDataType(S, 100, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 100, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 100, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 100, 1); /*direct input signal access*/	
	/* Input Port 101 */
    ssSetInputPortMatrixDimensions(S,  101, 2, 14);
    ssSetInputPortDataType(S, 101, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 101, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 101, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 101, 1); /*direct input signal access*/	
	/* Input Port 102 */
    ssSetInputPortMatrixDimensions(S,  102, 2, 14);
    ssSetInputPortDataType(S, 102, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 102, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 102, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 102, 1); /*direct input signal access*/	
	/* Input Port 103 */
    ssSetInputPortMatrixDimensions(S,  103, 2, 14);
    ssSetInputPortDataType(S, 103, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 103, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 103, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 103, 1); /*direct input signal access*/	
	/* Input Port 104 */
    ssSetInputPortMatrixDimensions(S,  104, 2, 14);
    ssSetInputPortDataType(S, 104, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 104, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 104, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 104, 1); /*direct input signal access*/	
	/* Input Port 105 */
    ssSetInputPortMatrixDimensions(S,  105, 2, 14);
    ssSetInputPortDataType(S, 105, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 105, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 105, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 105, 1); /*direct input signal access*/	
	/* Input Port 106 */
    ssSetInputPortMatrixDimensions(S,  106, 2, 14);
    ssSetInputPortDataType(S, 106, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 106, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 106, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 106, 1); /*direct input signal access*/	
	/* Input Port 107 */
    ssSetInputPortMatrixDimensions(S,  107, 2, 14);
    ssSetInputPortDataType(S, 107, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 107, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 107, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 107, 1); /*direct input signal access*/	
	/* Input Port 108 */
    ssSetInputPortMatrixDimensions(S,  108, 2, 14);
    ssSetInputPortDataType(S, 108, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 108, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 108, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 108, 1); /*direct input signal access*/	
	/* Input Port 109 */
    ssSetInputPortMatrixDimensions(S,  109, 2, 14);
    ssSetInputPortDataType(S, 109, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 109, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 109, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 109, 1); /*direct input signal access*/	
	/* Input Port 110 */
    ssSetInputPortMatrixDimensions(S,  110, 2, 14);
    ssSetInputPortDataType(S, 110, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 110, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 110, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 110, 1); /*direct input signal access*/	
	/* Input Port 111 */
    ssSetInputPortMatrixDimensions(S,  111, 2, 14);
    ssSetInputPortDataType(S, 111, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 111, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 111, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 111, 1); /*direct input signal access*/	
	/* Input Port 112 */
    ssSetInputPortMatrixDimensions(S,  112, 2, 14);
    ssSetInputPortDataType(S, 112, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 112, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 112, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 112, 1); /*direct input signal access*/	
	/* Input Port 113 */
    ssSetInputPortMatrixDimensions(S,  113, 2, 14);
    ssSetInputPortDataType(S, 113, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 113, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 113, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 113, 1); /*direct input signal access*/	
	/* Input Port 114 */
    ssSetInputPortMatrixDimensions(S,  114, 2, 14);
    ssSetInputPortDataType(S, 114, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 114, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 114, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 114, 1); /*direct input signal access*/	
	/* Input Port 115 */
    ssSetInputPortMatrixDimensions(S,  115, 2, 14);
    ssSetInputPortDataType(S, 115, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 115, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 115, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 115, 1); /*direct input signal access*/	
	/* Input Port 116 */
    ssSetInputPortMatrixDimensions(S,  116, 2, 14);
    ssSetInputPortDataType(S, 116, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 116, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 116, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 116, 1); /*direct input signal access*/	
	/* Input Port 117 */
    ssSetInputPortMatrixDimensions(S,  117, 2, 14);
    ssSetInputPortDataType(S, 117, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 117, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 117, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 117, 1); /*direct input signal access*/	
	/* Input Port 118 */
    ssSetInputPortMatrixDimensions(S,  118, 2, 14);
    ssSetInputPortDataType(S, 118, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 118, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 118, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 118, 1); /*direct input signal access*/	
	/* Input Port 119 */
    ssSetInputPortMatrixDimensions(S,  119, 2, 14);
    ssSetInputPortDataType(S, 119, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 119, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 119, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 119, 1); /*direct input signal access*/	
	/* Input Port 120 */
    ssSetInputPortMatrixDimensions(S,  120, 2, 14);
    ssSetInputPortDataType(S, 120, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 120, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 120, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 120, 1); /*direct input signal access*/	
	/* Input Port 121 */
    ssSetInputPortMatrixDimensions(S,  121, 2, 14);
    ssSetInputPortDataType(S, 121, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 121, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 121, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 121, 1); /*direct input signal access*/	
	/* Input Port 122 */
    ssSetInputPortMatrixDimensions(S,  122, 2, 14);
    ssSetInputPortDataType(S, 122, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 122, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 122, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 122, 1); /*direct input signal access*/	
	/* Input Port 123 */
    ssSetInputPortMatrixDimensions(S,  123, 2, 14);
    ssSetInputPortDataType(S, 123, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 123, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 123, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 123, 1); /*direct input signal access*/	
	/* Input Port 124 */
    ssSetInputPortMatrixDimensions(S,  124, 2, 14);
    ssSetInputPortDataType(S, 124, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 124, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 124, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 124, 1); /*direct input signal access*/	
	/* Input Port 125 */
    ssSetInputPortMatrixDimensions(S,  125, 2, 14);
    ssSetInputPortDataType(S, 125, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 125, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 125, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 125, 1); /*direct input signal access*/	
	/* Input Port 126 */
    ssSetInputPortMatrixDimensions(S,  126, 2, 14);
    ssSetInputPortDataType(S, 126, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 126, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 126, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 126, 1); /*direct input signal access*/	
	/* Input Port 127 */
    ssSetInputPortMatrixDimensions(S,  127, 2, 14);
    ssSetInputPortDataType(S, 127, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 127, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 127, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 127, 1); /*direct input signal access*/	
	/* Input Port 128 */
    ssSetInputPortMatrixDimensions(S,  128, 2, 14);
    ssSetInputPortDataType(S, 128, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 128, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 128, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 128, 1); /*direct input signal access*/	
	/* Input Port 129 */
    ssSetInputPortMatrixDimensions(S,  129, 2, 14);
    ssSetInputPortDataType(S, 129, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 129, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 129, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 129, 1); /*direct input signal access*/	
	/* Input Port 130 */
    ssSetInputPortMatrixDimensions(S,  130, 2, 14);
    ssSetInputPortDataType(S, 130, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 130, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 130, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 130, 1); /*direct input signal access*/	
	/* Input Port 131 */
    ssSetInputPortMatrixDimensions(S,  131, 2, 14);
    ssSetInputPortDataType(S, 131, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 131, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 131, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 131, 1); /*direct input signal access*/	
	/* Input Port 132 */
    ssSetInputPortMatrixDimensions(S,  132, 2, 14);
    ssSetInputPortDataType(S, 132, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 132, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 132, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 132, 1); /*direct input signal access*/	
	/* Input Port 133 */
    ssSetInputPortMatrixDimensions(S,  133, 2, 14);
    ssSetInputPortDataType(S, 133, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 133, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 133, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 133, 1); /*direct input signal access*/	
	/* Input Port 134 */
    ssSetInputPortMatrixDimensions(S,  134, 2, 14);
    ssSetInputPortDataType(S, 134, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 134, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 134, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 134, 1); /*direct input signal access*/	
	/* Input Port 135 */
    ssSetInputPortMatrixDimensions(S,  135, 2, 14);
    ssSetInputPortDataType(S, 135, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 135, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 135, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 135, 1); /*direct input signal access*/	
	/* Input Port 136 */
    ssSetInputPortMatrixDimensions(S,  136, 2, 14);
    ssSetInputPortDataType(S, 136, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 136, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 136, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 136, 1); /*direct input signal access*/	
	/* Input Port 137 */
    ssSetInputPortMatrixDimensions(S,  137, 2, 14);
    ssSetInputPortDataType(S, 137, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 137, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 137, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 137, 1); /*direct input signal access*/	
	/* Input Port 138 */
    ssSetInputPortMatrixDimensions(S,  138, 2, 14);
    ssSetInputPortDataType(S, 138, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 138, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 138, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 138, 1); /*direct input signal access*/	
	/* Input Port 139 */
    ssSetInputPortMatrixDimensions(S,  139, 2, 14);
    ssSetInputPortDataType(S, 139, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 139, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 139, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 139, 1); /*direct input signal access*/	
	/* Input Port 140 */
    ssSetInputPortMatrixDimensions(S,  140, 2, 14);
    ssSetInputPortDataType(S, 140, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 140, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 140, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 140, 1); /*direct input signal access*/	
	/* Input Port 141 */
    ssSetInputPortMatrixDimensions(S,  141, 2, 14);
    ssSetInputPortDataType(S, 141, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 141, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 141, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 141, 1); /*direct input signal access*/	
	/* Input Port 142 */
    ssSetInputPortMatrixDimensions(S,  142, 2, 14);
    ssSetInputPortDataType(S, 142, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 142, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 142, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 142, 1); /*direct input signal access*/	
	/* Input Port 143 */
    ssSetInputPortMatrixDimensions(S,  143, 2, 14);
    ssSetInputPortDataType(S, 143, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 143, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 143, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 143, 1); /*direct input signal access*/	
	/* Input Port 144 */
    ssSetInputPortMatrixDimensions(S,  144, 2, 14);
    ssSetInputPortDataType(S, 144, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 144, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 144, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 144, 1); /*direct input signal access*/	
	/* Input Port 145 */
    ssSetInputPortMatrixDimensions(S,  145, 2, 14);
    ssSetInputPortDataType(S, 145, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 145, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 145, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 145, 1); /*direct input signal access*/	
	/* Input Port 146 */
    ssSetInputPortMatrixDimensions(S,  146, 2, 14);
    ssSetInputPortDataType(S, 146, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 146, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 146, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 146, 1); /*direct input signal access*/	
	/* Input Port 147 */
    ssSetInputPortMatrixDimensions(S,  147, 2, 14);
    ssSetInputPortDataType(S, 147, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 147, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 147, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 147, 1); /*direct input signal access*/	
	/* Input Port 148 */
    ssSetInputPortMatrixDimensions(S,  148, 2, 14);
    ssSetInputPortDataType(S, 148, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 148, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 148, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 148, 1); /*direct input signal access*/	
	/* Input Port 149 */
    ssSetInputPortMatrixDimensions(S,  149, 2, 14);
    ssSetInputPortDataType(S, 149, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 149, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 149, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 149, 1); /*direct input signal access*/	
	/* Input Port 150 */
    ssSetInputPortMatrixDimensions(S,  150, 2, 1);
    ssSetInputPortDataType(S, 150, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 150, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 150, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 150, 1); /*direct input signal access*/	
	/* Input Port 151 */
    ssSetInputPortMatrixDimensions(S,  151, 2, 1);
    ssSetInputPortDataType(S, 151, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 151, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 151, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 151, 1); /*direct input signal access*/	
	/* Input Port 152 */
    ssSetInputPortMatrixDimensions(S,  152, 2, 1);
    ssSetInputPortDataType(S, 152, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 152, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 152, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 152, 1); /*direct input signal access*/	
	/* Input Port 153 */
    ssSetInputPortMatrixDimensions(S,  153, 2, 1);
    ssSetInputPortDataType(S, 153, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 153, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 153, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 153, 1); /*direct input signal access*/	
	/* Input Port 154 */
    ssSetInputPortMatrixDimensions(S,  154, 2, 1);
    ssSetInputPortDataType(S, 154, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 154, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 154, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 154, 1); /*direct input signal access*/	
	/* Input Port 155 */
    ssSetInputPortMatrixDimensions(S,  155, 2, 1);
    ssSetInputPortDataType(S, 155, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 155, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 155, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 155, 1); /*direct input signal access*/	
	/* Input Port 156 */
    ssSetInputPortMatrixDimensions(S,  156, 2, 1);
    ssSetInputPortDataType(S, 156, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 156, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 156, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 156, 1); /*direct input signal access*/	
	/* Input Port 157 */
    ssSetInputPortMatrixDimensions(S,  157, 2, 1);
    ssSetInputPortDataType(S, 157, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 157, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 157, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 157, 1); /*direct input signal access*/	
	/* Input Port 158 */
    ssSetInputPortMatrixDimensions(S,  158, 2, 1);
    ssSetInputPortDataType(S, 158, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 158, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 158, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 158, 1); /*direct input signal access*/	
	/* Input Port 159 */
    ssSetInputPortMatrixDimensions(S,  159, 2, 1);
    ssSetInputPortDataType(S, 159, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 159, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 159, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 159, 1); /*direct input signal access*/	
	/* Input Port 160 */
    ssSetInputPortMatrixDimensions(S,  160, 2, 1);
    ssSetInputPortDataType(S, 160, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 160, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 160, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 160, 1); /*direct input signal access*/	
	/* Input Port 161 */
    ssSetInputPortMatrixDimensions(S,  161, 2, 1);
    ssSetInputPortDataType(S, 161, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 161, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 161, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 161, 1); /*direct input signal access*/	
	/* Input Port 162 */
    ssSetInputPortMatrixDimensions(S,  162, 2, 1);
    ssSetInputPortDataType(S, 162, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 162, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 162, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 162, 1); /*direct input signal access*/	
	/* Input Port 163 */
    ssSetInputPortMatrixDimensions(S,  163, 2, 1);
    ssSetInputPortDataType(S, 163, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 163, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 163, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 163, 1); /*direct input signal access*/	
	/* Input Port 164 */
    ssSetInputPortMatrixDimensions(S,  164, 2, 1);
    ssSetInputPortDataType(S, 164, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 164, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 164, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 164, 1); /*direct input signal access*/	
	/* Input Port 165 */
    ssSetInputPortMatrixDimensions(S,  165, 2, 1);
    ssSetInputPortDataType(S, 165, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 165, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 165, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 165, 1); /*direct input signal access*/	
	/* Input Port 166 */
    ssSetInputPortMatrixDimensions(S,  166, 2, 1);
    ssSetInputPortDataType(S, 166, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 166, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 166, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 166, 1); /*direct input signal access*/	
	/* Input Port 167 */
    ssSetInputPortMatrixDimensions(S,  167, 2, 1);
    ssSetInputPortDataType(S, 167, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 167, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 167, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 167, 1); /*direct input signal access*/	
	/* Input Port 168 */
    ssSetInputPortMatrixDimensions(S,  168, 2, 1);
    ssSetInputPortDataType(S, 168, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 168, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 168, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 168, 1); /*direct input signal access*/	
	/* Input Port 169 */
    ssSetInputPortMatrixDimensions(S,  169, 2, 1);
    ssSetInputPortDataType(S, 169, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 169, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 169, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 169, 1); /*direct input signal access*/	
	/* Input Port 170 */
    ssSetInputPortMatrixDimensions(S,  170, 2, 1);
    ssSetInputPortDataType(S, 170, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 170, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 170, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 170, 1); /*direct input signal access*/	
	/* Input Port 171 */
    ssSetInputPortMatrixDimensions(S,  171, 2, 1);
    ssSetInputPortDataType(S, 171, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 171, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 171, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 171, 1); /*direct input signal access*/	
	/* Input Port 172 */
    ssSetInputPortMatrixDimensions(S,  172, 2, 1);
    ssSetInputPortDataType(S, 172, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 172, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 172, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 172, 1); /*direct input signal access*/	
	/* Input Port 173 */
    ssSetInputPortMatrixDimensions(S,  173, 2, 1);
    ssSetInputPortDataType(S, 173, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 173, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 173, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 173, 1); /*direct input signal access*/	
	/* Input Port 174 */
    ssSetInputPortMatrixDimensions(S,  174, 2, 1);
    ssSetInputPortDataType(S, 174, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 174, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 174, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 174, 1); /*direct input signal access*/	
	/* Input Port 175 */
    ssSetInputPortMatrixDimensions(S,  175, 2, 1);
    ssSetInputPortDataType(S, 175, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 175, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 175, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 175, 1); /*direct input signal access*/	
	/* Input Port 176 */
    ssSetInputPortMatrixDimensions(S,  176, 2, 1);
    ssSetInputPortDataType(S, 176, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 176, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 176, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 176, 1); /*direct input signal access*/	
	/* Input Port 177 */
    ssSetInputPortMatrixDimensions(S,  177, 2, 1);
    ssSetInputPortDataType(S, 177, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 177, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 177, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 177, 1); /*direct input signal access*/	
	/* Input Port 178 */
    ssSetInputPortMatrixDimensions(S,  178, 2, 1);
    ssSetInputPortDataType(S, 178, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 178, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 178, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 178, 1); /*direct input signal access*/	
	/* Input Port 179 */
    ssSetInputPortMatrixDimensions(S,  179, 2, 1);
    ssSetInputPortDataType(S, 179, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 179, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 179, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 179, 1); /*direct input signal access*/	
	/* Input Port 180 */
    ssSetInputPortMatrixDimensions(S,  180, 2, 1);
    ssSetInputPortDataType(S, 180, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 180, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 180, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 180, 1); /*direct input signal access*/	
	/* Input Port 181 */
    ssSetInputPortMatrixDimensions(S,  181, 2, 1);
    ssSetInputPortDataType(S, 181, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 181, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 181, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 181, 1); /*direct input signal access*/	
	/* Input Port 182 */
    ssSetInputPortMatrixDimensions(S,  182, 2, 1);
    ssSetInputPortDataType(S, 182, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 182, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 182, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 182, 1); /*direct input signal access*/	
	/* Input Port 183 */
    ssSetInputPortMatrixDimensions(S,  183, 2, 1);
    ssSetInputPortDataType(S, 183, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 183, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 183, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 183, 1); /*direct input signal access*/	
	/* Input Port 184 */
    ssSetInputPortMatrixDimensions(S,  184, 2, 1);
    ssSetInputPortDataType(S, 184, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 184, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 184, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 184, 1); /*direct input signal access*/	
	/* Input Port 185 */
    ssSetInputPortMatrixDimensions(S,  185, 2, 1);
    ssSetInputPortDataType(S, 185, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 185, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 185, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 185, 1); /*direct input signal access*/	
	/* Input Port 186 */
    ssSetInputPortMatrixDimensions(S,  186, 2, 1);
    ssSetInputPortDataType(S, 186, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 186, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 186, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 186, 1); /*direct input signal access*/	
	/* Input Port 187 */
    ssSetInputPortMatrixDimensions(S,  187, 2, 1);
    ssSetInputPortDataType(S, 187, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 187, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 187, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 187, 1); /*direct input signal access*/	
	/* Input Port 188 */
    ssSetInputPortMatrixDimensions(S,  188, 2, 1);
    ssSetInputPortDataType(S, 188, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 188, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 188, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 188, 1); /*direct input signal access*/	
	/* Input Port 189 */
    ssSetInputPortMatrixDimensions(S,  189, 2, 1);
    ssSetInputPortDataType(S, 189, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 189, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 189, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 189, 1); /*direct input signal access*/	
	/* Input Port 190 */
    ssSetInputPortMatrixDimensions(S,  190, 2, 1);
    ssSetInputPortDataType(S, 190, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 190, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 190, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 190, 1); /*direct input signal access*/	
	/* Input Port 191 */
    ssSetInputPortMatrixDimensions(S,  191, 2, 1);
    ssSetInputPortDataType(S, 191, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 191, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 191, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 191, 1); /*direct input signal access*/	
	/* Input Port 192 */
    ssSetInputPortMatrixDimensions(S,  192, 2, 1);
    ssSetInputPortDataType(S, 192, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 192, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 192, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 192, 1); /*direct input signal access*/	
	/* Input Port 193 */
    ssSetInputPortMatrixDimensions(S,  193, 2, 1);
    ssSetInputPortDataType(S, 193, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 193, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 193, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 193, 1); /*direct input signal access*/	
	/* Input Port 194 */
    ssSetInputPortMatrixDimensions(S,  194, 2, 1);
    ssSetInputPortDataType(S, 194, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 194, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 194, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 194, 1); /*direct input signal access*/	
	/* Input Port 195 */
    ssSetInputPortMatrixDimensions(S,  195, 2, 1);
    ssSetInputPortDataType(S, 195, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 195, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 195, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 195, 1); /*direct input signal access*/	
	/* Input Port 196 */
    ssSetInputPortMatrixDimensions(S,  196, 2, 1);
    ssSetInputPortDataType(S, 196, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 196, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 196, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 196, 1); /*direct input signal access*/	
	/* Input Port 197 */
    ssSetInputPortMatrixDimensions(S,  197, 2, 1);
    ssSetInputPortDataType(S, 197, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 197, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 197, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 197, 1); /*direct input signal access*/	
	/* Input Port 198 */
    ssSetInputPortMatrixDimensions(S,  198, 2, 1);
    ssSetInputPortDataType(S, 198, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 198, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 198, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 198, 1); /*direct input signal access*/	
	/* Input Port 199 */
    ssSetInputPortMatrixDimensions(S,  199, 2, 1);
    ssSetInputPortDataType(S, 199, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 199, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 199, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 199, 1); /*direct input signal access*/	
	/* Input Port 200 */
    ssSetInputPortMatrixDimensions(S,  200, 11, 14);
    ssSetInputPortDataType(S, 200, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 200, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 200, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 200, 1); /*direct input signal access*/	
	/* Input Port 201 */
    ssSetInputPortMatrixDimensions(S,  201, 11, 14);
    ssSetInputPortDataType(S, 201, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 201, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 201, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 201, 1); /*direct input signal access*/	
	/* Input Port 202 */
    ssSetInputPortMatrixDimensions(S,  202, 11, 14);
    ssSetInputPortDataType(S, 202, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 202, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 202, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 202, 1); /*direct input signal access*/	
	/* Input Port 203 */
    ssSetInputPortMatrixDimensions(S,  203, 11, 14);
    ssSetInputPortDataType(S, 203, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 203, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 203, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 203, 1); /*direct input signal access*/	
	/* Input Port 204 */
    ssSetInputPortMatrixDimensions(S,  204, 11, 14);
    ssSetInputPortDataType(S, 204, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 204, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 204, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 204, 1); /*direct input signal access*/	
	/* Input Port 205 */
    ssSetInputPortMatrixDimensions(S,  205, 11, 14);
    ssSetInputPortDataType(S, 205, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 205, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 205, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 205, 1); /*direct input signal access*/	
	/* Input Port 206 */
    ssSetInputPortMatrixDimensions(S,  206, 11, 14);
    ssSetInputPortDataType(S, 206, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 206, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 206, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 206, 1); /*direct input signal access*/	
	/* Input Port 207 */
    ssSetInputPortMatrixDimensions(S,  207, 11, 14);
    ssSetInputPortDataType(S, 207, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 207, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 207, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 207, 1); /*direct input signal access*/	
	/* Input Port 208 */
    ssSetInputPortMatrixDimensions(S,  208, 11, 14);
    ssSetInputPortDataType(S, 208, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 208, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 208, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 208, 1); /*direct input signal access*/	
	/* Input Port 209 */
    ssSetInputPortMatrixDimensions(S,  209, 11, 14);
    ssSetInputPortDataType(S, 209, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 209, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 209, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 209, 1); /*direct input signal access*/	
	/* Input Port 210 */
    ssSetInputPortMatrixDimensions(S,  210, 11, 14);
    ssSetInputPortDataType(S, 210, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 210, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 210, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 210, 1); /*direct input signal access*/	
	/* Input Port 211 */
    ssSetInputPortMatrixDimensions(S,  211, 11, 14);
    ssSetInputPortDataType(S, 211, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 211, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 211, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 211, 1); /*direct input signal access*/	
	/* Input Port 212 */
    ssSetInputPortMatrixDimensions(S,  212, 11, 14);
    ssSetInputPortDataType(S, 212, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 212, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 212, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 212, 1); /*direct input signal access*/	
	/* Input Port 213 */
    ssSetInputPortMatrixDimensions(S,  213, 11, 14);
    ssSetInputPortDataType(S, 213, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 213, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 213, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 213, 1); /*direct input signal access*/	
	/* Input Port 214 */
    ssSetInputPortMatrixDimensions(S,  214, 11, 14);
    ssSetInputPortDataType(S, 214, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 214, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 214, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 214, 1); /*direct input signal access*/	
	/* Input Port 215 */
    ssSetInputPortMatrixDimensions(S,  215, 11, 14);
    ssSetInputPortDataType(S, 215, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 215, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 215, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 215, 1); /*direct input signal access*/	
	/* Input Port 216 */
    ssSetInputPortMatrixDimensions(S,  216, 11, 14);
    ssSetInputPortDataType(S, 216, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 216, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 216, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 216, 1); /*direct input signal access*/	
	/* Input Port 217 */
    ssSetInputPortMatrixDimensions(S,  217, 11, 14);
    ssSetInputPortDataType(S, 217, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 217, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 217, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 217, 1); /*direct input signal access*/	
	/* Input Port 218 */
    ssSetInputPortMatrixDimensions(S,  218, 11, 14);
    ssSetInputPortDataType(S, 218, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 218, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 218, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 218, 1); /*direct input signal access*/	
	/* Input Port 219 */
    ssSetInputPortMatrixDimensions(S,  219, 11, 14);
    ssSetInputPortDataType(S, 219, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 219, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 219, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 219, 1); /*direct input signal access*/	
	/* Input Port 220 */
    ssSetInputPortMatrixDimensions(S,  220, 11, 14);
    ssSetInputPortDataType(S, 220, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 220, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 220, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 220, 1); /*direct input signal access*/	
	/* Input Port 221 */
    ssSetInputPortMatrixDimensions(S,  221, 11, 14);
    ssSetInputPortDataType(S, 221, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 221, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 221, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 221, 1); /*direct input signal access*/	
	/* Input Port 222 */
    ssSetInputPortMatrixDimensions(S,  222, 11, 14);
    ssSetInputPortDataType(S, 222, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 222, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 222, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 222, 1); /*direct input signal access*/	
	/* Input Port 223 */
    ssSetInputPortMatrixDimensions(S,  223, 11, 14);
    ssSetInputPortDataType(S, 223, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 223, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 223, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 223, 1); /*direct input signal access*/	
	/* Input Port 224 */
    ssSetInputPortMatrixDimensions(S,  224, 11, 14);
    ssSetInputPortDataType(S, 224, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 224, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 224, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 224, 1); /*direct input signal access*/	
	/* Input Port 225 */
    ssSetInputPortMatrixDimensions(S,  225, 11, 14);
    ssSetInputPortDataType(S, 225, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 225, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 225, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 225, 1); /*direct input signal access*/	
	/* Input Port 226 */
    ssSetInputPortMatrixDimensions(S,  226, 11, 14);
    ssSetInputPortDataType(S, 226, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 226, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 226, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 226, 1); /*direct input signal access*/	
	/* Input Port 227 */
    ssSetInputPortMatrixDimensions(S,  227, 11, 14);
    ssSetInputPortDataType(S, 227, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 227, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 227, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 227, 1); /*direct input signal access*/	
	/* Input Port 228 */
    ssSetInputPortMatrixDimensions(S,  228, 11, 14);
    ssSetInputPortDataType(S, 228, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 228, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 228, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 228, 1); /*direct input signal access*/	
	/* Input Port 229 */
    ssSetInputPortMatrixDimensions(S,  229, 11, 14);
    ssSetInputPortDataType(S, 229, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 229, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 229, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 229, 1); /*direct input signal access*/	
	/* Input Port 230 */
    ssSetInputPortMatrixDimensions(S,  230, 11, 14);
    ssSetInputPortDataType(S, 230, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 230, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 230, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 230, 1); /*direct input signal access*/	
	/* Input Port 231 */
    ssSetInputPortMatrixDimensions(S,  231, 11, 14);
    ssSetInputPortDataType(S, 231, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 231, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 231, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 231, 1); /*direct input signal access*/	
	/* Input Port 232 */
    ssSetInputPortMatrixDimensions(S,  232, 11, 14);
    ssSetInputPortDataType(S, 232, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 232, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 232, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 232, 1); /*direct input signal access*/	
	/* Input Port 233 */
    ssSetInputPortMatrixDimensions(S,  233, 11, 14);
    ssSetInputPortDataType(S, 233, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 233, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 233, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 233, 1); /*direct input signal access*/	
	/* Input Port 234 */
    ssSetInputPortMatrixDimensions(S,  234, 11, 14);
    ssSetInputPortDataType(S, 234, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 234, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 234, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 234, 1); /*direct input signal access*/	
	/* Input Port 235 */
    ssSetInputPortMatrixDimensions(S,  235, 11, 14);
    ssSetInputPortDataType(S, 235, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 235, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 235, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 235, 1); /*direct input signal access*/	
	/* Input Port 236 */
    ssSetInputPortMatrixDimensions(S,  236, 11, 14);
    ssSetInputPortDataType(S, 236, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 236, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 236, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 236, 1); /*direct input signal access*/	
	/* Input Port 237 */
    ssSetInputPortMatrixDimensions(S,  237, 11, 14);
    ssSetInputPortDataType(S, 237, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 237, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 237, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 237, 1); /*direct input signal access*/	
	/* Input Port 238 */
    ssSetInputPortMatrixDimensions(S,  238, 11, 14);
    ssSetInputPortDataType(S, 238, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 238, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 238, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 238, 1); /*direct input signal access*/	
	/* Input Port 239 */
    ssSetInputPortMatrixDimensions(S,  239, 11, 14);
    ssSetInputPortDataType(S, 239, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 239, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 239, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 239, 1); /*direct input signal access*/	
	/* Input Port 240 */
    ssSetInputPortMatrixDimensions(S,  240, 11, 14);
    ssSetInputPortDataType(S, 240, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 240, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 240, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 240, 1); /*direct input signal access*/	
	/* Input Port 241 */
    ssSetInputPortMatrixDimensions(S,  241, 11, 14);
    ssSetInputPortDataType(S, 241, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 241, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 241, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 241, 1); /*direct input signal access*/	
	/* Input Port 242 */
    ssSetInputPortMatrixDimensions(S,  242, 11, 14);
    ssSetInputPortDataType(S, 242, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 242, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 242, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 242, 1); /*direct input signal access*/	
	/* Input Port 243 */
    ssSetInputPortMatrixDimensions(S,  243, 11, 14);
    ssSetInputPortDataType(S, 243, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 243, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 243, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 243, 1); /*direct input signal access*/	
	/* Input Port 244 */
    ssSetInputPortMatrixDimensions(S,  244, 11, 14);
    ssSetInputPortDataType(S, 244, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 244, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 244, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 244, 1); /*direct input signal access*/	
	/* Input Port 245 */
    ssSetInputPortMatrixDimensions(S,  245, 11, 14);
    ssSetInputPortDataType(S, 245, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 245, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 245, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 245, 1); /*direct input signal access*/	
	/* Input Port 246 */
    ssSetInputPortMatrixDimensions(S,  246, 11, 14);
    ssSetInputPortDataType(S, 246, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 246, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 246, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 246, 1); /*direct input signal access*/	
	/* Input Port 247 */
    ssSetInputPortMatrixDimensions(S,  247, 11, 14);
    ssSetInputPortDataType(S, 247, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 247, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 247, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 247, 1); /*direct input signal access*/	
	/* Input Port 248 */
    ssSetInputPortMatrixDimensions(S,  248, 11, 14);
    ssSetInputPortDataType(S, 248, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 248, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 248, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 248, 1); /*direct input signal access*/	
	/* Input Port 249 */
    ssSetInputPortMatrixDimensions(S,  249, 11, 14);
    ssSetInputPortDataType(S, 249, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 249, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 249, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 249, 1); /*direct input signal access*/	
	/* Input Port 250 */
    ssSetInputPortMatrixDimensions(S,  250, 11, 14);
    ssSetInputPortDataType(S, 250, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 250, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 250, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 250, 1); /*direct input signal access*/	
	/* Input Port 251 */
    ssSetInputPortMatrixDimensions(S,  251, 11, 14);
    ssSetInputPortDataType(S, 251, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 251, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 251, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 251, 1); /*direct input signal access*/	
	/* Input Port 252 */
    ssSetInputPortMatrixDimensions(S,  252, 11, 14);
    ssSetInputPortDataType(S, 252, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 252, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 252, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 252, 1); /*direct input signal access*/	
	/* Input Port 253 */
    ssSetInputPortMatrixDimensions(S,  253, 11, 14);
    ssSetInputPortDataType(S, 253, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 253, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 253, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 253, 1); /*direct input signal access*/	
	/* Input Port 254 */
    ssSetInputPortMatrixDimensions(S,  254, 11, 14);
    ssSetInputPortDataType(S, 254, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 254, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 254, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 254, 1); /*direct input signal access*/	
	/* Input Port 255 */
    ssSetInputPortMatrixDimensions(S,  255, 11, 14);
    ssSetInputPortDataType(S, 255, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 255, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 255, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 255, 1); /*direct input signal access*/	
	/* Input Port 256 */
    ssSetInputPortMatrixDimensions(S,  256, 11, 14);
    ssSetInputPortDataType(S, 256, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 256, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 256, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 256, 1); /*direct input signal access*/	
	/* Input Port 257 */
    ssSetInputPortMatrixDimensions(S,  257, 11, 14);
    ssSetInputPortDataType(S, 257, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 257, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 257, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 257, 1); /*direct input signal access*/	
	/* Input Port 258 */
    ssSetInputPortMatrixDimensions(S,  258, 11, 14);
    ssSetInputPortDataType(S, 258, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 258, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 258, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 258, 1); /*direct input signal access*/	
	/* Input Port 259 */
    ssSetInputPortMatrixDimensions(S,  259, 11, 14);
    ssSetInputPortDataType(S, 259, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 259, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 259, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 259, 1); /*direct input signal access*/	
	/* Input Port 260 */
    ssSetInputPortMatrixDimensions(S,  260, 11, 14);
    ssSetInputPortDataType(S, 260, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 260, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 260, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 260, 1); /*direct input signal access*/	
	/* Input Port 261 */
    ssSetInputPortMatrixDimensions(S,  261, 11, 14);
    ssSetInputPortDataType(S, 261, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 261, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 261, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 261, 1); /*direct input signal access*/	
	/* Input Port 262 */
    ssSetInputPortMatrixDimensions(S,  262, 11, 14);
    ssSetInputPortDataType(S, 262, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 262, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 262, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 262, 1); /*direct input signal access*/	
	/* Input Port 263 */
    ssSetInputPortMatrixDimensions(S,  263, 11, 14);
    ssSetInputPortDataType(S, 263, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 263, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 263, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 263, 1); /*direct input signal access*/	
	/* Input Port 264 */
    ssSetInputPortMatrixDimensions(S,  264, 11, 14);
    ssSetInputPortDataType(S, 264, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 264, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 264, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 264, 1); /*direct input signal access*/	
	/* Input Port 265 */
    ssSetInputPortMatrixDimensions(S,  265, 11, 14);
    ssSetInputPortDataType(S, 265, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 265, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 265, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 265, 1); /*direct input signal access*/	
	/* Input Port 266 */
    ssSetInputPortMatrixDimensions(S,  266, 11, 14);
    ssSetInputPortDataType(S, 266, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 266, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 266, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 266, 1); /*direct input signal access*/	
	/* Input Port 267 */
    ssSetInputPortMatrixDimensions(S,  267, 11, 14);
    ssSetInputPortDataType(S, 267, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 267, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 267, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 267, 1); /*direct input signal access*/	
	/* Input Port 268 */
    ssSetInputPortMatrixDimensions(S,  268, 11, 14);
    ssSetInputPortDataType(S, 268, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 268, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 268, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 268, 1); /*direct input signal access*/	
	/* Input Port 269 */
    ssSetInputPortMatrixDimensions(S,  269, 11, 14);
    ssSetInputPortDataType(S, 269, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 269, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 269, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 269, 1); /*direct input signal access*/	
	/* Input Port 270 */
    ssSetInputPortMatrixDimensions(S,  270, 11, 14);
    ssSetInputPortDataType(S, 270, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 270, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 270, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 270, 1); /*direct input signal access*/	
	/* Input Port 271 */
    ssSetInputPortMatrixDimensions(S,  271, 11, 14);
    ssSetInputPortDataType(S, 271, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 271, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 271, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 271, 1); /*direct input signal access*/	
	/* Input Port 272 */
    ssSetInputPortMatrixDimensions(S,  272, 11, 14);
    ssSetInputPortDataType(S, 272, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 272, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 272, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 272, 1); /*direct input signal access*/	
	/* Input Port 273 */
    ssSetInputPortMatrixDimensions(S,  273, 11, 14);
    ssSetInputPortDataType(S, 273, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 273, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 273, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 273, 1); /*direct input signal access*/	
	/* Input Port 274 */
    ssSetInputPortMatrixDimensions(S,  274, 11, 14);
    ssSetInputPortDataType(S, 274, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 274, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 274, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 274, 1); /*direct input signal access*/	
	/* Input Port 275 */
    ssSetInputPortMatrixDimensions(S,  275, 11, 14);
    ssSetInputPortDataType(S, 275, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 275, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 275, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 275, 1); /*direct input signal access*/	
	/* Input Port 276 */
    ssSetInputPortMatrixDimensions(S,  276, 11, 14);
    ssSetInputPortDataType(S, 276, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 276, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 276, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 276, 1); /*direct input signal access*/	
	/* Input Port 277 */
    ssSetInputPortMatrixDimensions(S,  277, 11, 14);
    ssSetInputPortDataType(S, 277, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 277, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 277, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 277, 1); /*direct input signal access*/	
	/* Input Port 278 */
    ssSetInputPortMatrixDimensions(S,  278, 11, 14);
    ssSetInputPortDataType(S, 278, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 278, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 278, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 278, 1); /*direct input signal access*/	
	/* Input Port 279 */
    ssSetInputPortMatrixDimensions(S,  279, 11, 14);
    ssSetInputPortDataType(S, 279, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 279, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 279, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 279, 1); /*direct input signal access*/	
	/* Input Port 280 */
    ssSetInputPortMatrixDimensions(S,  280, 11, 14);
    ssSetInputPortDataType(S, 280, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 280, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 280, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 280, 1); /*direct input signal access*/	
	/* Input Port 281 */
    ssSetInputPortMatrixDimensions(S,  281, 11, 14);
    ssSetInputPortDataType(S, 281, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 281, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 281, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 281, 1); /*direct input signal access*/	
	/* Input Port 282 */
    ssSetInputPortMatrixDimensions(S,  282, 11, 14);
    ssSetInputPortDataType(S, 282, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 282, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 282, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 282, 1); /*direct input signal access*/	
	/* Input Port 283 */
    ssSetInputPortMatrixDimensions(S,  283, 11, 14);
    ssSetInputPortDataType(S, 283, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 283, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 283, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 283, 1); /*direct input signal access*/	
	/* Input Port 284 */
    ssSetInputPortMatrixDimensions(S,  284, 11, 14);
    ssSetInputPortDataType(S, 284, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 284, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 284, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 284, 1); /*direct input signal access*/	
	/* Input Port 285 */
    ssSetInputPortMatrixDimensions(S,  285, 11, 14);
    ssSetInputPortDataType(S, 285, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 285, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 285, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 285, 1); /*direct input signal access*/	
	/* Input Port 286 */
    ssSetInputPortMatrixDimensions(S,  286, 11, 14);
    ssSetInputPortDataType(S, 286, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 286, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 286, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 286, 1); /*direct input signal access*/	
	/* Input Port 287 */
    ssSetInputPortMatrixDimensions(S,  287, 11, 14);
    ssSetInputPortDataType(S, 287, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 287, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 287, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 287, 1); /*direct input signal access*/	
	/* Input Port 288 */
    ssSetInputPortMatrixDimensions(S,  288, 11, 14);
    ssSetInputPortDataType(S, 288, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 288, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 288, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 288, 1); /*direct input signal access*/	
	/* Input Port 289 */
    ssSetInputPortMatrixDimensions(S,  289, 11, 14);
    ssSetInputPortDataType(S, 289, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 289, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 289, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 289, 1); /*direct input signal access*/	
	/* Input Port 290 */
    ssSetInputPortMatrixDimensions(S,  290, 11, 14);
    ssSetInputPortDataType(S, 290, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 290, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 290, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 290, 1); /*direct input signal access*/	
	/* Input Port 291 */
    ssSetInputPortMatrixDimensions(S,  291, 11, 14);
    ssSetInputPortDataType(S, 291, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 291, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 291, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 291, 1); /*direct input signal access*/	
	/* Input Port 292 */
    ssSetInputPortMatrixDimensions(S,  292, 11, 14);
    ssSetInputPortDataType(S, 292, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 292, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 292, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 292, 1); /*direct input signal access*/	
	/* Input Port 293 */
    ssSetInputPortMatrixDimensions(S,  293, 11, 14);
    ssSetInputPortDataType(S, 293, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 293, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 293, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 293, 1); /*direct input signal access*/	
	/* Input Port 294 */
    ssSetInputPortMatrixDimensions(S,  294, 11, 14);
    ssSetInputPortDataType(S, 294, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 294, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 294, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 294, 1); /*direct input signal access*/	
	/* Input Port 295 */
    ssSetInputPortMatrixDimensions(S,  295, 11, 14);
    ssSetInputPortDataType(S, 295, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 295, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 295, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 295, 1); /*direct input signal access*/	
	/* Input Port 296 */
    ssSetInputPortMatrixDimensions(S,  296, 11, 14);
    ssSetInputPortDataType(S, 296, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 296, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 296, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 296, 1); /*direct input signal access*/	
	/* Input Port 297 */
    ssSetInputPortMatrixDimensions(S,  297, 11, 14);
    ssSetInputPortDataType(S, 297, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 297, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 297, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 297, 1); /*direct input signal access*/	
	/* Input Port 298 */
    ssSetInputPortMatrixDimensions(S,  298, 11, 14);
    ssSetInputPortDataType(S, 298, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 298, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 298, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 298, 1); /*direct input signal access*/	
	/* Input Port 299 */
    ssSetInputPortMatrixDimensions(S,  299, 11, 1);
    ssSetInputPortDataType(S, 299, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 299, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 299, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 299, 1); /*direct input signal access*/	
	/* Input Port 300 */
    ssSetInputPortMatrixDimensions(S,  300, 11, 1);
    ssSetInputPortDataType(S, 300, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 300, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 300, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 300, 1); /*direct input signal access*/	
	/* Input Port 301 */
    ssSetInputPortMatrixDimensions(S,  301, 11, 1);
    ssSetInputPortDataType(S, 301, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 301, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 301, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 301, 1); /*direct input signal access*/	
	/* Input Port 302 */
    ssSetInputPortMatrixDimensions(S,  302, 11, 1);
    ssSetInputPortDataType(S, 302, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 302, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 302, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 302, 1); /*direct input signal access*/	
	/* Input Port 303 */
    ssSetInputPortMatrixDimensions(S,  303, 11, 1);
    ssSetInputPortDataType(S, 303, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 303, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 303, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 303, 1); /*direct input signal access*/	
	/* Input Port 304 */
    ssSetInputPortMatrixDimensions(S,  304, 11, 1);
    ssSetInputPortDataType(S, 304, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 304, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 304, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 304, 1); /*direct input signal access*/	
	/* Input Port 305 */
    ssSetInputPortMatrixDimensions(S,  305, 11, 1);
    ssSetInputPortDataType(S, 305, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 305, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 305, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 305, 1); /*direct input signal access*/	
	/* Input Port 306 */
    ssSetInputPortMatrixDimensions(S,  306, 11, 1);
    ssSetInputPortDataType(S, 306, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 306, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 306, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 306, 1); /*direct input signal access*/	
	/* Input Port 307 */
    ssSetInputPortMatrixDimensions(S,  307, 11, 1);
    ssSetInputPortDataType(S, 307, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 307, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 307, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 307, 1); /*direct input signal access*/	
	/* Input Port 308 */
    ssSetInputPortMatrixDimensions(S,  308, 11, 1);
    ssSetInputPortDataType(S, 308, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 308, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 308, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 308, 1); /*direct input signal access*/	
	/* Input Port 309 */
    ssSetInputPortMatrixDimensions(S,  309, 11, 1);
    ssSetInputPortDataType(S, 309, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 309, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 309, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 309, 1); /*direct input signal access*/	
	/* Input Port 310 */
    ssSetInputPortMatrixDimensions(S,  310, 11, 1);
    ssSetInputPortDataType(S, 310, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 310, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 310, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 310, 1); /*direct input signal access*/	
	/* Input Port 311 */
    ssSetInputPortMatrixDimensions(S,  311, 11, 1);
    ssSetInputPortDataType(S, 311, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 311, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 311, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 311, 1); /*direct input signal access*/	
	/* Input Port 312 */
    ssSetInputPortMatrixDimensions(S,  312, 11, 1);
    ssSetInputPortDataType(S, 312, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 312, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 312, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 312, 1); /*direct input signal access*/	
	/* Input Port 313 */
    ssSetInputPortMatrixDimensions(S,  313, 11, 1);
    ssSetInputPortDataType(S, 313, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 313, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 313, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 313, 1); /*direct input signal access*/	
	/* Input Port 314 */
    ssSetInputPortMatrixDimensions(S,  314, 11, 1);
    ssSetInputPortDataType(S, 314, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 314, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 314, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 314, 1); /*direct input signal access*/	
	/* Input Port 315 */
    ssSetInputPortMatrixDimensions(S,  315, 11, 1);
    ssSetInputPortDataType(S, 315, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 315, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 315, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 315, 1); /*direct input signal access*/	
	/* Input Port 316 */
    ssSetInputPortMatrixDimensions(S,  316, 11, 1);
    ssSetInputPortDataType(S, 316, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 316, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 316, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 316, 1); /*direct input signal access*/	
	/* Input Port 317 */
    ssSetInputPortMatrixDimensions(S,  317, 11, 1);
    ssSetInputPortDataType(S, 317, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 317, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 317, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 317, 1); /*direct input signal access*/	
	/* Input Port 318 */
    ssSetInputPortMatrixDimensions(S,  318, 11, 1);
    ssSetInputPortDataType(S, 318, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 318, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 318, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 318, 1); /*direct input signal access*/	
	/* Input Port 319 */
    ssSetInputPortMatrixDimensions(S,  319, 11, 1);
    ssSetInputPortDataType(S, 319, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 319, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 319, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 319, 1); /*direct input signal access*/	
	/* Input Port 320 */
    ssSetInputPortMatrixDimensions(S,  320, 11, 1);
    ssSetInputPortDataType(S, 320, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 320, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 320, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 320, 1); /*direct input signal access*/	
	/* Input Port 321 */
    ssSetInputPortMatrixDimensions(S,  321, 11, 1);
    ssSetInputPortDataType(S, 321, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 321, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 321, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 321, 1); /*direct input signal access*/	
	/* Input Port 322 */
    ssSetInputPortMatrixDimensions(S,  322, 11, 1);
    ssSetInputPortDataType(S, 322, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 322, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 322, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 322, 1); /*direct input signal access*/	
	/* Input Port 323 */
    ssSetInputPortMatrixDimensions(S,  323, 11, 1);
    ssSetInputPortDataType(S, 323, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 323, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 323, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 323, 1); /*direct input signal access*/	
	/* Input Port 324 */
    ssSetInputPortMatrixDimensions(S,  324, 11, 1);
    ssSetInputPortDataType(S, 324, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 324, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 324, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 324, 1); /*direct input signal access*/	
	/* Input Port 325 */
    ssSetInputPortMatrixDimensions(S,  325, 11, 1);
    ssSetInputPortDataType(S, 325, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 325, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 325, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 325, 1); /*direct input signal access*/	
	/* Input Port 326 */
    ssSetInputPortMatrixDimensions(S,  326, 11, 1);
    ssSetInputPortDataType(S, 326, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 326, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 326, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 326, 1); /*direct input signal access*/	
	/* Input Port 327 */
    ssSetInputPortMatrixDimensions(S,  327, 11, 1);
    ssSetInputPortDataType(S, 327, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 327, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 327, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 327, 1); /*direct input signal access*/	
	/* Input Port 328 */
    ssSetInputPortMatrixDimensions(S,  328, 11, 1);
    ssSetInputPortDataType(S, 328, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 328, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 328, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 328, 1); /*direct input signal access*/	
	/* Input Port 329 */
    ssSetInputPortMatrixDimensions(S,  329, 11, 1);
    ssSetInputPortDataType(S, 329, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 329, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 329, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 329, 1); /*direct input signal access*/	
	/* Input Port 330 */
    ssSetInputPortMatrixDimensions(S,  330, 11, 1);
    ssSetInputPortDataType(S, 330, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 330, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 330, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 330, 1); /*direct input signal access*/	
	/* Input Port 331 */
    ssSetInputPortMatrixDimensions(S,  331, 11, 1);
    ssSetInputPortDataType(S, 331, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 331, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 331, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 331, 1); /*direct input signal access*/	
	/* Input Port 332 */
    ssSetInputPortMatrixDimensions(S,  332, 11, 1);
    ssSetInputPortDataType(S, 332, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 332, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 332, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 332, 1); /*direct input signal access*/	
	/* Input Port 333 */
    ssSetInputPortMatrixDimensions(S,  333, 11, 1);
    ssSetInputPortDataType(S, 333, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 333, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 333, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 333, 1); /*direct input signal access*/	
	/* Input Port 334 */
    ssSetInputPortMatrixDimensions(S,  334, 11, 1);
    ssSetInputPortDataType(S, 334, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 334, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 334, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 334, 1); /*direct input signal access*/	
	/* Input Port 335 */
    ssSetInputPortMatrixDimensions(S,  335, 11, 1);
    ssSetInputPortDataType(S, 335, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 335, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 335, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 335, 1); /*direct input signal access*/	
	/* Input Port 336 */
    ssSetInputPortMatrixDimensions(S,  336, 11, 1);
    ssSetInputPortDataType(S, 336, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 336, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 336, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 336, 1); /*direct input signal access*/	
	/* Input Port 337 */
    ssSetInputPortMatrixDimensions(S,  337, 11, 1);
    ssSetInputPortDataType(S, 337, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 337, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 337, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 337, 1); /*direct input signal access*/	
	/* Input Port 338 */
    ssSetInputPortMatrixDimensions(S,  338, 11, 1);
    ssSetInputPortDataType(S, 338, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 338, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 338, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 338, 1); /*direct input signal access*/	
	/* Input Port 339 */
    ssSetInputPortMatrixDimensions(S,  339, 11, 1);
    ssSetInputPortDataType(S, 339, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 339, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 339, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 339, 1); /*direct input signal access*/	
	/* Input Port 340 */
    ssSetInputPortMatrixDimensions(S,  340, 11, 1);
    ssSetInputPortDataType(S, 340, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 340, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 340, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 340, 1); /*direct input signal access*/	
	/* Input Port 341 */
    ssSetInputPortMatrixDimensions(S,  341, 11, 1);
    ssSetInputPortDataType(S, 341, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 341, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 341, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 341, 1); /*direct input signal access*/	
	/* Input Port 342 */
    ssSetInputPortMatrixDimensions(S,  342, 11, 1);
    ssSetInputPortDataType(S, 342, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 342, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 342, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 342, 1); /*direct input signal access*/	
	/* Input Port 343 */
    ssSetInputPortMatrixDimensions(S,  343, 11, 1);
    ssSetInputPortDataType(S, 343, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 343, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 343, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 343, 1); /*direct input signal access*/	
	/* Input Port 344 */
    ssSetInputPortMatrixDimensions(S,  344, 11, 1);
    ssSetInputPortDataType(S, 344, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 344, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 344, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 344, 1); /*direct input signal access*/	
	/* Input Port 345 */
    ssSetInputPortMatrixDimensions(S,  345, 11, 1);
    ssSetInputPortDataType(S, 345, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 345, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 345, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 345, 1); /*direct input signal access*/	
	/* Input Port 346 */
    ssSetInputPortMatrixDimensions(S,  346, 11, 1);
    ssSetInputPortDataType(S, 346, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 346, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 346, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 346, 1); /*direct input signal access*/	
	/* Input Port 347 */
    ssSetInputPortMatrixDimensions(S,  347, 11, 1);
    ssSetInputPortDataType(S, 347, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 347, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 347, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 347, 1); /*direct input signal access*/	
	/* Input Port 348 */
    ssSetInputPortMatrixDimensions(S,  348, 11, 1);
    ssSetInputPortDataType(S, 348, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 348, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 348, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 348, 1); /*direct input signal access*/ 


	/* initialize output ports - there are 1 in total */
    if (!ssSetNumOutputPorts(S, 1)) return;    
		
	/* Output Port 0 */
    ssSetOutputPortMatrixDimensions(S,  0, 700, 1);
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
	const real_T *H21 = (const real_T*) ssGetInputPortSignal(S,20);
	const real_T *H22 = (const real_T*) ssGetInputPortSignal(S,21);
	const real_T *H23 = (const real_T*) ssGetInputPortSignal(S,22);
	const real_T *H24 = (const real_T*) ssGetInputPortSignal(S,23);
	const real_T *H25 = (const real_T*) ssGetInputPortSignal(S,24);
	const real_T *H26 = (const real_T*) ssGetInputPortSignal(S,25);
	const real_T *H27 = (const real_T*) ssGetInputPortSignal(S,26);
	const real_T *H28 = (const real_T*) ssGetInputPortSignal(S,27);
	const real_T *H29 = (const real_T*) ssGetInputPortSignal(S,28);
	const real_T *H30 = (const real_T*) ssGetInputPortSignal(S,29);
	const real_T *H31 = (const real_T*) ssGetInputPortSignal(S,30);
	const real_T *H32 = (const real_T*) ssGetInputPortSignal(S,31);
	const real_T *H33 = (const real_T*) ssGetInputPortSignal(S,32);
	const real_T *H34 = (const real_T*) ssGetInputPortSignal(S,33);
	const real_T *H35 = (const real_T*) ssGetInputPortSignal(S,34);
	const real_T *H36 = (const real_T*) ssGetInputPortSignal(S,35);
	const real_T *H37 = (const real_T*) ssGetInputPortSignal(S,36);
	const real_T *H38 = (const real_T*) ssGetInputPortSignal(S,37);
	const real_T *H39 = (const real_T*) ssGetInputPortSignal(S,38);
	const real_T *H40 = (const real_T*) ssGetInputPortSignal(S,39);
	const real_T *H41 = (const real_T*) ssGetInputPortSignal(S,40);
	const real_T *H42 = (const real_T*) ssGetInputPortSignal(S,41);
	const real_T *H43 = (const real_T*) ssGetInputPortSignal(S,42);
	const real_T *H44 = (const real_T*) ssGetInputPortSignal(S,43);
	const real_T *H45 = (const real_T*) ssGetInputPortSignal(S,44);
	const real_T *H46 = (const real_T*) ssGetInputPortSignal(S,45);
	const real_T *H47 = (const real_T*) ssGetInputPortSignal(S,46);
	const real_T *H48 = (const real_T*) ssGetInputPortSignal(S,47);
	const real_T *H49 = (const real_T*) ssGetInputPortSignal(S,48);
	const real_T *H50 = (const real_T*) ssGetInputPortSignal(S,49);
	const real_T *f01 = (const real_T*) ssGetInputPortSignal(S,50);
	const real_T *f02 = (const real_T*) ssGetInputPortSignal(S,51);
	const real_T *f03 = (const real_T*) ssGetInputPortSignal(S,52);
	const real_T *f04 = (const real_T*) ssGetInputPortSignal(S,53);
	const real_T *f05 = (const real_T*) ssGetInputPortSignal(S,54);
	const real_T *f06 = (const real_T*) ssGetInputPortSignal(S,55);
	const real_T *f07 = (const real_T*) ssGetInputPortSignal(S,56);
	const real_T *f08 = (const real_T*) ssGetInputPortSignal(S,57);
	const real_T *f09 = (const real_T*) ssGetInputPortSignal(S,58);
	const real_T *f10 = (const real_T*) ssGetInputPortSignal(S,59);
	const real_T *f11 = (const real_T*) ssGetInputPortSignal(S,60);
	const real_T *f12 = (const real_T*) ssGetInputPortSignal(S,61);
	const real_T *f13 = (const real_T*) ssGetInputPortSignal(S,62);
	const real_T *f14 = (const real_T*) ssGetInputPortSignal(S,63);
	const real_T *f15 = (const real_T*) ssGetInputPortSignal(S,64);
	const real_T *f16 = (const real_T*) ssGetInputPortSignal(S,65);
	const real_T *f17 = (const real_T*) ssGetInputPortSignal(S,66);
	const real_T *f18 = (const real_T*) ssGetInputPortSignal(S,67);
	const real_T *f19 = (const real_T*) ssGetInputPortSignal(S,68);
	const real_T *f20 = (const real_T*) ssGetInputPortSignal(S,69);
	const real_T *f21 = (const real_T*) ssGetInputPortSignal(S,70);
	const real_T *f22 = (const real_T*) ssGetInputPortSignal(S,71);
	const real_T *f23 = (const real_T*) ssGetInputPortSignal(S,72);
	const real_T *f24 = (const real_T*) ssGetInputPortSignal(S,73);
	const real_T *f25 = (const real_T*) ssGetInputPortSignal(S,74);
	const real_T *f26 = (const real_T*) ssGetInputPortSignal(S,75);
	const real_T *f27 = (const real_T*) ssGetInputPortSignal(S,76);
	const real_T *f28 = (const real_T*) ssGetInputPortSignal(S,77);
	const real_T *f29 = (const real_T*) ssGetInputPortSignal(S,78);
	const real_T *f30 = (const real_T*) ssGetInputPortSignal(S,79);
	const real_T *f31 = (const real_T*) ssGetInputPortSignal(S,80);
	const real_T *f32 = (const real_T*) ssGetInputPortSignal(S,81);
	const real_T *f33 = (const real_T*) ssGetInputPortSignal(S,82);
	const real_T *f34 = (const real_T*) ssGetInputPortSignal(S,83);
	const real_T *f35 = (const real_T*) ssGetInputPortSignal(S,84);
	const real_T *f36 = (const real_T*) ssGetInputPortSignal(S,85);
	const real_T *f37 = (const real_T*) ssGetInputPortSignal(S,86);
	const real_T *f38 = (const real_T*) ssGetInputPortSignal(S,87);
	const real_T *f39 = (const real_T*) ssGetInputPortSignal(S,88);
	const real_T *f40 = (const real_T*) ssGetInputPortSignal(S,89);
	const real_T *f41 = (const real_T*) ssGetInputPortSignal(S,90);
	const real_T *f42 = (const real_T*) ssGetInputPortSignal(S,91);
	const real_T *f43 = (const real_T*) ssGetInputPortSignal(S,92);
	const real_T *f44 = (const real_T*) ssGetInputPortSignal(S,93);
	const real_T *f45 = (const real_T*) ssGetInputPortSignal(S,94);
	const real_T *f46 = (const real_T*) ssGetInputPortSignal(S,95);
	const real_T *f47 = (const real_T*) ssGetInputPortSignal(S,96);
	const real_T *f48 = (const real_T*) ssGetInputPortSignal(S,97);
	const real_T *f49 = (const real_T*) ssGetInputPortSignal(S,98);
	const real_T *f50 = (const real_T*) ssGetInputPortSignal(S,99);
	const real_T *A01 = (const real_T*) ssGetInputPortSignal(S,100);
	const real_T *A02 = (const real_T*) ssGetInputPortSignal(S,101);
	const real_T *A03 = (const real_T*) ssGetInputPortSignal(S,102);
	const real_T *A04 = (const real_T*) ssGetInputPortSignal(S,103);
	const real_T *A05 = (const real_T*) ssGetInputPortSignal(S,104);
	const real_T *A06 = (const real_T*) ssGetInputPortSignal(S,105);
	const real_T *A07 = (const real_T*) ssGetInputPortSignal(S,106);
	const real_T *A08 = (const real_T*) ssGetInputPortSignal(S,107);
	const real_T *A09 = (const real_T*) ssGetInputPortSignal(S,108);
	const real_T *A10 = (const real_T*) ssGetInputPortSignal(S,109);
	const real_T *A11 = (const real_T*) ssGetInputPortSignal(S,110);
	const real_T *A12 = (const real_T*) ssGetInputPortSignal(S,111);
	const real_T *A13 = (const real_T*) ssGetInputPortSignal(S,112);
	const real_T *A14 = (const real_T*) ssGetInputPortSignal(S,113);
	const real_T *A15 = (const real_T*) ssGetInputPortSignal(S,114);
	const real_T *A16 = (const real_T*) ssGetInputPortSignal(S,115);
	const real_T *A17 = (const real_T*) ssGetInputPortSignal(S,116);
	const real_T *A18 = (const real_T*) ssGetInputPortSignal(S,117);
	const real_T *A19 = (const real_T*) ssGetInputPortSignal(S,118);
	const real_T *A20 = (const real_T*) ssGetInputPortSignal(S,119);
	const real_T *A21 = (const real_T*) ssGetInputPortSignal(S,120);
	const real_T *A22 = (const real_T*) ssGetInputPortSignal(S,121);
	const real_T *A23 = (const real_T*) ssGetInputPortSignal(S,122);
	const real_T *A24 = (const real_T*) ssGetInputPortSignal(S,123);
	const real_T *A25 = (const real_T*) ssGetInputPortSignal(S,124);
	const real_T *A26 = (const real_T*) ssGetInputPortSignal(S,125);
	const real_T *A27 = (const real_T*) ssGetInputPortSignal(S,126);
	const real_T *A28 = (const real_T*) ssGetInputPortSignal(S,127);
	const real_T *A29 = (const real_T*) ssGetInputPortSignal(S,128);
	const real_T *A30 = (const real_T*) ssGetInputPortSignal(S,129);
	const real_T *A31 = (const real_T*) ssGetInputPortSignal(S,130);
	const real_T *A32 = (const real_T*) ssGetInputPortSignal(S,131);
	const real_T *A33 = (const real_T*) ssGetInputPortSignal(S,132);
	const real_T *A34 = (const real_T*) ssGetInputPortSignal(S,133);
	const real_T *A35 = (const real_T*) ssGetInputPortSignal(S,134);
	const real_T *A36 = (const real_T*) ssGetInputPortSignal(S,135);
	const real_T *A37 = (const real_T*) ssGetInputPortSignal(S,136);
	const real_T *A38 = (const real_T*) ssGetInputPortSignal(S,137);
	const real_T *A39 = (const real_T*) ssGetInputPortSignal(S,138);
	const real_T *A40 = (const real_T*) ssGetInputPortSignal(S,139);
	const real_T *A41 = (const real_T*) ssGetInputPortSignal(S,140);
	const real_T *A42 = (const real_T*) ssGetInputPortSignal(S,141);
	const real_T *A43 = (const real_T*) ssGetInputPortSignal(S,142);
	const real_T *A44 = (const real_T*) ssGetInputPortSignal(S,143);
	const real_T *A45 = (const real_T*) ssGetInputPortSignal(S,144);
	const real_T *A46 = (const real_T*) ssGetInputPortSignal(S,145);
	const real_T *A47 = (const real_T*) ssGetInputPortSignal(S,146);
	const real_T *A48 = (const real_T*) ssGetInputPortSignal(S,147);
	const real_T *A49 = (const real_T*) ssGetInputPortSignal(S,148);
	const real_T *A50 = (const real_T*) ssGetInputPortSignal(S,149);
	const real_T *b01 = (const real_T*) ssGetInputPortSignal(S,150);
	const real_T *b02 = (const real_T*) ssGetInputPortSignal(S,151);
	const real_T *b03 = (const real_T*) ssGetInputPortSignal(S,152);
	const real_T *b04 = (const real_T*) ssGetInputPortSignal(S,153);
	const real_T *b05 = (const real_T*) ssGetInputPortSignal(S,154);
	const real_T *b06 = (const real_T*) ssGetInputPortSignal(S,155);
	const real_T *b07 = (const real_T*) ssGetInputPortSignal(S,156);
	const real_T *b08 = (const real_T*) ssGetInputPortSignal(S,157);
	const real_T *b09 = (const real_T*) ssGetInputPortSignal(S,158);
	const real_T *b10 = (const real_T*) ssGetInputPortSignal(S,159);
	const real_T *b11 = (const real_T*) ssGetInputPortSignal(S,160);
	const real_T *b12 = (const real_T*) ssGetInputPortSignal(S,161);
	const real_T *b13 = (const real_T*) ssGetInputPortSignal(S,162);
	const real_T *b14 = (const real_T*) ssGetInputPortSignal(S,163);
	const real_T *b15 = (const real_T*) ssGetInputPortSignal(S,164);
	const real_T *b16 = (const real_T*) ssGetInputPortSignal(S,165);
	const real_T *b17 = (const real_T*) ssGetInputPortSignal(S,166);
	const real_T *b18 = (const real_T*) ssGetInputPortSignal(S,167);
	const real_T *b19 = (const real_T*) ssGetInputPortSignal(S,168);
	const real_T *b20 = (const real_T*) ssGetInputPortSignal(S,169);
	const real_T *b21 = (const real_T*) ssGetInputPortSignal(S,170);
	const real_T *b22 = (const real_T*) ssGetInputPortSignal(S,171);
	const real_T *b23 = (const real_T*) ssGetInputPortSignal(S,172);
	const real_T *b24 = (const real_T*) ssGetInputPortSignal(S,173);
	const real_T *b25 = (const real_T*) ssGetInputPortSignal(S,174);
	const real_T *b26 = (const real_T*) ssGetInputPortSignal(S,175);
	const real_T *b27 = (const real_T*) ssGetInputPortSignal(S,176);
	const real_T *b28 = (const real_T*) ssGetInputPortSignal(S,177);
	const real_T *b29 = (const real_T*) ssGetInputPortSignal(S,178);
	const real_T *b30 = (const real_T*) ssGetInputPortSignal(S,179);
	const real_T *b31 = (const real_T*) ssGetInputPortSignal(S,180);
	const real_T *b32 = (const real_T*) ssGetInputPortSignal(S,181);
	const real_T *b33 = (const real_T*) ssGetInputPortSignal(S,182);
	const real_T *b34 = (const real_T*) ssGetInputPortSignal(S,183);
	const real_T *b35 = (const real_T*) ssGetInputPortSignal(S,184);
	const real_T *b36 = (const real_T*) ssGetInputPortSignal(S,185);
	const real_T *b37 = (const real_T*) ssGetInputPortSignal(S,186);
	const real_T *b38 = (const real_T*) ssGetInputPortSignal(S,187);
	const real_T *b39 = (const real_T*) ssGetInputPortSignal(S,188);
	const real_T *b40 = (const real_T*) ssGetInputPortSignal(S,189);
	const real_T *b41 = (const real_T*) ssGetInputPortSignal(S,190);
	const real_T *b42 = (const real_T*) ssGetInputPortSignal(S,191);
	const real_T *b43 = (const real_T*) ssGetInputPortSignal(S,192);
	const real_T *b44 = (const real_T*) ssGetInputPortSignal(S,193);
	const real_T *b45 = (const real_T*) ssGetInputPortSignal(S,194);
	const real_T *b46 = (const real_T*) ssGetInputPortSignal(S,195);
	const real_T *b47 = (const real_T*) ssGetInputPortSignal(S,196);
	const real_T *b48 = (const real_T*) ssGetInputPortSignal(S,197);
	const real_T *b49 = (const real_T*) ssGetInputPortSignal(S,198);
	const real_T *b50 = (const real_T*) ssGetInputPortSignal(S,199);
	const real_T *C01 = (const real_T*) ssGetInputPortSignal(S,200);
	const real_T *C02 = (const real_T*) ssGetInputPortSignal(S,201);
	const real_T *C03 = (const real_T*) ssGetInputPortSignal(S,202);
	const real_T *C04 = (const real_T*) ssGetInputPortSignal(S,203);
	const real_T *C05 = (const real_T*) ssGetInputPortSignal(S,204);
	const real_T *C06 = (const real_T*) ssGetInputPortSignal(S,205);
	const real_T *C07 = (const real_T*) ssGetInputPortSignal(S,206);
	const real_T *C08 = (const real_T*) ssGetInputPortSignal(S,207);
	const real_T *C09 = (const real_T*) ssGetInputPortSignal(S,208);
	const real_T *C10 = (const real_T*) ssGetInputPortSignal(S,209);
	const real_T *C11 = (const real_T*) ssGetInputPortSignal(S,210);
	const real_T *C12 = (const real_T*) ssGetInputPortSignal(S,211);
	const real_T *C13 = (const real_T*) ssGetInputPortSignal(S,212);
	const real_T *C14 = (const real_T*) ssGetInputPortSignal(S,213);
	const real_T *C15 = (const real_T*) ssGetInputPortSignal(S,214);
	const real_T *C16 = (const real_T*) ssGetInputPortSignal(S,215);
	const real_T *C17 = (const real_T*) ssGetInputPortSignal(S,216);
	const real_T *C18 = (const real_T*) ssGetInputPortSignal(S,217);
	const real_T *C19 = (const real_T*) ssGetInputPortSignal(S,218);
	const real_T *C20 = (const real_T*) ssGetInputPortSignal(S,219);
	const real_T *C21 = (const real_T*) ssGetInputPortSignal(S,220);
	const real_T *C22 = (const real_T*) ssGetInputPortSignal(S,221);
	const real_T *C23 = (const real_T*) ssGetInputPortSignal(S,222);
	const real_T *C24 = (const real_T*) ssGetInputPortSignal(S,223);
	const real_T *C25 = (const real_T*) ssGetInputPortSignal(S,224);
	const real_T *C26 = (const real_T*) ssGetInputPortSignal(S,225);
	const real_T *C27 = (const real_T*) ssGetInputPortSignal(S,226);
	const real_T *C28 = (const real_T*) ssGetInputPortSignal(S,227);
	const real_T *C29 = (const real_T*) ssGetInputPortSignal(S,228);
	const real_T *C30 = (const real_T*) ssGetInputPortSignal(S,229);
	const real_T *C31 = (const real_T*) ssGetInputPortSignal(S,230);
	const real_T *C32 = (const real_T*) ssGetInputPortSignal(S,231);
	const real_T *C33 = (const real_T*) ssGetInputPortSignal(S,232);
	const real_T *C34 = (const real_T*) ssGetInputPortSignal(S,233);
	const real_T *C35 = (const real_T*) ssGetInputPortSignal(S,234);
	const real_T *C36 = (const real_T*) ssGetInputPortSignal(S,235);
	const real_T *C37 = (const real_T*) ssGetInputPortSignal(S,236);
	const real_T *C38 = (const real_T*) ssGetInputPortSignal(S,237);
	const real_T *C39 = (const real_T*) ssGetInputPortSignal(S,238);
	const real_T *C40 = (const real_T*) ssGetInputPortSignal(S,239);
	const real_T *C41 = (const real_T*) ssGetInputPortSignal(S,240);
	const real_T *C42 = (const real_T*) ssGetInputPortSignal(S,241);
	const real_T *C43 = (const real_T*) ssGetInputPortSignal(S,242);
	const real_T *C44 = (const real_T*) ssGetInputPortSignal(S,243);
	const real_T *C45 = (const real_T*) ssGetInputPortSignal(S,244);
	const real_T *C46 = (const real_T*) ssGetInputPortSignal(S,245);
	const real_T *C47 = (const real_T*) ssGetInputPortSignal(S,246);
	const real_T *C48 = (const real_T*) ssGetInputPortSignal(S,247);
	const real_T *C49 = (const real_T*) ssGetInputPortSignal(S,248);
	const real_T *D01 = (const real_T*) ssGetInputPortSignal(S,249);
	const real_T *D02 = (const real_T*) ssGetInputPortSignal(S,250);
	const real_T *D03 = (const real_T*) ssGetInputPortSignal(S,251);
	const real_T *D04 = (const real_T*) ssGetInputPortSignal(S,252);
	const real_T *D05 = (const real_T*) ssGetInputPortSignal(S,253);
	const real_T *D06 = (const real_T*) ssGetInputPortSignal(S,254);
	const real_T *D07 = (const real_T*) ssGetInputPortSignal(S,255);
	const real_T *D08 = (const real_T*) ssGetInputPortSignal(S,256);
	const real_T *D09 = (const real_T*) ssGetInputPortSignal(S,257);
	const real_T *D10 = (const real_T*) ssGetInputPortSignal(S,258);
	const real_T *D11 = (const real_T*) ssGetInputPortSignal(S,259);
	const real_T *D12 = (const real_T*) ssGetInputPortSignal(S,260);
	const real_T *D13 = (const real_T*) ssGetInputPortSignal(S,261);
	const real_T *D14 = (const real_T*) ssGetInputPortSignal(S,262);
	const real_T *D15 = (const real_T*) ssGetInputPortSignal(S,263);
	const real_T *D16 = (const real_T*) ssGetInputPortSignal(S,264);
	const real_T *D17 = (const real_T*) ssGetInputPortSignal(S,265);
	const real_T *D18 = (const real_T*) ssGetInputPortSignal(S,266);
	const real_T *D19 = (const real_T*) ssGetInputPortSignal(S,267);
	const real_T *D20 = (const real_T*) ssGetInputPortSignal(S,268);
	const real_T *D21 = (const real_T*) ssGetInputPortSignal(S,269);
	const real_T *D22 = (const real_T*) ssGetInputPortSignal(S,270);
	const real_T *D23 = (const real_T*) ssGetInputPortSignal(S,271);
	const real_T *D24 = (const real_T*) ssGetInputPortSignal(S,272);
	const real_T *D25 = (const real_T*) ssGetInputPortSignal(S,273);
	const real_T *D26 = (const real_T*) ssGetInputPortSignal(S,274);
	const real_T *D27 = (const real_T*) ssGetInputPortSignal(S,275);
	const real_T *D28 = (const real_T*) ssGetInputPortSignal(S,276);
	const real_T *D29 = (const real_T*) ssGetInputPortSignal(S,277);
	const real_T *D30 = (const real_T*) ssGetInputPortSignal(S,278);
	const real_T *D31 = (const real_T*) ssGetInputPortSignal(S,279);
	const real_T *D32 = (const real_T*) ssGetInputPortSignal(S,280);
	const real_T *D33 = (const real_T*) ssGetInputPortSignal(S,281);
	const real_T *D34 = (const real_T*) ssGetInputPortSignal(S,282);
	const real_T *D35 = (const real_T*) ssGetInputPortSignal(S,283);
	const real_T *D36 = (const real_T*) ssGetInputPortSignal(S,284);
	const real_T *D37 = (const real_T*) ssGetInputPortSignal(S,285);
	const real_T *D38 = (const real_T*) ssGetInputPortSignal(S,286);
	const real_T *D39 = (const real_T*) ssGetInputPortSignal(S,287);
	const real_T *D40 = (const real_T*) ssGetInputPortSignal(S,288);
	const real_T *D41 = (const real_T*) ssGetInputPortSignal(S,289);
	const real_T *D42 = (const real_T*) ssGetInputPortSignal(S,290);
	const real_T *D43 = (const real_T*) ssGetInputPortSignal(S,291);
	const real_T *D44 = (const real_T*) ssGetInputPortSignal(S,292);
	const real_T *D45 = (const real_T*) ssGetInputPortSignal(S,293);
	const real_T *D46 = (const real_T*) ssGetInputPortSignal(S,294);
	const real_T *D47 = (const real_T*) ssGetInputPortSignal(S,295);
	const real_T *D48 = (const real_T*) ssGetInputPortSignal(S,296);
	const real_T *D49 = (const real_T*) ssGetInputPortSignal(S,297);
	const real_T *D50 = (const real_T*) ssGetInputPortSignal(S,298);
	const real_T *c02 = (const real_T*) ssGetInputPortSignal(S,299);
	const real_T *c03 = (const real_T*) ssGetInputPortSignal(S,300);
	const real_T *c04 = (const real_T*) ssGetInputPortSignal(S,301);
	const real_T *c05 = (const real_T*) ssGetInputPortSignal(S,302);
	const real_T *c06 = (const real_T*) ssGetInputPortSignal(S,303);
	const real_T *c07 = (const real_T*) ssGetInputPortSignal(S,304);
	const real_T *c08 = (const real_T*) ssGetInputPortSignal(S,305);
	const real_T *c09 = (const real_T*) ssGetInputPortSignal(S,306);
	const real_T *c10 = (const real_T*) ssGetInputPortSignal(S,307);
	const real_T *c11 = (const real_T*) ssGetInputPortSignal(S,308);
	const real_T *c12 = (const real_T*) ssGetInputPortSignal(S,309);
	const real_T *c13 = (const real_T*) ssGetInputPortSignal(S,310);
	const real_T *c14 = (const real_T*) ssGetInputPortSignal(S,311);
	const real_T *c15 = (const real_T*) ssGetInputPortSignal(S,312);
	const real_T *c16 = (const real_T*) ssGetInputPortSignal(S,313);
	const real_T *c17 = (const real_T*) ssGetInputPortSignal(S,314);
	const real_T *c18 = (const real_T*) ssGetInputPortSignal(S,315);
	const real_T *c19 = (const real_T*) ssGetInputPortSignal(S,316);
	const real_T *c20 = (const real_T*) ssGetInputPortSignal(S,317);
	const real_T *c21 = (const real_T*) ssGetInputPortSignal(S,318);
	const real_T *c22 = (const real_T*) ssGetInputPortSignal(S,319);
	const real_T *c23 = (const real_T*) ssGetInputPortSignal(S,320);
	const real_T *c24 = (const real_T*) ssGetInputPortSignal(S,321);
	const real_T *c25 = (const real_T*) ssGetInputPortSignal(S,322);
	const real_T *c26 = (const real_T*) ssGetInputPortSignal(S,323);
	const real_T *c27 = (const real_T*) ssGetInputPortSignal(S,324);
	const real_T *c28 = (const real_T*) ssGetInputPortSignal(S,325);
	const real_T *c29 = (const real_T*) ssGetInputPortSignal(S,326);
	const real_T *c30 = (const real_T*) ssGetInputPortSignal(S,327);
	const real_T *c31 = (const real_T*) ssGetInputPortSignal(S,328);
	const real_T *c32 = (const real_T*) ssGetInputPortSignal(S,329);
	const real_T *c33 = (const real_T*) ssGetInputPortSignal(S,330);
	const real_T *c34 = (const real_T*) ssGetInputPortSignal(S,331);
	const real_T *c35 = (const real_T*) ssGetInputPortSignal(S,332);
	const real_T *c36 = (const real_T*) ssGetInputPortSignal(S,333);
	const real_T *c37 = (const real_T*) ssGetInputPortSignal(S,334);
	const real_T *c38 = (const real_T*) ssGetInputPortSignal(S,335);
	const real_T *c39 = (const real_T*) ssGetInputPortSignal(S,336);
	const real_T *c40 = (const real_T*) ssGetInputPortSignal(S,337);
	const real_T *c41 = (const real_T*) ssGetInputPortSignal(S,338);
	const real_T *c42 = (const real_T*) ssGetInputPortSignal(S,339);
	const real_T *c43 = (const real_T*) ssGetInputPortSignal(S,340);
	const real_T *c44 = (const real_T*) ssGetInputPortSignal(S,341);
	const real_T *c45 = (const real_T*) ssGetInputPortSignal(S,342);
	const real_T *c46 = (const real_T*) ssGetInputPortSignal(S,343);
	const real_T *c47 = (const real_T*) ssGetInputPortSignal(S,344);
	const real_T *c48 = (const real_T*) ssGetInputPortSignal(S,345);
	const real_T *c49 = (const real_T*) ssGetInputPortSignal(S,346);
	const real_T *c50 = (const real_T*) ssGetInputPortSignal(S,347);
	const real_T *minusA_times_x0 = (const real_T*) ssGetInputPortSignal(S,348);
	
    real_T *u0 = (real_T*) ssGetOutputPortSignal(S,0);
	
	

	/* Solver data */
	static mpcc_params params;
	static mpcc_output output;
	static mpcc_info info;	
	solver_int32_default exitflag;

	/* Extra NMPC data */
	

	/* Copy inputs */
	for( i=0; i<196; i++)
	{ 
		params.H01[i] = (double) H01[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H02[i] = (double) H02[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H03[i] = (double) H03[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H04[i] = (double) H04[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H05[i] = (double) H05[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H06[i] = (double) H06[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H07[i] = (double) H07[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H08[i] = (double) H08[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H09[i] = (double) H09[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H10[i] = (double) H10[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H11[i] = (double) H11[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H12[i] = (double) H12[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H13[i] = (double) H13[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H14[i] = (double) H14[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H15[i] = (double) H15[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H16[i] = (double) H16[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H17[i] = (double) H17[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H18[i] = (double) H18[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H19[i] = (double) H19[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H20[i] = (double) H20[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H21[i] = (double) H21[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H22[i] = (double) H22[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H23[i] = (double) H23[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H24[i] = (double) H24[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H25[i] = (double) H25[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H26[i] = (double) H26[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H27[i] = (double) H27[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H28[i] = (double) H28[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H29[i] = (double) H29[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H30[i] = (double) H30[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H31[i] = (double) H31[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H32[i] = (double) H32[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H33[i] = (double) H33[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H34[i] = (double) H34[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H35[i] = (double) H35[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H36[i] = (double) H36[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H37[i] = (double) H37[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H38[i] = (double) H38[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H39[i] = (double) H39[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H40[i] = (double) H40[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H41[i] = (double) H41[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H42[i] = (double) H42[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H43[i] = (double) H43[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H44[i] = (double) H44[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H45[i] = (double) H45[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H46[i] = (double) H46[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H47[i] = (double) H47[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H48[i] = (double) H48[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H49[i] = (double) H49[i]; 
	}

	for( i=0; i<196; i++)
	{ 
		params.H50[i] = (double) H50[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f01[i] = (double) f01[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f02[i] = (double) f02[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f03[i] = (double) f03[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f04[i] = (double) f04[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f05[i] = (double) f05[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f06[i] = (double) f06[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f07[i] = (double) f07[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f08[i] = (double) f08[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f09[i] = (double) f09[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f10[i] = (double) f10[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f11[i] = (double) f11[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f12[i] = (double) f12[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f13[i] = (double) f13[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f14[i] = (double) f14[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f15[i] = (double) f15[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f16[i] = (double) f16[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f17[i] = (double) f17[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f18[i] = (double) f18[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f19[i] = (double) f19[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f20[i] = (double) f20[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f21[i] = (double) f21[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f22[i] = (double) f22[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f23[i] = (double) f23[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f24[i] = (double) f24[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f25[i] = (double) f25[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f26[i] = (double) f26[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f27[i] = (double) f27[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f28[i] = (double) f28[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f29[i] = (double) f29[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f30[i] = (double) f30[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f31[i] = (double) f31[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f32[i] = (double) f32[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f33[i] = (double) f33[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f34[i] = (double) f34[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f35[i] = (double) f35[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f36[i] = (double) f36[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f37[i] = (double) f37[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f38[i] = (double) f38[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f39[i] = (double) f39[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f40[i] = (double) f40[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f41[i] = (double) f41[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f42[i] = (double) f42[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f43[i] = (double) f43[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f44[i] = (double) f44[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f45[i] = (double) f45[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f46[i] = (double) f46[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f47[i] = (double) f47[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f48[i] = (double) f48[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f49[i] = (double) f49[i]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f50[i] = (double) f50[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A01[i] = (double) A01[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A02[i] = (double) A02[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A03[i] = (double) A03[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A04[i] = (double) A04[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A05[i] = (double) A05[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A06[i] = (double) A06[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A07[i] = (double) A07[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A08[i] = (double) A08[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A09[i] = (double) A09[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A10[i] = (double) A10[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A11[i] = (double) A11[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A12[i] = (double) A12[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A13[i] = (double) A13[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A14[i] = (double) A14[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A15[i] = (double) A15[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A16[i] = (double) A16[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A17[i] = (double) A17[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A18[i] = (double) A18[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A19[i] = (double) A19[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A20[i] = (double) A20[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A21[i] = (double) A21[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A22[i] = (double) A22[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A23[i] = (double) A23[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A24[i] = (double) A24[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A25[i] = (double) A25[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A26[i] = (double) A26[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A27[i] = (double) A27[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A28[i] = (double) A28[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A29[i] = (double) A29[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A30[i] = (double) A30[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A31[i] = (double) A31[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A32[i] = (double) A32[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A33[i] = (double) A33[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A34[i] = (double) A34[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A35[i] = (double) A35[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A36[i] = (double) A36[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A37[i] = (double) A37[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A38[i] = (double) A38[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A39[i] = (double) A39[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A40[i] = (double) A40[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A41[i] = (double) A41[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A42[i] = (double) A42[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A43[i] = (double) A43[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A44[i] = (double) A44[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A45[i] = (double) A45[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A46[i] = (double) A46[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A47[i] = (double) A47[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A48[i] = (double) A48[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A49[i] = (double) A49[i]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A50[i] = (double) A50[i]; 
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

	for( i=0; i<2; i++)
	{ 
		params.b21[i] = (double) b21[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b22[i] = (double) b22[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b23[i] = (double) b23[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b24[i] = (double) b24[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b25[i] = (double) b25[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b26[i] = (double) b26[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b27[i] = (double) b27[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b28[i] = (double) b28[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b29[i] = (double) b29[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b30[i] = (double) b30[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b31[i] = (double) b31[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b32[i] = (double) b32[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b33[i] = (double) b33[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b34[i] = (double) b34[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b35[i] = (double) b35[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b36[i] = (double) b36[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b37[i] = (double) b37[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b38[i] = (double) b38[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b39[i] = (double) b39[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b40[i] = (double) b40[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b41[i] = (double) b41[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b42[i] = (double) b42[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b43[i] = (double) b43[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b44[i] = (double) b44[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b45[i] = (double) b45[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b46[i] = (double) b46[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b47[i] = (double) b47[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b48[i] = (double) b48[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b49[i] = (double) b49[i]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b50[i] = (double) b50[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C01[i] = (double) C01[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C02[i] = (double) C02[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C03[i] = (double) C03[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C04[i] = (double) C04[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C05[i] = (double) C05[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C06[i] = (double) C06[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C07[i] = (double) C07[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C08[i] = (double) C08[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C09[i] = (double) C09[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C10[i] = (double) C10[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C11[i] = (double) C11[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C12[i] = (double) C12[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C13[i] = (double) C13[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C14[i] = (double) C14[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C15[i] = (double) C15[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C16[i] = (double) C16[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C17[i] = (double) C17[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C18[i] = (double) C18[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C19[i] = (double) C19[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C20[i] = (double) C20[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C21[i] = (double) C21[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C22[i] = (double) C22[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C23[i] = (double) C23[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C24[i] = (double) C24[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C25[i] = (double) C25[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C26[i] = (double) C26[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C27[i] = (double) C27[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C28[i] = (double) C28[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C29[i] = (double) C29[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C30[i] = (double) C30[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C31[i] = (double) C31[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C32[i] = (double) C32[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C33[i] = (double) C33[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C34[i] = (double) C34[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C35[i] = (double) C35[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C36[i] = (double) C36[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C37[i] = (double) C37[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C38[i] = (double) C38[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C39[i] = (double) C39[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C40[i] = (double) C40[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C41[i] = (double) C41[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C42[i] = (double) C42[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C43[i] = (double) C43[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C44[i] = (double) C44[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C45[i] = (double) C45[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C46[i] = (double) C46[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C47[i] = (double) C47[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C48[i] = (double) C48[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C49[i] = (double) C49[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D01[i] = (double) D01[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D02[i] = (double) D02[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D03[i] = (double) D03[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D04[i] = (double) D04[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D05[i] = (double) D05[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D06[i] = (double) D06[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D07[i] = (double) D07[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D08[i] = (double) D08[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D09[i] = (double) D09[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D10[i] = (double) D10[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D11[i] = (double) D11[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D12[i] = (double) D12[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D13[i] = (double) D13[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D14[i] = (double) D14[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D15[i] = (double) D15[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D16[i] = (double) D16[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D17[i] = (double) D17[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D18[i] = (double) D18[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D19[i] = (double) D19[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D20[i] = (double) D20[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D21[i] = (double) D21[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D22[i] = (double) D22[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D23[i] = (double) D23[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D24[i] = (double) D24[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D25[i] = (double) D25[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D26[i] = (double) D26[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D27[i] = (double) D27[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D28[i] = (double) D28[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D29[i] = (double) D29[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D30[i] = (double) D30[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D31[i] = (double) D31[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D32[i] = (double) D32[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D33[i] = (double) D33[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D34[i] = (double) D34[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D35[i] = (double) D35[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D36[i] = (double) D36[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D37[i] = (double) D37[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D38[i] = (double) D38[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D39[i] = (double) D39[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D40[i] = (double) D40[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D41[i] = (double) D41[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D42[i] = (double) D42[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D43[i] = (double) D43[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D44[i] = (double) D44[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D45[i] = (double) D45[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D46[i] = (double) D46[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D47[i] = (double) D47[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D48[i] = (double) D48[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D49[i] = (double) D49[i]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D50[i] = (double) D50[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c02[i] = (double) c02[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c03[i] = (double) c03[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c04[i] = (double) c04[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c05[i] = (double) c05[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c06[i] = (double) c06[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c07[i] = (double) c07[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c08[i] = (double) c08[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c09[i] = (double) c09[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c10[i] = (double) c10[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c11[i] = (double) c11[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c12[i] = (double) c12[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c13[i] = (double) c13[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c14[i] = (double) c14[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c15[i] = (double) c15[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c16[i] = (double) c16[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c17[i] = (double) c17[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c18[i] = (double) c18[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c19[i] = (double) c19[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c20[i] = (double) c20[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c21[i] = (double) c21[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c22[i] = (double) c22[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c23[i] = (double) c23[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c24[i] = (double) c24[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c25[i] = (double) c25[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c26[i] = (double) c26[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c27[i] = (double) c27[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c28[i] = (double) c28[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c29[i] = (double) c29[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c30[i] = (double) c30[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c31[i] = (double) c31[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c32[i] = (double) c32[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c33[i] = (double) c33[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c34[i] = (double) c34[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c35[i] = (double) c35[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c36[i] = (double) c36[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c37[i] = (double) c37[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c38[i] = (double) c38[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c39[i] = (double) c39[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c40[i] = (double) c40[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c41[i] = (double) c41[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c42[i] = (double) c42[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c43[i] = (double) c43[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c44[i] = (double) c44[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c45[i] = (double) c45[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c46[i] = (double) c46[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c47[i] = (double) c47[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c48[i] = (double) c48[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c49[i] = (double) c49[i]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c50[i] = (double) c50[i]; 
	}

	for( i=0; i<11; i++)
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
	for( i=0; i<700; i++)
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



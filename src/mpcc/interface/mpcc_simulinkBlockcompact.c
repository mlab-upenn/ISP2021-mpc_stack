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
#define S_FUNCTION_NAME mpcc_simulinkBlockcompact

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

	/* initialize input ports - there are 7 in total */
    if (!ssSetNumInputPorts(S, 7)) return;
    	
	/* Input Port 0 */
    ssSetInputPortMatrixDimensions(S,  0, 11, 50);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 11, 700);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 11, 686);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 14, 700);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 3, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/	
	/* Input Port 4 */
    ssSetInputPortMatrixDimensions(S,  4, 14, 50);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 4, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 4, 1); /*direct input signal access*/	
	/* Input Port 5 */
    ssSetInputPortMatrixDimensions(S,  5, 2, 50);
    ssSetInputPortDataType(S, 5, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 5, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 5, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 5, 1); /*direct input signal access*/	
	/* Input Port 6 */
    ssSetInputPortMatrixDimensions(S,  6, 2, 700);
    ssSetInputPortDataType(S, 6, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 6, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 6, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 6, 1); /*direct input signal access*/ 


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
	const real_T *c = (const real_T*) ssGetInputPortSignal(S,0);
	const real_T *D = (const real_T*) ssGetInputPortSignal(S,1);
	const real_T *C = (const real_T*) ssGetInputPortSignal(S,2);
	const real_T *H = (const real_T*) ssGetInputPortSignal(S,3);
	const real_T *f = (const real_T*) ssGetInputPortSignal(S,4);
	const real_T *b = (const real_T*) ssGetInputPortSignal(S,5);
	const real_T *A = (const real_T*) ssGetInputPortSignal(S,6);
	
    real_T *outputs = (real_T*) ssGetOutputPortSignal(S,0);
	
	

	/* Solver data */
	static mpcc_params params;
	static mpcc_output output;
	static mpcc_info info;	
	solver_int32_default exitflag;

	/* Extra NMPC data */
	

	/* Copy inputs */
	for( i=0; i<196; i++)
	{ 
		params.H01[i] = (double) H[i]; 
	}

	j=196; 
	for( i=0; i<196; i++)
	{ 
		params.H02[i] = (double) H[j++]; 
	}

	j=392; 
	for( i=0; i<196; i++)
	{ 
		params.H03[i] = (double) H[j++]; 
	}

	j=588; 
	for( i=0; i<196; i++)
	{ 
		params.H04[i] = (double) H[j++]; 
	}

	j=784; 
	for( i=0; i<196; i++)
	{ 
		params.H05[i] = (double) H[j++]; 
	}

	j=980; 
	for( i=0; i<196; i++)
	{ 
		params.H06[i] = (double) H[j++]; 
	}

	j=1176; 
	for( i=0; i<196; i++)
	{ 
		params.H07[i] = (double) H[j++]; 
	}

	j=1372; 
	for( i=0; i<196; i++)
	{ 
		params.H08[i] = (double) H[j++]; 
	}

	j=1568; 
	for( i=0; i<196; i++)
	{ 
		params.H09[i] = (double) H[j++]; 
	}

	j=1764; 
	for( i=0; i<196; i++)
	{ 
		params.H10[i] = (double) H[j++]; 
	}

	j=1960; 
	for( i=0; i<196; i++)
	{ 
		params.H11[i] = (double) H[j++]; 
	}

	j=2156; 
	for( i=0; i<196; i++)
	{ 
		params.H12[i] = (double) H[j++]; 
	}

	j=2352; 
	for( i=0; i<196; i++)
	{ 
		params.H13[i] = (double) H[j++]; 
	}

	j=2548; 
	for( i=0; i<196; i++)
	{ 
		params.H14[i] = (double) H[j++]; 
	}

	j=2744; 
	for( i=0; i<196; i++)
	{ 
		params.H15[i] = (double) H[j++]; 
	}

	j=2940; 
	for( i=0; i<196; i++)
	{ 
		params.H16[i] = (double) H[j++]; 
	}

	j=3136; 
	for( i=0; i<196; i++)
	{ 
		params.H17[i] = (double) H[j++]; 
	}

	j=3332; 
	for( i=0; i<196; i++)
	{ 
		params.H18[i] = (double) H[j++]; 
	}

	j=3528; 
	for( i=0; i<196; i++)
	{ 
		params.H19[i] = (double) H[j++]; 
	}

	j=3724; 
	for( i=0; i<196; i++)
	{ 
		params.H20[i] = (double) H[j++]; 
	}

	j=3920; 
	for( i=0; i<196; i++)
	{ 
		params.H21[i] = (double) H[j++]; 
	}

	j=4116; 
	for( i=0; i<196; i++)
	{ 
		params.H22[i] = (double) H[j++]; 
	}

	j=4312; 
	for( i=0; i<196; i++)
	{ 
		params.H23[i] = (double) H[j++]; 
	}

	j=4508; 
	for( i=0; i<196; i++)
	{ 
		params.H24[i] = (double) H[j++]; 
	}

	j=4704; 
	for( i=0; i<196; i++)
	{ 
		params.H25[i] = (double) H[j++]; 
	}

	j=4900; 
	for( i=0; i<196; i++)
	{ 
		params.H26[i] = (double) H[j++]; 
	}

	j=5096; 
	for( i=0; i<196; i++)
	{ 
		params.H27[i] = (double) H[j++]; 
	}

	j=5292; 
	for( i=0; i<196; i++)
	{ 
		params.H28[i] = (double) H[j++]; 
	}

	j=5488; 
	for( i=0; i<196; i++)
	{ 
		params.H29[i] = (double) H[j++]; 
	}

	j=5684; 
	for( i=0; i<196; i++)
	{ 
		params.H30[i] = (double) H[j++]; 
	}

	j=5880; 
	for( i=0; i<196; i++)
	{ 
		params.H31[i] = (double) H[j++]; 
	}

	j=6076; 
	for( i=0; i<196; i++)
	{ 
		params.H32[i] = (double) H[j++]; 
	}

	j=6272; 
	for( i=0; i<196; i++)
	{ 
		params.H33[i] = (double) H[j++]; 
	}

	j=6468; 
	for( i=0; i<196; i++)
	{ 
		params.H34[i] = (double) H[j++]; 
	}

	j=6664; 
	for( i=0; i<196; i++)
	{ 
		params.H35[i] = (double) H[j++]; 
	}

	j=6860; 
	for( i=0; i<196; i++)
	{ 
		params.H36[i] = (double) H[j++]; 
	}

	j=7056; 
	for( i=0; i<196; i++)
	{ 
		params.H37[i] = (double) H[j++]; 
	}

	j=7252; 
	for( i=0; i<196; i++)
	{ 
		params.H38[i] = (double) H[j++]; 
	}

	j=7448; 
	for( i=0; i<196; i++)
	{ 
		params.H39[i] = (double) H[j++]; 
	}

	j=7644; 
	for( i=0; i<196; i++)
	{ 
		params.H40[i] = (double) H[j++]; 
	}

	j=7840; 
	for( i=0; i<196; i++)
	{ 
		params.H41[i] = (double) H[j++]; 
	}

	j=8036; 
	for( i=0; i<196; i++)
	{ 
		params.H42[i] = (double) H[j++]; 
	}

	j=8232; 
	for( i=0; i<196; i++)
	{ 
		params.H43[i] = (double) H[j++]; 
	}

	j=8428; 
	for( i=0; i<196; i++)
	{ 
		params.H44[i] = (double) H[j++]; 
	}

	j=8624; 
	for( i=0; i<196; i++)
	{ 
		params.H45[i] = (double) H[j++]; 
	}

	j=8820; 
	for( i=0; i<196; i++)
	{ 
		params.H46[i] = (double) H[j++]; 
	}

	j=9016; 
	for( i=0; i<196; i++)
	{ 
		params.H47[i] = (double) H[j++]; 
	}

	j=9212; 
	for( i=0; i<196; i++)
	{ 
		params.H48[i] = (double) H[j++]; 
	}

	j=9408; 
	for( i=0; i<196; i++)
	{ 
		params.H49[i] = (double) H[j++]; 
	}

	j=9604; 
	for( i=0; i<196; i++)
	{ 
		params.H50[i] = (double) H[j++]; 
	}

	for( i=0; i<14; i++)
	{ 
		params.f01[i] = (double) f[i]; 
	}

	j=14; 
	for( i=0; i<14; i++)
	{ 
		params.f02[i] = (double) f[j++]; 
	}

	j=28; 
	for( i=0; i<14; i++)
	{ 
		params.f03[i] = (double) f[j++]; 
	}

	j=42; 
	for( i=0; i<14; i++)
	{ 
		params.f04[i] = (double) f[j++]; 
	}

	j=56; 
	for( i=0; i<14; i++)
	{ 
		params.f05[i] = (double) f[j++]; 
	}

	j=70; 
	for( i=0; i<14; i++)
	{ 
		params.f06[i] = (double) f[j++]; 
	}

	j=84; 
	for( i=0; i<14; i++)
	{ 
		params.f07[i] = (double) f[j++]; 
	}

	j=98; 
	for( i=0; i<14; i++)
	{ 
		params.f08[i] = (double) f[j++]; 
	}

	j=112; 
	for( i=0; i<14; i++)
	{ 
		params.f09[i] = (double) f[j++]; 
	}

	j=126; 
	for( i=0; i<14; i++)
	{ 
		params.f10[i] = (double) f[j++]; 
	}

	j=140; 
	for( i=0; i<14; i++)
	{ 
		params.f11[i] = (double) f[j++]; 
	}

	j=154; 
	for( i=0; i<14; i++)
	{ 
		params.f12[i] = (double) f[j++]; 
	}

	j=168; 
	for( i=0; i<14; i++)
	{ 
		params.f13[i] = (double) f[j++]; 
	}

	j=182; 
	for( i=0; i<14; i++)
	{ 
		params.f14[i] = (double) f[j++]; 
	}

	j=196; 
	for( i=0; i<14; i++)
	{ 
		params.f15[i] = (double) f[j++]; 
	}

	j=210; 
	for( i=0; i<14; i++)
	{ 
		params.f16[i] = (double) f[j++]; 
	}

	j=224; 
	for( i=0; i<14; i++)
	{ 
		params.f17[i] = (double) f[j++]; 
	}

	j=238; 
	for( i=0; i<14; i++)
	{ 
		params.f18[i] = (double) f[j++]; 
	}

	j=252; 
	for( i=0; i<14; i++)
	{ 
		params.f19[i] = (double) f[j++]; 
	}

	j=266; 
	for( i=0; i<14; i++)
	{ 
		params.f20[i] = (double) f[j++]; 
	}

	j=280; 
	for( i=0; i<14; i++)
	{ 
		params.f21[i] = (double) f[j++]; 
	}

	j=294; 
	for( i=0; i<14; i++)
	{ 
		params.f22[i] = (double) f[j++]; 
	}

	j=308; 
	for( i=0; i<14; i++)
	{ 
		params.f23[i] = (double) f[j++]; 
	}

	j=322; 
	for( i=0; i<14; i++)
	{ 
		params.f24[i] = (double) f[j++]; 
	}

	j=336; 
	for( i=0; i<14; i++)
	{ 
		params.f25[i] = (double) f[j++]; 
	}

	j=350; 
	for( i=0; i<14; i++)
	{ 
		params.f26[i] = (double) f[j++]; 
	}

	j=364; 
	for( i=0; i<14; i++)
	{ 
		params.f27[i] = (double) f[j++]; 
	}

	j=378; 
	for( i=0; i<14; i++)
	{ 
		params.f28[i] = (double) f[j++]; 
	}

	j=392; 
	for( i=0; i<14; i++)
	{ 
		params.f29[i] = (double) f[j++]; 
	}

	j=406; 
	for( i=0; i<14; i++)
	{ 
		params.f30[i] = (double) f[j++]; 
	}

	j=420; 
	for( i=0; i<14; i++)
	{ 
		params.f31[i] = (double) f[j++]; 
	}

	j=434; 
	for( i=0; i<14; i++)
	{ 
		params.f32[i] = (double) f[j++]; 
	}

	j=448; 
	for( i=0; i<14; i++)
	{ 
		params.f33[i] = (double) f[j++]; 
	}

	j=462; 
	for( i=0; i<14; i++)
	{ 
		params.f34[i] = (double) f[j++]; 
	}

	j=476; 
	for( i=0; i<14; i++)
	{ 
		params.f35[i] = (double) f[j++]; 
	}

	j=490; 
	for( i=0; i<14; i++)
	{ 
		params.f36[i] = (double) f[j++]; 
	}

	j=504; 
	for( i=0; i<14; i++)
	{ 
		params.f37[i] = (double) f[j++]; 
	}

	j=518; 
	for( i=0; i<14; i++)
	{ 
		params.f38[i] = (double) f[j++]; 
	}

	j=532; 
	for( i=0; i<14; i++)
	{ 
		params.f39[i] = (double) f[j++]; 
	}

	j=546; 
	for( i=0; i<14; i++)
	{ 
		params.f40[i] = (double) f[j++]; 
	}

	j=560; 
	for( i=0; i<14; i++)
	{ 
		params.f41[i] = (double) f[j++]; 
	}

	j=574; 
	for( i=0; i<14; i++)
	{ 
		params.f42[i] = (double) f[j++]; 
	}

	j=588; 
	for( i=0; i<14; i++)
	{ 
		params.f43[i] = (double) f[j++]; 
	}

	j=602; 
	for( i=0; i<14; i++)
	{ 
		params.f44[i] = (double) f[j++]; 
	}

	j=616; 
	for( i=0; i<14; i++)
	{ 
		params.f45[i] = (double) f[j++]; 
	}

	j=630; 
	for( i=0; i<14; i++)
	{ 
		params.f46[i] = (double) f[j++]; 
	}

	j=644; 
	for( i=0; i<14; i++)
	{ 
		params.f47[i] = (double) f[j++]; 
	}

	j=658; 
	for( i=0; i<14; i++)
	{ 
		params.f48[i] = (double) f[j++]; 
	}

	j=672; 
	for( i=0; i<14; i++)
	{ 
		params.f49[i] = (double) f[j++]; 
	}

	j=686; 
	for( i=0; i<14; i++)
	{ 
		params.f50[i] = (double) f[j++]; 
	}

	for( i=0; i<28; i++)
	{ 
		params.A01[i] = (double) A[i]; 
	}

	j=28; 
	for( i=0; i<28; i++)
	{ 
		params.A02[i] = (double) A[j++]; 
	}

	j=56; 
	for( i=0; i<28; i++)
	{ 
		params.A03[i] = (double) A[j++]; 
	}

	j=84; 
	for( i=0; i<28; i++)
	{ 
		params.A04[i] = (double) A[j++]; 
	}

	j=112; 
	for( i=0; i<28; i++)
	{ 
		params.A05[i] = (double) A[j++]; 
	}

	j=140; 
	for( i=0; i<28; i++)
	{ 
		params.A06[i] = (double) A[j++]; 
	}

	j=168; 
	for( i=0; i<28; i++)
	{ 
		params.A07[i] = (double) A[j++]; 
	}

	j=196; 
	for( i=0; i<28; i++)
	{ 
		params.A08[i] = (double) A[j++]; 
	}

	j=224; 
	for( i=0; i<28; i++)
	{ 
		params.A09[i] = (double) A[j++]; 
	}

	j=252; 
	for( i=0; i<28; i++)
	{ 
		params.A10[i] = (double) A[j++]; 
	}

	j=280; 
	for( i=0; i<28; i++)
	{ 
		params.A11[i] = (double) A[j++]; 
	}

	j=308; 
	for( i=0; i<28; i++)
	{ 
		params.A12[i] = (double) A[j++]; 
	}

	j=336; 
	for( i=0; i<28; i++)
	{ 
		params.A13[i] = (double) A[j++]; 
	}

	j=364; 
	for( i=0; i<28; i++)
	{ 
		params.A14[i] = (double) A[j++]; 
	}

	j=392; 
	for( i=0; i<28; i++)
	{ 
		params.A15[i] = (double) A[j++]; 
	}

	j=420; 
	for( i=0; i<28; i++)
	{ 
		params.A16[i] = (double) A[j++]; 
	}

	j=448; 
	for( i=0; i<28; i++)
	{ 
		params.A17[i] = (double) A[j++]; 
	}

	j=476; 
	for( i=0; i<28; i++)
	{ 
		params.A18[i] = (double) A[j++]; 
	}

	j=504; 
	for( i=0; i<28; i++)
	{ 
		params.A19[i] = (double) A[j++]; 
	}

	j=532; 
	for( i=0; i<28; i++)
	{ 
		params.A20[i] = (double) A[j++]; 
	}

	j=560; 
	for( i=0; i<28; i++)
	{ 
		params.A21[i] = (double) A[j++]; 
	}

	j=588; 
	for( i=0; i<28; i++)
	{ 
		params.A22[i] = (double) A[j++]; 
	}

	j=616; 
	for( i=0; i<28; i++)
	{ 
		params.A23[i] = (double) A[j++]; 
	}

	j=644; 
	for( i=0; i<28; i++)
	{ 
		params.A24[i] = (double) A[j++]; 
	}

	j=672; 
	for( i=0; i<28; i++)
	{ 
		params.A25[i] = (double) A[j++]; 
	}

	j=700; 
	for( i=0; i<28; i++)
	{ 
		params.A26[i] = (double) A[j++]; 
	}

	j=728; 
	for( i=0; i<28; i++)
	{ 
		params.A27[i] = (double) A[j++]; 
	}

	j=756; 
	for( i=0; i<28; i++)
	{ 
		params.A28[i] = (double) A[j++]; 
	}

	j=784; 
	for( i=0; i<28; i++)
	{ 
		params.A29[i] = (double) A[j++]; 
	}

	j=812; 
	for( i=0; i<28; i++)
	{ 
		params.A30[i] = (double) A[j++]; 
	}

	j=840; 
	for( i=0; i<28; i++)
	{ 
		params.A31[i] = (double) A[j++]; 
	}

	j=868; 
	for( i=0; i<28; i++)
	{ 
		params.A32[i] = (double) A[j++]; 
	}

	j=896; 
	for( i=0; i<28; i++)
	{ 
		params.A33[i] = (double) A[j++]; 
	}

	j=924; 
	for( i=0; i<28; i++)
	{ 
		params.A34[i] = (double) A[j++]; 
	}

	j=952; 
	for( i=0; i<28; i++)
	{ 
		params.A35[i] = (double) A[j++]; 
	}

	j=980; 
	for( i=0; i<28; i++)
	{ 
		params.A36[i] = (double) A[j++]; 
	}

	j=1008; 
	for( i=0; i<28; i++)
	{ 
		params.A37[i] = (double) A[j++]; 
	}

	j=1036; 
	for( i=0; i<28; i++)
	{ 
		params.A38[i] = (double) A[j++]; 
	}

	j=1064; 
	for( i=0; i<28; i++)
	{ 
		params.A39[i] = (double) A[j++]; 
	}

	j=1092; 
	for( i=0; i<28; i++)
	{ 
		params.A40[i] = (double) A[j++]; 
	}

	j=1120; 
	for( i=0; i<28; i++)
	{ 
		params.A41[i] = (double) A[j++]; 
	}

	j=1148; 
	for( i=0; i<28; i++)
	{ 
		params.A42[i] = (double) A[j++]; 
	}

	j=1176; 
	for( i=0; i<28; i++)
	{ 
		params.A43[i] = (double) A[j++]; 
	}

	j=1204; 
	for( i=0; i<28; i++)
	{ 
		params.A44[i] = (double) A[j++]; 
	}

	j=1232; 
	for( i=0; i<28; i++)
	{ 
		params.A45[i] = (double) A[j++]; 
	}

	j=1260; 
	for( i=0; i<28; i++)
	{ 
		params.A46[i] = (double) A[j++]; 
	}

	j=1288; 
	for( i=0; i<28; i++)
	{ 
		params.A47[i] = (double) A[j++]; 
	}

	j=1316; 
	for( i=0; i<28; i++)
	{ 
		params.A48[i] = (double) A[j++]; 
	}

	j=1344; 
	for( i=0; i<28; i++)
	{ 
		params.A49[i] = (double) A[j++]; 
	}

	j=1372; 
	for( i=0; i<28; i++)
	{ 
		params.A50[i] = (double) A[j++]; 
	}

	for( i=0; i<2; i++)
	{ 
		params.b01[i] = (double) b[i]; 
	}

	j=2; 
	for( i=0; i<2; i++)
	{ 
		params.b02[i] = (double) b[j++]; 
	}

	j=4; 
	for( i=0; i<2; i++)
	{ 
		params.b03[i] = (double) b[j++]; 
	}

	j=6; 
	for( i=0; i<2; i++)
	{ 
		params.b04[i] = (double) b[j++]; 
	}

	j=8; 
	for( i=0; i<2; i++)
	{ 
		params.b05[i] = (double) b[j++]; 
	}

	j=10; 
	for( i=0; i<2; i++)
	{ 
		params.b06[i] = (double) b[j++]; 
	}

	j=12; 
	for( i=0; i<2; i++)
	{ 
		params.b07[i] = (double) b[j++]; 
	}

	j=14; 
	for( i=0; i<2; i++)
	{ 
		params.b08[i] = (double) b[j++]; 
	}

	j=16; 
	for( i=0; i<2; i++)
	{ 
		params.b09[i] = (double) b[j++]; 
	}

	j=18; 
	for( i=0; i<2; i++)
	{ 
		params.b10[i] = (double) b[j++]; 
	}

	j=20; 
	for( i=0; i<2; i++)
	{ 
		params.b11[i] = (double) b[j++]; 
	}

	j=22; 
	for( i=0; i<2; i++)
	{ 
		params.b12[i] = (double) b[j++]; 
	}

	j=24; 
	for( i=0; i<2; i++)
	{ 
		params.b13[i] = (double) b[j++]; 
	}

	j=26; 
	for( i=0; i<2; i++)
	{ 
		params.b14[i] = (double) b[j++]; 
	}

	j=28; 
	for( i=0; i<2; i++)
	{ 
		params.b15[i] = (double) b[j++]; 
	}

	j=30; 
	for( i=0; i<2; i++)
	{ 
		params.b16[i] = (double) b[j++]; 
	}

	j=32; 
	for( i=0; i<2; i++)
	{ 
		params.b17[i] = (double) b[j++]; 
	}

	j=34; 
	for( i=0; i<2; i++)
	{ 
		params.b18[i] = (double) b[j++]; 
	}

	j=36; 
	for( i=0; i<2; i++)
	{ 
		params.b19[i] = (double) b[j++]; 
	}

	j=38; 
	for( i=0; i<2; i++)
	{ 
		params.b20[i] = (double) b[j++]; 
	}

	j=40; 
	for( i=0; i<2; i++)
	{ 
		params.b21[i] = (double) b[j++]; 
	}

	j=42; 
	for( i=0; i<2; i++)
	{ 
		params.b22[i] = (double) b[j++]; 
	}

	j=44; 
	for( i=0; i<2; i++)
	{ 
		params.b23[i] = (double) b[j++]; 
	}

	j=46; 
	for( i=0; i<2; i++)
	{ 
		params.b24[i] = (double) b[j++]; 
	}

	j=48; 
	for( i=0; i<2; i++)
	{ 
		params.b25[i] = (double) b[j++]; 
	}

	j=50; 
	for( i=0; i<2; i++)
	{ 
		params.b26[i] = (double) b[j++]; 
	}

	j=52; 
	for( i=0; i<2; i++)
	{ 
		params.b27[i] = (double) b[j++]; 
	}

	j=54; 
	for( i=0; i<2; i++)
	{ 
		params.b28[i] = (double) b[j++]; 
	}

	j=56; 
	for( i=0; i<2; i++)
	{ 
		params.b29[i] = (double) b[j++]; 
	}

	j=58; 
	for( i=0; i<2; i++)
	{ 
		params.b30[i] = (double) b[j++]; 
	}

	j=60; 
	for( i=0; i<2; i++)
	{ 
		params.b31[i] = (double) b[j++]; 
	}

	j=62; 
	for( i=0; i<2; i++)
	{ 
		params.b32[i] = (double) b[j++]; 
	}

	j=64; 
	for( i=0; i<2; i++)
	{ 
		params.b33[i] = (double) b[j++]; 
	}

	j=66; 
	for( i=0; i<2; i++)
	{ 
		params.b34[i] = (double) b[j++]; 
	}

	j=68; 
	for( i=0; i<2; i++)
	{ 
		params.b35[i] = (double) b[j++]; 
	}

	j=70; 
	for( i=0; i<2; i++)
	{ 
		params.b36[i] = (double) b[j++]; 
	}

	j=72; 
	for( i=0; i<2; i++)
	{ 
		params.b37[i] = (double) b[j++]; 
	}

	j=74; 
	for( i=0; i<2; i++)
	{ 
		params.b38[i] = (double) b[j++]; 
	}

	j=76; 
	for( i=0; i<2; i++)
	{ 
		params.b39[i] = (double) b[j++]; 
	}

	j=78; 
	for( i=0; i<2; i++)
	{ 
		params.b40[i] = (double) b[j++]; 
	}

	j=80; 
	for( i=0; i<2; i++)
	{ 
		params.b41[i] = (double) b[j++]; 
	}

	j=82; 
	for( i=0; i<2; i++)
	{ 
		params.b42[i] = (double) b[j++]; 
	}

	j=84; 
	for( i=0; i<2; i++)
	{ 
		params.b43[i] = (double) b[j++]; 
	}

	j=86; 
	for( i=0; i<2; i++)
	{ 
		params.b44[i] = (double) b[j++]; 
	}

	j=88; 
	for( i=0; i<2; i++)
	{ 
		params.b45[i] = (double) b[j++]; 
	}

	j=90; 
	for( i=0; i<2; i++)
	{ 
		params.b46[i] = (double) b[j++]; 
	}

	j=92; 
	for( i=0; i<2; i++)
	{ 
		params.b47[i] = (double) b[j++]; 
	}

	j=94; 
	for( i=0; i<2; i++)
	{ 
		params.b48[i] = (double) b[j++]; 
	}

	j=96; 
	for( i=0; i<2; i++)
	{ 
		params.b49[i] = (double) b[j++]; 
	}

	j=98; 
	for( i=0; i<2; i++)
	{ 
		params.b50[i] = (double) b[j++]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.C01[i] = (double) C[i]; 
	}

	j=154; 
	for( i=0; i<154; i++)
	{ 
		params.C02[i] = (double) C[j++]; 
	}

	j=308; 
	for( i=0; i<154; i++)
	{ 
		params.C03[i] = (double) C[j++]; 
	}

	j=462; 
	for( i=0; i<154; i++)
	{ 
		params.C04[i] = (double) C[j++]; 
	}

	j=616; 
	for( i=0; i<154; i++)
	{ 
		params.C05[i] = (double) C[j++]; 
	}

	j=770; 
	for( i=0; i<154; i++)
	{ 
		params.C06[i] = (double) C[j++]; 
	}

	j=924; 
	for( i=0; i<154; i++)
	{ 
		params.C07[i] = (double) C[j++]; 
	}

	j=1078; 
	for( i=0; i<154; i++)
	{ 
		params.C08[i] = (double) C[j++]; 
	}

	j=1232; 
	for( i=0; i<154; i++)
	{ 
		params.C09[i] = (double) C[j++]; 
	}

	j=1386; 
	for( i=0; i<154; i++)
	{ 
		params.C10[i] = (double) C[j++]; 
	}

	j=1540; 
	for( i=0; i<154; i++)
	{ 
		params.C11[i] = (double) C[j++]; 
	}

	j=1694; 
	for( i=0; i<154; i++)
	{ 
		params.C12[i] = (double) C[j++]; 
	}

	j=1848; 
	for( i=0; i<154; i++)
	{ 
		params.C13[i] = (double) C[j++]; 
	}

	j=2002; 
	for( i=0; i<154; i++)
	{ 
		params.C14[i] = (double) C[j++]; 
	}

	j=2156; 
	for( i=0; i<154; i++)
	{ 
		params.C15[i] = (double) C[j++]; 
	}

	j=2310; 
	for( i=0; i<154; i++)
	{ 
		params.C16[i] = (double) C[j++]; 
	}

	j=2464; 
	for( i=0; i<154; i++)
	{ 
		params.C17[i] = (double) C[j++]; 
	}

	j=2618; 
	for( i=0; i<154; i++)
	{ 
		params.C18[i] = (double) C[j++]; 
	}

	j=2772; 
	for( i=0; i<154; i++)
	{ 
		params.C19[i] = (double) C[j++]; 
	}

	j=2926; 
	for( i=0; i<154; i++)
	{ 
		params.C20[i] = (double) C[j++]; 
	}

	j=3080; 
	for( i=0; i<154; i++)
	{ 
		params.C21[i] = (double) C[j++]; 
	}

	j=3234; 
	for( i=0; i<154; i++)
	{ 
		params.C22[i] = (double) C[j++]; 
	}

	j=3388; 
	for( i=0; i<154; i++)
	{ 
		params.C23[i] = (double) C[j++]; 
	}

	j=3542; 
	for( i=0; i<154; i++)
	{ 
		params.C24[i] = (double) C[j++]; 
	}

	j=3696; 
	for( i=0; i<154; i++)
	{ 
		params.C25[i] = (double) C[j++]; 
	}

	j=3850; 
	for( i=0; i<154; i++)
	{ 
		params.C26[i] = (double) C[j++]; 
	}

	j=4004; 
	for( i=0; i<154; i++)
	{ 
		params.C27[i] = (double) C[j++]; 
	}

	j=4158; 
	for( i=0; i<154; i++)
	{ 
		params.C28[i] = (double) C[j++]; 
	}

	j=4312; 
	for( i=0; i<154; i++)
	{ 
		params.C29[i] = (double) C[j++]; 
	}

	j=4466; 
	for( i=0; i<154; i++)
	{ 
		params.C30[i] = (double) C[j++]; 
	}

	j=4620; 
	for( i=0; i<154; i++)
	{ 
		params.C31[i] = (double) C[j++]; 
	}

	j=4774; 
	for( i=0; i<154; i++)
	{ 
		params.C32[i] = (double) C[j++]; 
	}

	j=4928; 
	for( i=0; i<154; i++)
	{ 
		params.C33[i] = (double) C[j++]; 
	}

	j=5082; 
	for( i=0; i<154; i++)
	{ 
		params.C34[i] = (double) C[j++]; 
	}

	j=5236; 
	for( i=0; i<154; i++)
	{ 
		params.C35[i] = (double) C[j++]; 
	}

	j=5390; 
	for( i=0; i<154; i++)
	{ 
		params.C36[i] = (double) C[j++]; 
	}

	j=5544; 
	for( i=0; i<154; i++)
	{ 
		params.C37[i] = (double) C[j++]; 
	}

	j=5698; 
	for( i=0; i<154; i++)
	{ 
		params.C38[i] = (double) C[j++]; 
	}

	j=5852; 
	for( i=0; i<154; i++)
	{ 
		params.C39[i] = (double) C[j++]; 
	}

	j=6006; 
	for( i=0; i<154; i++)
	{ 
		params.C40[i] = (double) C[j++]; 
	}

	j=6160; 
	for( i=0; i<154; i++)
	{ 
		params.C41[i] = (double) C[j++]; 
	}

	j=6314; 
	for( i=0; i<154; i++)
	{ 
		params.C42[i] = (double) C[j++]; 
	}

	j=6468; 
	for( i=0; i<154; i++)
	{ 
		params.C43[i] = (double) C[j++]; 
	}

	j=6622; 
	for( i=0; i<154; i++)
	{ 
		params.C44[i] = (double) C[j++]; 
	}

	j=6776; 
	for( i=0; i<154; i++)
	{ 
		params.C45[i] = (double) C[j++]; 
	}

	j=6930; 
	for( i=0; i<154; i++)
	{ 
		params.C46[i] = (double) C[j++]; 
	}

	j=7084; 
	for( i=0; i<154; i++)
	{ 
		params.C47[i] = (double) C[j++]; 
	}

	j=7238; 
	for( i=0; i<154; i++)
	{ 
		params.C48[i] = (double) C[j++]; 
	}

	j=7392; 
	for( i=0; i<154; i++)
	{ 
		params.C49[i] = (double) C[j++]; 
	}

	for( i=0; i<154; i++)
	{ 
		params.D01[i] = (double) D[i]; 
	}

	j=154; 
	for( i=0; i<154; i++)
	{ 
		params.D02[i] = (double) D[j++]; 
	}

	j=308; 
	for( i=0; i<154; i++)
	{ 
		params.D03[i] = (double) D[j++]; 
	}

	j=462; 
	for( i=0; i<154; i++)
	{ 
		params.D04[i] = (double) D[j++]; 
	}

	j=616; 
	for( i=0; i<154; i++)
	{ 
		params.D05[i] = (double) D[j++]; 
	}

	j=770; 
	for( i=0; i<154; i++)
	{ 
		params.D06[i] = (double) D[j++]; 
	}

	j=924; 
	for( i=0; i<154; i++)
	{ 
		params.D07[i] = (double) D[j++]; 
	}

	j=1078; 
	for( i=0; i<154; i++)
	{ 
		params.D08[i] = (double) D[j++]; 
	}

	j=1232; 
	for( i=0; i<154; i++)
	{ 
		params.D09[i] = (double) D[j++]; 
	}

	j=1386; 
	for( i=0; i<154; i++)
	{ 
		params.D10[i] = (double) D[j++]; 
	}

	j=1540; 
	for( i=0; i<154; i++)
	{ 
		params.D11[i] = (double) D[j++]; 
	}

	j=1694; 
	for( i=0; i<154; i++)
	{ 
		params.D12[i] = (double) D[j++]; 
	}

	j=1848; 
	for( i=0; i<154; i++)
	{ 
		params.D13[i] = (double) D[j++]; 
	}

	j=2002; 
	for( i=0; i<154; i++)
	{ 
		params.D14[i] = (double) D[j++]; 
	}

	j=2156; 
	for( i=0; i<154; i++)
	{ 
		params.D15[i] = (double) D[j++]; 
	}

	j=2310; 
	for( i=0; i<154; i++)
	{ 
		params.D16[i] = (double) D[j++]; 
	}

	j=2464; 
	for( i=0; i<154; i++)
	{ 
		params.D17[i] = (double) D[j++]; 
	}

	j=2618; 
	for( i=0; i<154; i++)
	{ 
		params.D18[i] = (double) D[j++]; 
	}

	j=2772; 
	for( i=0; i<154; i++)
	{ 
		params.D19[i] = (double) D[j++]; 
	}

	j=2926; 
	for( i=0; i<154; i++)
	{ 
		params.D20[i] = (double) D[j++]; 
	}

	j=3080; 
	for( i=0; i<154; i++)
	{ 
		params.D21[i] = (double) D[j++]; 
	}

	j=3234; 
	for( i=0; i<154; i++)
	{ 
		params.D22[i] = (double) D[j++]; 
	}

	j=3388; 
	for( i=0; i<154; i++)
	{ 
		params.D23[i] = (double) D[j++]; 
	}

	j=3542; 
	for( i=0; i<154; i++)
	{ 
		params.D24[i] = (double) D[j++]; 
	}

	j=3696; 
	for( i=0; i<154; i++)
	{ 
		params.D25[i] = (double) D[j++]; 
	}

	j=3850; 
	for( i=0; i<154; i++)
	{ 
		params.D26[i] = (double) D[j++]; 
	}

	j=4004; 
	for( i=0; i<154; i++)
	{ 
		params.D27[i] = (double) D[j++]; 
	}

	j=4158; 
	for( i=0; i<154; i++)
	{ 
		params.D28[i] = (double) D[j++]; 
	}

	j=4312; 
	for( i=0; i<154; i++)
	{ 
		params.D29[i] = (double) D[j++]; 
	}

	j=4466; 
	for( i=0; i<154; i++)
	{ 
		params.D30[i] = (double) D[j++]; 
	}

	j=4620; 
	for( i=0; i<154; i++)
	{ 
		params.D31[i] = (double) D[j++]; 
	}

	j=4774; 
	for( i=0; i<154; i++)
	{ 
		params.D32[i] = (double) D[j++]; 
	}

	j=4928; 
	for( i=0; i<154; i++)
	{ 
		params.D33[i] = (double) D[j++]; 
	}

	j=5082; 
	for( i=0; i<154; i++)
	{ 
		params.D34[i] = (double) D[j++]; 
	}

	j=5236; 
	for( i=0; i<154; i++)
	{ 
		params.D35[i] = (double) D[j++]; 
	}

	j=5390; 
	for( i=0; i<154; i++)
	{ 
		params.D36[i] = (double) D[j++]; 
	}

	j=5544; 
	for( i=0; i<154; i++)
	{ 
		params.D37[i] = (double) D[j++]; 
	}

	j=5698; 
	for( i=0; i<154; i++)
	{ 
		params.D38[i] = (double) D[j++]; 
	}

	j=5852; 
	for( i=0; i<154; i++)
	{ 
		params.D39[i] = (double) D[j++]; 
	}

	j=6006; 
	for( i=0; i<154; i++)
	{ 
		params.D40[i] = (double) D[j++]; 
	}

	j=6160; 
	for( i=0; i<154; i++)
	{ 
		params.D41[i] = (double) D[j++]; 
	}

	j=6314; 
	for( i=0; i<154; i++)
	{ 
		params.D42[i] = (double) D[j++]; 
	}

	j=6468; 
	for( i=0; i<154; i++)
	{ 
		params.D43[i] = (double) D[j++]; 
	}

	j=6622; 
	for( i=0; i<154; i++)
	{ 
		params.D44[i] = (double) D[j++]; 
	}

	j=6776; 
	for( i=0; i<154; i++)
	{ 
		params.D45[i] = (double) D[j++]; 
	}

	j=6930; 
	for( i=0; i<154; i++)
	{ 
		params.D46[i] = (double) D[j++]; 
	}

	j=7084; 
	for( i=0; i<154; i++)
	{ 
		params.D47[i] = (double) D[j++]; 
	}

	j=7238; 
	for( i=0; i<154; i++)
	{ 
		params.D48[i] = (double) D[j++]; 
	}

	j=7392; 
	for( i=0; i<154; i++)
	{ 
		params.D49[i] = (double) D[j++]; 
	}

	j=7546; 
	for( i=0; i<154; i++)
	{ 
		params.D50[i] = (double) D[j++]; 
	}

	for( i=0; i<11; i++)
	{ 
		params.c02[i] = (double) c[i]; 
	}

	j=11; 
	for( i=0; i<11; i++)
	{ 
		params.c03[i] = (double) c[j++]; 
	}

	j=22; 
	for( i=0; i<11; i++)
	{ 
		params.c04[i] = (double) c[j++]; 
	}

	j=33; 
	for( i=0; i<11; i++)
	{ 
		params.c05[i] = (double) c[j++]; 
	}

	j=44; 
	for( i=0; i<11; i++)
	{ 
		params.c06[i] = (double) c[j++]; 
	}

	j=55; 
	for( i=0; i<11; i++)
	{ 
		params.c07[i] = (double) c[j++]; 
	}

	j=66; 
	for( i=0; i<11; i++)
	{ 
		params.c08[i] = (double) c[j++]; 
	}

	j=77; 
	for( i=0; i<11; i++)
	{ 
		params.c09[i] = (double) c[j++]; 
	}

	j=88; 
	for( i=0; i<11; i++)
	{ 
		params.c10[i] = (double) c[j++]; 
	}

	j=99; 
	for( i=0; i<11; i++)
	{ 
		params.c11[i] = (double) c[j++]; 
	}

	j=110; 
	for( i=0; i<11; i++)
	{ 
		params.c12[i] = (double) c[j++]; 
	}

	j=121; 
	for( i=0; i<11; i++)
	{ 
		params.c13[i] = (double) c[j++]; 
	}

	j=132; 
	for( i=0; i<11; i++)
	{ 
		params.c14[i] = (double) c[j++]; 
	}

	j=143; 
	for( i=0; i<11; i++)
	{ 
		params.c15[i] = (double) c[j++]; 
	}

	j=154; 
	for( i=0; i<11; i++)
	{ 
		params.c16[i] = (double) c[j++]; 
	}

	j=165; 
	for( i=0; i<11; i++)
	{ 
		params.c17[i] = (double) c[j++]; 
	}

	j=176; 
	for( i=0; i<11; i++)
	{ 
		params.c18[i] = (double) c[j++]; 
	}

	j=187; 
	for( i=0; i<11; i++)
	{ 
		params.c19[i] = (double) c[j++]; 
	}

	j=198; 
	for( i=0; i<11; i++)
	{ 
		params.c20[i] = (double) c[j++]; 
	}

	j=209; 
	for( i=0; i<11; i++)
	{ 
		params.c21[i] = (double) c[j++]; 
	}

	j=220; 
	for( i=0; i<11; i++)
	{ 
		params.c22[i] = (double) c[j++]; 
	}

	j=231; 
	for( i=0; i<11; i++)
	{ 
		params.c23[i] = (double) c[j++]; 
	}

	j=242; 
	for( i=0; i<11; i++)
	{ 
		params.c24[i] = (double) c[j++]; 
	}

	j=253; 
	for( i=0; i<11; i++)
	{ 
		params.c25[i] = (double) c[j++]; 
	}

	j=264; 
	for( i=0; i<11; i++)
	{ 
		params.c26[i] = (double) c[j++]; 
	}

	j=275; 
	for( i=0; i<11; i++)
	{ 
		params.c27[i] = (double) c[j++]; 
	}

	j=286; 
	for( i=0; i<11; i++)
	{ 
		params.c28[i] = (double) c[j++]; 
	}

	j=297; 
	for( i=0; i<11; i++)
	{ 
		params.c29[i] = (double) c[j++]; 
	}

	j=308; 
	for( i=0; i<11; i++)
	{ 
		params.c30[i] = (double) c[j++]; 
	}

	j=319; 
	for( i=0; i<11; i++)
	{ 
		params.c31[i] = (double) c[j++]; 
	}

	j=330; 
	for( i=0; i<11; i++)
	{ 
		params.c32[i] = (double) c[j++]; 
	}

	j=341; 
	for( i=0; i<11; i++)
	{ 
		params.c33[i] = (double) c[j++]; 
	}

	j=352; 
	for( i=0; i<11; i++)
	{ 
		params.c34[i] = (double) c[j++]; 
	}

	j=363; 
	for( i=0; i<11; i++)
	{ 
		params.c35[i] = (double) c[j++]; 
	}

	j=374; 
	for( i=0; i<11; i++)
	{ 
		params.c36[i] = (double) c[j++]; 
	}

	j=385; 
	for( i=0; i<11; i++)
	{ 
		params.c37[i] = (double) c[j++]; 
	}

	j=396; 
	for( i=0; i<11; i++)
	{ 
		params.c38[i] = (double) c[j++]; 
	}

	j=407; 
	for( i=0; i<11; i++)
	{ 
		params.c39[i] = (double) c[j++]; 
	}

	j=418; 
	for( i=0; i<11; i++)
	{ 
		params.c40[i] = (double) c[j++]; 
	}

	j=429; 
	for( i=0; i<11; i++)
	{ 
		params.c41[i] = (double) c[j++]; 
	}

	j=440; 
	for( i=0; i<11; i++)
	{ 
		params.c42[i] = (double) c[j++]; 
	}

	j=451; 
	for( i=0; i<11; i++)
	{ 
		params.c43[i] = (double) c[j++]; 
	}

	j=462; 
	for( i=0; i<11; i++)
	{ 
		params.c44[i] = (double) c[j++]; 
	}

	j=473; 
	for( i=0; i<11; i++)
	{ 
		params.c45[i] = (double) c[j++]; 
	}

	j=484; 
	for( i=0; i<11; i++)
	{ 
		params.c46[i] = (double) c[j++]; 
	}

	j=495; 
	for( i=0; i<11; i++)
	{ 
		params.c47[i] = (double) c[j++]; 
	}

	j=506; 
	for( i=0; i<11; i++)
	{ 
		params.c48[i] = (double) c[j++]; 
	}

	j=517; 
	for( i=0; i<11; i++)
	{ 
		params.c49[i] = (double) c[j++]; 
	}

	j=528; 
	for( i=0; i<11; i++)
	{ 
		params.c50[i] = (double) c[j++]; 
	}

	j=539; 
	for( i=0; i<11; i++)
	{ 
		params.minusA_times_x0[i] = (double) c[j++]; 
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
		outputs[i] = (real_T) output.u0[i]; 
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



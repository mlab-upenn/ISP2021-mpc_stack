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
    ssSetInputPortMatrixDimensions(S,  0, 10, 20);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 10, 260);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 10, 247);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 13, 260);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 3, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/	
	/* Input Port 4 */
    ssSetInputPortMatrixDimensions(S,  4, 13, 20);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 4, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 4, 1); /*direct input signal access*/	
	/* Input Port 5 */
    ssSetInputPortMatrixDimensions(S,  5, 2, 20);
    ssSetInputPortDataType(S, 5, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 5, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 5, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 5, 1); /*direct input signal access*/	
	/* Input Port 6 */
    ssSetInputPortMatrixDimensions(S,  6, 2, 260);
    ssSetInputPortDataType(S, 6, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 6, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 6, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 6, 1); /*direct input signal access*/ 


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
	for( i=0; i<169; i++)
	{ 
		params.H01[i] = (double) H[i]; 
	}

	j=169; 
	for( i=0; i<169; i++)
	{ 
		params.H02[i] = (double) H[j++]; 
	}

	j=338; 
	for( i=0; i<169; i++)
	{ 
		params.H03[i] = (double) H[j++]; 
	}

	j=507; 
	for( i=0; i<169; i++)
	{ 
		params.H04[i] = (double) H[j++]; 
	}

	j=676; 
	for( i=0; i<169; i++)
	{ 
		params.H05[i] = (double) H[j++]; 
	}

	j=845; 
	for( i=0; i<169; i++)
	{ 
		params.H06[i] = (double) H[j++]; 
	}

	j=1014; 
	for( i=0; i<169; i++)
	{ 
		params.H07[i] = (double) H[j++]; 
	}

	j=1183; 
	for( i=0; i<169; i++)
	{ 
		params.H08[i] = (double) H[j++]; 
	}

	j=1352; 
	for( i=0; i<169; i++)
	{ 
		params.H09[i] = (double) H[j++]; 
	}

	j=1521; 
	for( i=0; i<169; i++)
	{ 
		params.H10[i] = (double) H[j++]; 
	}

	j=1690; 
	for( i=0; i<169; i++)
	{ 
		params.H11[i] = (double) H[j++]; 
	}

	j=1859; 
	for( i=0; i<169; i++)
	{ 
		params.H12[i] = (double) H[j++]; 
	}

	j=2028; 
	for( i=0; i<169; i++)
	{ 
		params.H13[i] = (double) H[j++]; 
	}

	j=2197; 
	for( i=0; i<169; i++)
	{ 
		params.H14[i] = (double) H[j++]; 
	}

	j=2366; 
	for( i=0; i<169; i++)
	{ 
		params.H15[i] = (double) H[j++]; 
	}

	j=2535; 
	for( i=0; i<169; i++)
	{ 
		params.H16[i] = (double) H[j++]; 
	}

	j=2704; 
	for( i=0; i<169; i++)
	{ 
		params.H17[i] = (double) H[j++]; 
	}

	j=2873; 
	for( i=0; i<169; i++)
	{ 
		params.H18[i] = (double) H[j++]; 
	}

	j=3042; 
	for( i=0; i<169; i++)
	{ 
		params.H19[i] = (double) H[j++]; 
	}

	j=3211; 
	for( i=0; i<169; i++)
	{ 
		params.H20[i] = (double) H[j++]; 
	}

	for( i=0; i<13; i++)
	{ 
		params.f01[i] = (double) f[i]; 
	}

	j=13; 
	for( i=0; i<13; i++)
	{ 
		params.f02[i] = (double) f[j++]; 
	}

	j=26; 
	for( i=0; i<13; i++)
	{ 
		params.f03[i] = (double) f[j++]; 
	}

	j=39; 
	for( i=0; i<13; i++)
	{ 
		params.f04[i] = (double) f[j++]; 
	}

	j=52; 
	for( i=0; i<13; i++)
	{ 
		params.f05[i] = (double) f[j++]; 
	}

	j=65; 
	for( i=0; i<13; i++)
	{ 
		params.f06[i] = (double) f[j++]; 
	}

	j=78; 
	for( i=0; i<13; i++)
	{ 
		params.f07[i] = (double) f[j++]; 
	}

	j=91; 
	for( i=0; i<13; i++)
	{ 
		params.f08[i] = (double) f[j++]; 
	}

	j=104; 
	for( i=0; i<13; i++)
	{ 
		params.f09[i] = (double) f[j++]; 
	}

	j=117; 
	for( i=0; i<13; i++)
	{ 
		params.f10[i] = (double) f[j++]; 
	}

	j=130; 
	for( i=0; i<13; i++)
	{ 
		params.f11[i] = (double) f[j++]; 
	}

	j=143; 
	for( i=0; i<13; i++)
	{ 
		params.f12[i] = (double) f[j++]; 
	}

	j=156; 
	for( i=0; i<13; i++)
	{ 
		params.f13[i] = (double) f[j++]; 
	}

	j=169; 
	for( i=0; i<13; i++)
	{ 
		params.f14[i] = (double) f[j++]; 
	}

	j=182; 
	for( i=0; i<13; i++)
	{ 
		params.f15[i] = (double) f[j++]; 
	}

	j=195; 
	for( i=0; i<13; i++)
	{ 
		params.f16[i] = (double) f[j++]; 
	}

	j=208; 
	for( i=0; i<13; i++)
	{ 
		params.f17[i] = (double) f[j++]; 
	}

	j=221; 
	for( i=0; i<13; i++)
	{ 
		params.f18[i] = (double) f[j++]; 
	}

	j=234; 
	for( i=0; i<13; i++)
	{ 
		params.f19[i] = (double) f[j++]; 
	}

	j=247; 
	for( i=0; i<13; i++)
	{ 
		params.f20[i] = (double) f[j++]; 
	}

	for( i=0; i<26; i++)
	{ 
		params.A01[i] = (double) A[i]; 
	}

	j=26; 
	for( i=0; i<26; i++)
	{ 
		params.A02[i] = (double) A[j++]; 
	}

	j=52; 
	for( i=0; i<26; i++)
	{ 
		params.A03[i] = (double) A[j++]; 
	}

	j=78; 
	for( i=0; i<26; i++)
	{ 
		params.A04[i] = (double) A[j++]; 
	}

	j=104; 
	for( i=0; i<26; i++)
	{ 
		params.A05[i] = (double) A[j++]; 
	}

	j=130; 
	for( i=0; i<26; i++)
	{ 
		params.A06[i] = (double) A[j++]; 
	}

	j=156; 
	for( i=0; i<26; i++)
	{ 
		params.A07[i] = (double) A[j++]; 
	}

	j=182; 
	for( i=0; i<26; i++)
	{ 
		params.A08[i] = (double) A[j++]; 
	}

	j=208; 
	for( i=0; i<26; i++)
	{ 
		params.A09[i] = (double) A[j++]; 
	}

	j=234; 
	for( i=0; i<26; i++)
	{ 
		params.A10[i] = (double) A[j++]; 
	}

	j=260; 
	for( i=0; i<26; i++)
	{ 
		params.A11[i] = (double) A[j++]; 
	}

	j=286; 
	for( i=0; i<26; i++)
	{ 
		params.A12[i] = (double) A[j++]; 
	}

	j=312; 
	for( i=0; i<26; i++)
	{ 
		params.A13[i] = (double) A[j++]; 
	}

	j=338; 
	for( i=0; i<26; i++)
	{ 
		params.A14[i] = (double) A[j++]; 
	}

	j=364; 
	for( i=0; i<26; i++)
	{ 
		params.A15[i] = (double) A[j++]; 
	}

	j=390; 
	for( i=0; i<26; i++)
	{ 
		params.A16[i] = (double) A[j++]; 
	}

	j=416; 
	for( i=0; i<26; i++)
	{ 
		params.A17[i] = (double) A[j++]; 
	}

	j=442; 
	for( i=0; i<26; i++)
	{ 
		params.A18[i] = (double) A[j++]; 
	}

	j=468; 
	for( i=0; i<26; i++)
	{ 
		params.A19[i] = (double) A[j++]; 
	}

	j=494; 
	for( i=0; i<26; i++)
	{ 
		params.A20[i] = (double) A[j++]; 
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

	for( i=0; i<130; i++)
	{ 
		params.C01[i] = (double) C[i]; 
	}

	j=130; 
	for( i=0; i<130; i++)
	{ 
		params.C02[i] = (double) C[j++]; 
	}

	j=260; 
	for( i=0; i<130; i++)
	{ 
		params.C03[i] = (double) C[j++]; 
	}

	j=390; 
	for( i=0; i<130; i++)
	{ 
		params.C04[i] = (double) C[j++]; 
	}

	j=520; 
	for( i=0; i<130; i++)
	{ 
		params.C05[i] = (double) C[j++]; 
	}

	j=650; 
	for( i=0; i<130; i++)
	{ 
		params.C06[i] = (double) C[j++]; 
	}

	j=780; 
	for( i=0; i<130; i++)
	{ 
		params.C07[i] = (double) C[j++]; 
	}

	j=910; 
	for( i=0; i<130; i++)
	{ 
		params.C08[i] = (double) C[j++]; 
	}

	j=1040; 
	for( i=0; i<130; i++)
	{ 
		params.C09[i] = (double) C[j++]; 
	}

	j=1170; 
	for( i=0; i<130; i++)
	{ 
		params.C10[i] = (double) C[j++]; 
	}

	j=1300; 
	for( i=0; i<130; i++)
	{ 
		params.C11[i] = (double) C[j++]; 
	}

	j=1430; 
	for( i=0; i<130; i++)
	{ 
		params.C12[i] = (double) C[j++]; 
	}

	j=1560; 
	for( i=0; i<130; i++)
	{ 
		params.C13[i] = (double) C[j++]; 
	}

	j=1690; 
	for( i=0; i<130; i++)
	{ 
		params.C14[i] = (double) C[j++]; 
	}

	j=1820; 
	for( i=0; i<130; i++)
	{ 
		params.C15[i] = (double) C[j++]; 
	}

	j=1950; 
	for( i=0; i<130; i++)
	{ 
		params.C16[i] = (double) C[j++]; 
	}

	j=2080; 
	for( i=0; i<130; i++)
	{ 
		params.C17[i] = (double) C[j++]; 
	}

	j=2210; 
	for( i=0; i<130; i++)
	{ 
		params.C18[i] = (double) C[j++]; 
	}

	j=2340; 
	for( i=0; i<130; i++)
	{ 
		params.C19[i] = (double) C[j++]; 
	}

	for( i=0; i<130; i++)
	{ 
		params.D01[i] = (double) D[i]; 
	}

	j=130; 
	for( i=0; i<130; i++)
	{ 
		params.D02[i] = (double) D[j++]; 
	}

	j=260; 
	for( i=0; i<130; i++)
	{ 
		params.D03[i] = (double) D[j++]; 
	}

	j=390; 
	for( i=0; i<130; i++)
	{ 
		params.D04[i] = (double) D[j++]; 
	}

	j=520; 
	for( i=0; i<130; i++)
	{ 
		params.D05[i] = (double) D[j++]; 
	}

	j=650; 
	for( i=0; i<130; i++)
	{ 
		params.D06[i] = (double) D[j++]; 
	}

	j=780; 
	for( i=0; i<130; i++)
	{ 
		params.D07[i] = (double) D[j++]; 
	}

	j=910; 
	for( i=0; i<130; i++)
	{ 
		params.D08[i] = (double) D[j++]; 
	}

	j=1040; 
	for( i=0; i<130; i++)
	{ 
		params.D09[i] = (double) D[j++]; 
	}

	j=1170; 
	for( i=0; i<130; i++)
	{ 
		params.D10[i] = (double) D[j++]; 
	}

	j=1300; 
	for( i=0; i<130; i++)
	{ 
		params.D11[i] = (double) D[j++]; 
	}

	j=1430; 
	for( i=0; i<130; i++)
	{ 
		params.D12[i] = (double) D[j++]; 
	}

	j=1560; 
	for( i=0; i<130; i++)
	{ 
		params.D13[i] = (double) D[j++]; 
	}

	j=1690; 
	for( i=0; i<130; i++)
	{ 
		params.D14[i] = (double) D[j++]; 
	}

	j=1820; 
	for( i=0; i<130; i++)
	{ 
		params.D15[i] = (double) D[j++]; 
	}

	j=1950; 
	for( i=0; i<130; i++)
	{ 
		params.D16[i] = (double) D[j++]; 
	}

	j=2080; 
	for( i=0; i<130; i++)
	{ 
		params.D17[i] = (double) D[j++]; 
	}

	j=2210; 
	for( i=0; i<130; i++)
	{ 
		params.D18[i] = (double) D[j++]; 
	}

	j=2340; 
	for( i=0; i<130; i++)
	{ 
		params.D19[i] = (double) D[j++]; 
	}

	j=2470; 
	for( i=0; i<130; i++)
	{ 
		params.D20[i] = (double) D[j++]; 
	}

	for( i=0; i<10; i++)
	{ 
		params.c02[i] = (double) c[i]; 
	}

	j=10; 
	for( i=0; i<10; i++)
	{ 
		params.c03[i] = (double) c[j++]; 
	}

	j=20; 
	for( i=0; i<10; i++)
	{ 
		params.c04[i] = (double) c[j++]; 
	}

	j=30; 
	for( i=0; i<10; i++)
	{ 
		params.c05[i] = (double) c[j++]; 
	}

	j=40; 
	for( i=0; i<10; i++)
	{ 
		params.c06[i] = (double) c[j++]; 
	}

	j=50; 
	for( i=0; i<10; i++)
	{ 
		params.c07[i] = (double) c[j++]; 
	}

	j=60; 
	for( i=0; i<10; i++)
	{ 
		params.c08[i] = (double) c[j++]; 
	}

	j=70; 
	for( i=0; i<10; i++)
	{ 
		params.c09[i] = (double) c[j++]; 
	}

	j=80; 
	for( i=0; i<10; i++)
	{ 
		params.c10[i] = (double) c[j++]; 
	}

	j=90; 
	for( i=0; i<10; i++)
	{ 
		params.c11[i] = (double) c[j++]; 
	}

	j=100; 
	for( i=0; i<10; i++)
	{ 
		params.c12[i] = (double) c[j++]; 
	}

	j=110; 
	for( i=0; i<10; i++)
	{ 
		params.c13[i] = (double) c[j++]; 
	}

	j=120; 
	for( i=0; i<10; i++)
	{ 
		params.c14[i] = (double) c[j++]; 
	}

	j=130; 
	for( i=0; i<10; i++)
	{ 
		params.c15[i] = (double) c[j++]; 
	}

	j=140; 
	for( i=0; i<10; i++)
	{ 
		params.c16[i] = (double) c[j++]; 
	}

	j=150; 
	for( i=0; i<10; i++)
	{ 
		params.c17[i] = (double) c[j++]; 
	}

	j=160; 
	for( i=0; i<10; i++)
	{ 
		params.c18[i] = (double) c[j++]; 
	}

	j=170; 
	for( i=0; i<10; i++)
	{ 
		params.c19[i] = (double) c[j++]; 
	}

	j=180; 
	for( i=0; i<10; i++)
	{ 
		params.c20[i] = (double) c[j++]; 
	}

	j=190; 
	for( i=0; i<10; i++)
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
	for( i=0; i<260; i++)
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



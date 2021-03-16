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

#include "mex.h"
#include "math.h"
#include "../include/mpcc.h"
#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif



/* copy functions */

void copyCArrayToM_double(double *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_double(double *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double) (*src++) ;
    }
}

void copyMValueToC_double(double * src, double * dest)
{
	*dest = (double) *src;
}





/* Some memory for mex-function */
static mpcc_params params;
static mpcc_output output;
static mpcc_info info;

/* THE mex-function */
void mexFunction( solver_int32_default nlhs, mxArray *plhs[], solver_int32_default nrhs, const mxArray *prhs[] )  
{
	/* file pointer for printing */
	FILE *fp = NULL;

	/* define variables */	
	mxArray *par;
	mxArray *outvar;
	const mxArray *PARAMS = prhs[0];
	double *pvalue;
	solver_int32_default i;
	solver_int32_default exitflag;
	const solver_int8_default *fname;
	const solver_int8_default *outputnames[1] = {"u0"};
	const solver_int8_default *infofields[16] = { "it", "it2opt", "res_eq", "res_ineq",  "pobj",  "dobj",  "dgap", "rdgap",  "mu",  "mu_aff",  "sigma",  "lsit_aff",  "lsit_cc",  "step_aff",   "step_cc",  "solvetime"};
	
	/* Check for proper number of arguments */
    if (nrhs != 1) 
	{
        mexErrMsgTxt("This function requires exactly 1 input: PARAMS struct.\nType 'help mpcc_mex' for details.");
    }    
	if (nlhs > 3) 
	{
        mexErrMsgTxt("This function returns at most 3 outputs.\nType 'help mpcc_mex' for details.");
    }

	/* Check whether params is actually a structure */
	if( !mxIsStruct(PARAMS) ) 
	{
		mexErrMsgTxt("PARAMS must be a structure.");
	}

	/* copy parameters into the right location */
	par = mxGetField(PARAMS, 0, "H01");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H01 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H01 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H01 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H01,196);

	}
	par = mxGetField(PARAMS, 0, "H02");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H02 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H02 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H02 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H02,196);

	}
	par = mxGetField(PARAMS, 0, "H03");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H03 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H03 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H03 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H03,196);

	}
	par = mxGetField(PARAMS, 0, "H04");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H04 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H04 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H04 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H04,196);

	}
	par = mxGetField(PARAMS, 0, "H05");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H05 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H05 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H05 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H05,196);

	}
	par = mxGetField(PARAMS, 0, "H06");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H06 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H06 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H06 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H06,196);

	}
	par = mxGetField(PARAMS, 0, "H07");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H07 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H07 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H07 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H07,196);

	}
	par = mxGetField(PARAMS, 0, "H08");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H08 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H08 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H08 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H08,196);

	}
	par = mxGetField(PARAMS, 0, "H09");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H09 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H09 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H09 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H09,196);

	}
	par = mxGetField(PARAMS, 0, "H10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H10 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H10 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H10,196);

	}
	par = mxGetField(PARAMS, 0, "H11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H11 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H11 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H11,196);

	}
	par = mxGetField(PARAMS, 0, "H12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H12 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H12 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H12,196);

	}
	par = mxGetField(PARAMS, 0, "H13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H13 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H13 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H13,196);

	}
	par = mxGetField(PARAMS, 0, "H14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H14 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H14 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H14,196);

	}
	par = mxGetField(PARAMS, 0, "H15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H15 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H15 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H15,196);

	}
	par = mxGetField(PARAMS, 0, "H16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H16 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H16 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H16,196);

	}
	par = mxGetField(PARAMS, 0, "H17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H17 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H17 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H17,196);

	}
	par = mxGetField(PARAMS, 0, "H18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H18 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H18 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H18,196);

	}
	par = mxGetField(PARAMS, 0, "H19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H19 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H19 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H19,196);

	}
	par = mxGetField(PARAMS, 0, "H20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H20 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H20 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H20,196);

	}
	par = mxGetField(PARAMS, 0, "H21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H21 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H21 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H21,196);

	}
	par = mxGetField(PARAMS, 0, "H22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H22 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H22 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H22,196);

	}
	par = mxGetField(PARAMS, 0, "H23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H23 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H23 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H23,196);

	}
	par = mxGetField(PARAMS, 0, "H24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H24 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H24 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H24,196);

	}
	par = mxGetField(PARAMS, 0, "H25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H25 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H25 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H25,196);

	}
	par = mxGetField(PARAMS, 0, "H26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H26 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H26 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H26,196);

	}
	par = mxGetField(PARAMS, 0, "H27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H27 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H27 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H27,196);

	}
	par = mxGetField(PARAMS, 0, "H28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H28 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H28 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H28,196);

	}
	par = mxGetField(PARAMS, 0, "H29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H29 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H29 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H29,196);

	}
	par = mxGetField(PARAMS, 0, "H30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H30 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H30 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H30,196);

	}
	par = mxGetField(PARAMS, 0, "H31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H31 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H31 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H31,196);

	}
	par = mxGetField(PARAMS, 0, "H32");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H32 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H32 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H32 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H32,196);

	}
	par = mxGetField(PARAMS, 0, "H33");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H33 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H33 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H33 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H33,196);

	}
	par = mxGetField(PARAMS, 0, "H34");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H34 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H34 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H34 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H34,196);

	}
	par = mxGetField(PARAMS, 0, "H35");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H35 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H35 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H35 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H35,196);

	}
	par = mxGetField(PARAMS, 0, "H36");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H36 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H36 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H36 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H36,196);

	}
	par = mxGetField(PARAMS, 0, "H37");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H37 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H37 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H37 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H37,196);

	}
	par = mxGetField(PARAMS, 0, "H38");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H38 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H38 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H38 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H38,196);

	}
	par = mxGetField(PARAMS, 0, "H39");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H39 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H39 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H39 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H39,196);

	}
	par = mxGetField(PARAMS, 0, "H40");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H40 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H40 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H40 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H40,196);

	}
	par = mxGetField(PARAMS, 0, "H41");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H41 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H41 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H41 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H41,196);

	}
	par = mxGetField(PARAMS, 0, "H42");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H42 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H42 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H42 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H42,196);

	}
	par = mxGetField(PARAMS, 0, "H43");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H43 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H43 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H43 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H43,196);

	}
	par = mxGetField(PARAMS, 0, "H44");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H44 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H44 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H44 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H44,196);

	}
	par = mxGetField(PARAMS, 0, "H45");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H45 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H45 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H45 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H45,196);

	}
	par = mxGetField(PARAMS, 0, "H46");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H46 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H46 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H46 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H46,196);

	}
	par = mxGetField(PARAMS, 0, "H47");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H47 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H47 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H47 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H47,196);

	}
	par = mxGetField(PARAMS, 0, "H48");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H48 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H48 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H48 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H48,196);

	}
	par = mxGetField(PARAMS, 0, "H49");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H49 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H49 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H49 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H49,196);

	}
	par = mxGetField(PARAMS, 0, "H50");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.H50 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H50 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.H50 must be of size [14 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.H50,196);

	}
	par = mxGetField(PARAMS, 0, "f01");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f01 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f01 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f01 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f01,14);

	}
	par = mxGetField(PARAMS, 0, "f02");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f02 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f02 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f02 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f02,14);

	}
	par = mxGetField(PARAMS, 0, "f03");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f03 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f03 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f03 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f03,14);

	}
	par = mxGetField(PARAMS, 0, "f04");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f04 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f04 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f04 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f04,14);

	}
	par = mxGetField(PARAMS, 0, "f05");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f05 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f05 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f05 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f05,14);

	}
	par = mxGetField(PARAMS, 0, "f06");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f06 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f06 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f06 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f06,14);

	}
	par = mxGetField(PARAMS, 0, "f07");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f07 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f07 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f07 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f07,14);

	}
	par = mxGetField(PARAMS, 0, "f08");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f08 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f08 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f08 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f08,14);

	}
	par = mxGetField(PARAMS, 0, "f09");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f09 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f09 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f09 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f09,14);

	}
	par = mxGetField(PARAMS, 0, "f10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f10 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f10 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f10,14);

	}
	par = mxGetField(PARAMS, 0, "f11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f11 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f11 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f11,14);

	}
	par = mxGetField(PARAMS, 0, "f12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f12 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f12 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f12,14);

	}
	par = mxGetField(PARAMS, 0, "f13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f13 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f13 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f13,14);

	}
	par = mxGetField(PARAMS, 0, "f14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f14 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f14 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f14,14);

	}
	par = mxGetField(PARAMS, 0, "f15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f15 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f15 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f15,14);

	}
	par = mxGetField(PARAMS, 0, "f16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f16 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f16 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f16,14);

	}
	par = mxGetField(PARAMS, 0, "f17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f17 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f17 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f17,14);

	}
	par = mxGetField(PARAMS, 0, "f18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f18 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f18 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f18,14);

	}
	par = mxGetField(PARAMS, 0, "f19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f19 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f19 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f19,14);

	}
	par = mxGetField(PARAMS, 0, "f20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f20 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f20 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f20,14);

	}
	par = mxGetField(PARAMS, 0, "f21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f21 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f21 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f21,14);

	}
	par = mxGetField(PARAMS, 0, "f22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f22 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f22 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f22,14);

	}
	par = mxGetField(PARAMS, 0, "f23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f23 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f23 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f23,14);

	}
	par = mxGetField(PARAMS, 0, "f24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f24 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f24 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f24,14);

	}
	par = mxGetField(PARAMS, 0, "f25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f25 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f25 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f25,14);

	}
	par = mxGetField(PARAMS, 0, "f26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f26 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f26 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f26,14);

	}
	par = mxGetField(PARAMS, 0, "f27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f27 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f27 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f27,14);

	}
	par = mxGetField(PARAMS, 0, "f28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f28 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f28 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f28,14);

	}
	par = mxGetField(PARAMS, 0, "f29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f29 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f29 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f29,14);

	}
	par = mxGetField(PARAMS, 0, "f30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f30 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f30 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f30,14);

	}
	par = mxGetField(PARAMS, 0, "f31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f31 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f31 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f31,14);

	}
	par = mxGetField(PARAMS, 0, "f32");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f32 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f32 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f32 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f32,14);

	}
	par = mxGetField(PARAMS, 0, "f33");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f33 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f33 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f33 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f33,14);

	}
	par = mxGetField(PARAMS, 0, "f34");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f34 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f34 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f34 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f34,14);

	}
	par = mxGetField(PARAMS, 0, "f35");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f35 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f35 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f35 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f35,14);

	}
	par = mxGetField(PARAMS, 0, "f36");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f36 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f36 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f36 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f36,14);

	}
	par = mxGetField(PARAMS, 0, "f37");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f37 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f37 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f37 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f37,14);

	}
	par = mxGetField(PARAMS, 0, "f38");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f38 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f38 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f38 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f38,14);

	}
	par = mxGetField(PARAMS, 0, "f39");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f39 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f39 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f39 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f39,14);

	}
	par = mxGetField(PARAMS, 0, "f40");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f40 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f40 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f40 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f40,14);

	}
	par = mxGetField(PARAMS, 0, "f41");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f41 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f41 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f41 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f41,14);

	}
	par = mxGetField(PARAMS, 0, "f42");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f42 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f42 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f42 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f42,14);

	}
	par = mxGetField(PARAMS, 0, "f43");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f43 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f43 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f43 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f43,14);

	}
	par = mxGetField(PARAMS, 0, "f44");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f44 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f44 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f44 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f44,14);

	}
	par = mxGetField(PARAMS, 0, "f45");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f45 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f45 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f45 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f45,14);

	}
	par = mxGetField(PARAMS, 0, "f46");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f46 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f46 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f46 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f46,14);

	}
	par = mxGetField(PARAMS, 0, "f47");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f47 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f47 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f47 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f47,14);

	}
	par = mxGetField(PARAMS, 0, "f48");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f48 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f48 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f48 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f48,14);

	}
	par = mxGetField(PARAMS, 0, "f49");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f49 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f49 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f49 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f49,14);

	}
	par = mxGetField(PARAMS, 0, "f50");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.f50 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f50 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.f50 must be of size [14 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.f50,14);

	}
	par = mxGetField(PARAMS, 0, "A01");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A01 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A01 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A01 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A01,28);

	}
	par = mxGetField(PARAMS, 0, "A02");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A02 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A02 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A02 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A02,28);

	}
	par = mxGetField(PARAMS, 0, "A03");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A03 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A03 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A03 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A03,28);

	}
	par = mxGetField(PARAMS, 0, "A04");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A04 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A04 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A04 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A04,28);

	}
	par = mxGetField(PARAMS, 0, "A05");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A05 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A05 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A05 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A05,28);

	}
	par = mxGetField(PARAMS, 0, "A06");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A06 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A06 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A06 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A06,28);

	}
	par = mxGetField(PARAMS, 0, "A07");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A07 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A07 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A07 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A07,28);

	}
	par = mxGetField(PARAMS, 0, "A08");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A08 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A08 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A08 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A08,28);

	}
	par = mxGetField(PARAMS, 0, "A09");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A09 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A09 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A09 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A09,28);

	}
	par = mxGetField(PARAMS, 0, "A10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A10 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A10 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A10,28);

	}
	par = mxGetField(PARAMS, 0, "A11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A11 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A11 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A11,28);

	}
	par = mxGetField(PARAMS, 0, "A12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A12 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A12 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A12,28);

	}
	par = mxGetField(PARAMS, 0, "A13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A13 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A13 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A13,28);

	}
	par = mxGetField(PARAMS, 0, "A14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A14 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A14 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A14,28);

	}
	par = mxGetField(PARAMS, 0, "A15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A15 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A15 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A15,28);

	}
	par = mxGetField(PARAMS, 0, "A16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A16 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A16 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A16,28);

	}
	par = mxGetField(PARAMS, 0, "A17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A17 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A17 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A17,28);

	}
	par = mxGetField(PARAMS, 0, "A18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A18 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A18 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A18,28);

	}
	par = mxGetField(PARAMS, 0, "A19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A19 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A19 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A19,28);

	}
	par = mxGetField(PARAMS, 0, "A20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A20 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A20 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A20,28);

	}
	par = mxGetField(PARAMS, 0, "A21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A21 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A21 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A21,28);

	}
	par = mxGetField(PARAMS, 0, "A22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A22 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A22 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A22,28);

	}
	par = mxGetField(PARAMS, 0, "A23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A23 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A23 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A23,28);

	}
	par = mxGetField(PARAMS, 0, "A24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A24 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A24 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A24,28);

	}
	par = mxGetField(PARAMS, 0, "A25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A25 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A25 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A25,28);

	}
	par = mxGetField(PARAMS, 0, "A26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A26 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A26 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A26,28);

	}
	par = mxGetField(PARAMS, 0, "A27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A27 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A27 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A27,28);

	}
	par = mxGetField(PARAMS, 0, "A28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A28 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A28 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A28,28);

	}
	par = mxGetField(PARAMS, 0, "A29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A29 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A29 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A29,28);

	}
	par = mxGetField(PARAMS, 0, "A30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A30 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A30 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A30,28);

	}
	par = mxGetField(PARAMS, 0, "A31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A31 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A31 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A31,28);

	}
	par = mxGetField(PARAMS, 0, "A32");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A32 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A32 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A32 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A32,28);

	}
	par = mxGetField(PARAMS, 0, "A33");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A33 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A33 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A33 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A33,28);

	}
	par = mxGetField(PARAMS, 0, "A34");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A34 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A34 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A34 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A34,28);

	}
	par = mxGetField(PARAMS, 0, "A35");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A35 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A35 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A35 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A35,28);

	}
	par = mxGetField(PARAMS, 0, "A36");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A36 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A36 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A36 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A36,28);

	}
	par = mxGetField(PARAMS, 0, "A37");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A37 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A37 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A37 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A37,28);

	}
	par = mxGetField(PARAMS, 0, "A38");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A38 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A38 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A38 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A38,28);

	}
	par = mxGetField(PARAMS, 0, "A39");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A39 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A39 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A39 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A39,28);

	}
	par = mxGetField(PARAMS, 0, "A40");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A40 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A40 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A40 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A40,28);

	}
	par = mxGetField(PARAMS, 0, "A41");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A41 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A41 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A41 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A41,28);

	}
	par = mxGetField(PARAMS, 0, "A42");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A42 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A42 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A42 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A42,28);

	}
	par = mxGetField(PARAMS, 0, "A43");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A43 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A43 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A43 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A43,28);

	}
	par = mxGetField(PARAMS, 0, "A44");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A44 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A44 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A44 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A44,28);

	}
	par = mxGetField(PARAMS, 0, "A45");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A45 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A45 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A45 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A45,28);

	}
	par = mxGetField(PARAMS, 0, "A46");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A46 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A46 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A46 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A46,28);

	}
	par = mxGetField(PARAMS, 0, "A47");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A47 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A47 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A47 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A47,28);

	}
	par = mxGetField(PARAMS, 0, "A48");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A48 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A48 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A48 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A48,28);

	}
	par = mxGetField(PARAMS, 0, "A49");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A49 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A49 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A49 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A49,28);

	}
	par = mxGetField(PARAMS, 0, "A50");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.A50 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A50 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.A50 must be of size [2 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.A50,28);

	}
	par = mxGetField(PARAMS, 0, "b01");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b01 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b01 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b01 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b01,2);

	}
	par = mxGetField(PARAMS, 0, "b02");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b02 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b02 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b02 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b02,2);

	}
	par = mxGetField(PARAMS, 0, "b03");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b03 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b03 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b03 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b03,2);

	}
	par = mxGetField(PARAMS, 0, "b04");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b04 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b04 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b04 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b04,2);

	}
	par = mxGetField(PARAMS, 0, "b05");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b05 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b05 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b05 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b05,2);

	}
	par = mxGetField(PARAMS, 0, "b06");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b06 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b06 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b06 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b06,2);

	}
	par = mxGetField(PARAMS, 0, "b07");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b07 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b07 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b07 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b07,2);

	}
	par = mxGetField(PARAMS, 0, "b08");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b08 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b08 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b08 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b08,2);

	}
	par = mxGetField(PARAMS, 0, "b09");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b09 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b09 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b09 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b09,2);

	}
	par = mxGetField(PARAMS, 0, "b10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b10 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b10 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b10,2);

	}
	par = mxGetField(PARAMS, 0, "b11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b11 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b11 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b11,2);

	}
	par = mxGetField(PARAMS, 0, "b12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b12 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b12 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b12,2);

	}
	par = mxGetField(PARAMS, 0, "b13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b13 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b13 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b13,2);

	}
	par = mxGetField(PARAMS, 0, "b14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b14 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b14 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b14,2);

	}
	par = mxGetField(PARAMS, 0, "b15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b15 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b15 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b15,2);

	}
	par = mxGetField(PARAMS, 0, "b16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b16 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b16 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b16,2);

	}
	par = mxGetField(PARAMS, 0, "b17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b17 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b17 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b17,2);

	}
	par = mxGetField(PARAMS, 0, "b18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b18 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b18 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b18,2);

	}
	par = mxGetField(PARAMS, 0, "b19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b19 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b19 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b19,2);

	}
	par = mxGetField(PARAMS, 0, "b20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b20 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b20 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b20,2);

	}
	par = mxGetField(PARAMS, 0, "b21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b21 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b21 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b21,2);

	}
	par = mxGetField(PARAMS, 0, "b22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b22 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b22 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b22,2);

	}
	par = mxGetField(PARAMS, 0, "b23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b23 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b23 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b23,2);

	}
	par = mxGetField(PARAMS, 0, "b24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b24 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b24 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b24,2);

	}
	par = mxGetField(PARAMS, 0, "b25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b25 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b25 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b25,2);

	}
	par = mxGetField(PARAMS, 0, "b26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b26 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b26 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b26,2);

	}
	par = mxGetField(PARAMS, 0, "b27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b27 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b27 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b27,2);

	}
	par = mxGetField(PARAMS, 0, "b28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b28 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b28 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b28,2);

	}
	par = mxGetField(PARAMS, 0, "b29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b29 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b29 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b29,2);

	}
	par = mxGetField(PARAMS, 0, "b30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b30 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b30 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b30,2);

	}
	par = mxGetField(PARAMS, 0, "b31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b31 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b31 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b31,2);

	}
	par = mxGetField(PARAMS, 0, "b32");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b32 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b32 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b32 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b32,2);

	}
	par = mxGetField(PARAMS, 0, "b33");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b33 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b33 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b33 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b33,2);

	}
	par = mxGetField(PARAMS, 0, "b34");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b34 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b34 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b34 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b34,2);

	}
	par = mxGetField(PARAMS, 0, "b35");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b35 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b35 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b35 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b35,2);

	}
	par = mxGetField(PARAMS, 0, "b36");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b36 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b36 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b36 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b36,2);

	}
	par = mxGetField(PARAMS, 0, "b37");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b37 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b37 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b37 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b37,2);

	}
	par = mxGetField(PARAMS, 0, "b38");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b38 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b38 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b38 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b38,2);

	}
	par = mxGetField(PARAMS, 0, "b39");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b39 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b39 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b39 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b39,2);

	}
	par = mxGetField(PARAMS, 0, "b40");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b40 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b40 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b40 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b40,2);

	}
	par = mxGetField(PARAMS, 0, "b41");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b41 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b41 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b41 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b41,2);

	}
	par = mxGetField(PARAMS, 0, "b42");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b42 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b42 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b42 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b42,2);

	}
	par = mxGetField(PARAMS, 0, "b43");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b43 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b43 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b43 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b43,2);

	}
	par = mxGetField(PARAMS, 0, "b44");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b44 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b44 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b44 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b44,2);

	}
	par = mxGetField(PARAMS, 0, "b45");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b45 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b45 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b45 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b45,2);

	}
	par = mxGetField(PARAMS, 0, "b46");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b46 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b46 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b46 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b46,2);

	}
	par = mxGetField(PARAMS, 0, "b47");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b47 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b47 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b47 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b47,2);

	}
	par = mxGetField(PARAMS, 0, "b48");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b48 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b48 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b48 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b48,2);

	}
	par = mxGetField(PARAMS, 0, "b49");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b49 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b49 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b49 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b49,2);

	}
	par = mxGetField(PARAMS, 0, "b50");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.b50 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b50 must be a double.");
    }
    if( mxGetM(par) != 2 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.b50 must be of size [2 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.b50,2);

	}
	par = mxGetField(PARAMS, 0, "C01");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C01 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C01 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C01 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C01,154);

	}
	par = mxGetField(PARAMS, 0, "C02");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C02 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C02 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C02 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C02,154);

	}
	par = mxGetField(PARAMS, 0, "C03");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C03 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C03 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C03 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C03,154);

	}
	par = mxGetField(PARAMS, 0, "C04");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C04 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C04 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C04 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C04,154);

	}
	par = mxGetField(PARAMS, 0, "C05");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C05 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C05 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C05 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C05,154);

	}
	par = mxGetField(PARAMS, 0, "C06");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C06 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C06 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C06 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C06,154);

	}
	par = mxGetField(PARAMS, 0, "C07");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C07 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C07 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C07 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C07,154);

	}
	par = mxGetField(PARAMS, 0, "C08");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C08 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C08 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C08 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C08,154);

	}
	par = mxGetField(PARAMS, 0, "C09");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C09 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C09 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C09 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C09,154);

	}
	par = mxGetField(PARAMS, 0, "C10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C10 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C10 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C10,154);

	}
	par = mxGetField(PARAMS, 0, "C11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C11 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C11 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C11,154);

	}
	par = mxGetField(PARAMS, 0, "C12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C12 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C12 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C12,154);

	}
	par = mxGetField(PARAMS, 0, "C13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C13 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C13 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C13,154);

	}
	par = mxGetField(PARAMS, 0, "C14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C14 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C14 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C14,154);

	}
	par = mxGetField(PARAMS, 0, "C15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C15 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C15 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C15,154);

	}
	par = mxGetField(PARAMS, 0, "C16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C16 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C16 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C16,154);

	}
	par = mxGetField(PARAMS, 0, "C17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C17 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C17 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C17,154);

	}
	par = mxGetField(PARAMS, 0, "C18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C18 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C18 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C18,154);

	}
	par = mxGetField(PARAMS, 0, "C19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C19 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C19 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C19,154);

	}
	par = mxGetField(PARAMS, 0, "C20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C20 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C20 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C20,154);

	}
	par = mxGetField(PARAMS, 0, "C21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C21 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C21 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C21,154);

	}
	par = mxGetField(PARAMS, 0, "C22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C22 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C22 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C22,154);

	}
	par = mxGetField(PARAMS, 0, "C23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C23 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C23 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C23,154);

	}
	par = mxGetField(PARAMS, 0, "C24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C24 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C24 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C24,154);

	}
	par = mxGetField(PARAMS, 0, "C25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C25 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C25 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C25,154);

	}
	par = mxGetField(PARAMS, 0, "C26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C26 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C26 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C26,154);

	}
	par = mxGetField(PARAMS, 0, "C27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C27 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C27 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C27,154);

	}
	par = mxGetField(PARAMS, 0, "C28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C28 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C28 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C28,154);

	}
	par = mxGetField(PARAMS, 0, "C29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C29 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C29 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C29,154);

	}
	par = mxGetField(PARAMS, 0, "C30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C30 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C30 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C30,154);

	}
	par = mxGetField(PARAMS, 0, "C31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C31 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C31 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C31,154);

	}
	par = mxGetField(PARAMS, 0, "C32");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C32 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C32 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C32 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C32,154);

	}
	par = mxGetField(PARAMS, 0, "C33");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C33 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C33 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C33 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C33,154);

	}
	par = mxGetField(PARAMS, 0, "C34");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C34 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C34 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C34 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C34,154);

	}
	par = mxGetField(PARAMS, 0, "C35");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C35 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C35 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C35 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C35,154);

	}
	par = mxGetField(PARAMS, 0, "C36");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C36 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C36 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C36 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C36,154);

	}
	par = mxGetField(PARAMS, 0, "C37");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C37 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C37 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C37 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C37,154);

	}
	par = mxGetField(PARAMS, 0, "C38");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C38 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C38 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C38 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C38,154);

	}
	par = mxGetField(PARAMS, 0, "C39");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C39 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C39 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C39 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C39,154);

	}
	par = mxGetField(PARAMS, 0, "C40");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C40 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C40 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C40 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C40,154);

	}
	par = mxGetField(PARAMS, 0, "C41");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C41 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C41 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C41 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C41,154);

	}
	par = mxGetField(PARAMS, 0, "C42");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C42 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C42 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C42 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C42,154);

	}
	par = mxGetField(PARAMS, 0, "C43");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C43 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C43 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C43 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C43,154);

	}
	par = mxGetField(PARAMS, 0, "C44");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C44 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C44 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C44 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C44,154);

	}
	par = mxGetField(PARAMS, 0, "C45");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C45 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C45 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C45 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C45,154);

	}
	par = mxGetField(PARAMS, 0, "C46");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C46 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C46 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C46 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C46,154);

	}
	par = mxGetField(PARAMS, 0, "C47");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C47 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C47 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C47 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C47,154);

	}
	par = mxGetField(PARAMS, 0, "C48");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C48 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C48 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C48 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C48,154);

	}
	par = mxGetField(PARAMS, 0, "C49");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.C49 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C49 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.C49 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.C49,154);

	}
	par = mxGetField(PARAMS, 0, "D01");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D01 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D01 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D01 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D01,154);

	}
	par = mxGetField(PARAMS, 0, "D02");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D02 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D02 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D02 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D02,154);

	}
	par = mxGetField(PARAMS, 0, "D03");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D03 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D03 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D03 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D03,154);

	}
	par = mxGetField(PARAMS, 0, "D04");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D04 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D04 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D04 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D04,154);

	}
	par = mxGetField(PARAMS, 0, "D05");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D05 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D05 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D05 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D05,154);

	}
	par = mxGetField(PARAMS, 0, "D06");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D06 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D06 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D06 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D06,154);

	}
	par = mxGetField(PARAMS, 0, "D07");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D07 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D07 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D07 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D07,154);

	}
	par = mxGetField(PARAMS, 0, "D08");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D08 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D08 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D08 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D08,154);

	}
	par = mxGetField(PARAMS, 0, "D09");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D09 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D09 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D09 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D09,154);

	}
	par = mxGetField(PARAMS, 0, "D10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D10 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D10 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D10,154);

	}
	par = mxGetField(PARAMS, 0, "D11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D11 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D11 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D11,154);

	}
	par = mxGetField(PARAMS, 0, "D12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D12 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D12 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D12,154);

	}
	par = mxGetField(PARAMS, 0, "D13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D13 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D13 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D13,154);

	}
	par = mxGetField(PARAMS, 0, "D14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D14 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D14 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D14,154);

	}
	par = mxGetField(PARAMS, 0, "D15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D15 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D15 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D15,154);

	}
	par = mxGetField(PARAMS, 0, "D16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D16 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D16 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D16,154);

	}
	par = mxGetField(PARAMS, 0, "D17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D17 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D17 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D17,154);

	}
	par = mxGetField(PARAMS, 0, "D18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D18 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D18 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D18,154);

	}
	par = mxGetField(PARAMS, 0, "D19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D19 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D19 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D19,154);

	}
	par = mxGetField(PARAMS, 0, "D20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D20 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D20 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D20,154);

	}
	par = mxGetField(PARAMS, 0, "D21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D21 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D21 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D21,154);

	}
	par = mxGetField(PARAMS, 0, "D22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D22 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D22 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D22,154);

	}
	par = mxGetField(PARAMS, 0, "D23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D23 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D23 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D23,154);

	}
	par = mxGetField(PARAMS, 0, "D24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D24 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D24 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D24,154);

	}
	par = mxGetField(PARAMS, 0, "D25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D25 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D25 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D25,154);

	}
	par = mxGetField(PARAMS, 0, "D26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D26 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D26 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D26,154);

	}
	par = mxGetField(PARAMS, 0, "D27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D27 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D27 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D27,154);

	}
	par = mxGetField(PARAMS, 0, "D28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D28 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D28 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D28,154);

	}
	par = mxGetField(PARAMS, 0, "D29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D29 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D29 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D29,154);

	}
	par = mxGetField(PARAMS, 0, "D30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D30 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D30 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D30,154);

	}
	par = mxGetField(PARAMS, 0, "D31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D31 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D31 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D31,154);

	}
	par = mxGetField(PARAMS, 0, "D32");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D32 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D32 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D32 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D32,154);

	}
	par = mxGetField(PARAMS, 0, "D33");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D33 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D33 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D33 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D33,154);

	}
	par = mxGetField(PARAMS, 0, "D34");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D34 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D34 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D34 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D34,154);

	}
	par = mxGetField(PARAMS, 0, "D35");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D35 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D35 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D35 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D35,154);

	}
	par = mxGetField(PARAMS, 0, "D36");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D36 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D36 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D36 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D36,154);

	}
	par = mxGetField(PARAMS, 0, "D37");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D37 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D37 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D37 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D37,154);

	}
	par = mxGetField(PARAMS, 0, "D38");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D38 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D38 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D38 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D38,154);

	}
	par = mxGetField(PARAMS, 0, "D39");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D39 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D39 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D39 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D39,154);

	}
	par = mxGetField(PARAMS, 0, "D40");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D40 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D40 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D40 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D40,154);

	}
	par = mxGetField(PARAMS, 0, "D41");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D41 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D41 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D41 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D41,154);

	}
	par = mxGetField(PARAMS, 0, "D42");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D42 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D42 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D42 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D42,154);

	}
	par = mxGetField(PARAMS, 0, "D43");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D43 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D43 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D43 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D43,154);

	}
	par = mxGetField(PARAMS, 0, "D44");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D44 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D44 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D44 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D44,154);

	}
	par = mxGetField(PARAMS, 0, "D45");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D45 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D45 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D45 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D45,154);

	}
	par = mxGetField(PARAMS, 0, "D46");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D46 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D46 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D46 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D46,154);

	}
	par = mxGetField(PARAMS, 0, "D47");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D47 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D47 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D47 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D47,154);

	}
	par = mxGetField(PARAMS, 0, "D48");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D48 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D48 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D48 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D48,154);

	}
	par = mxGetField(PARAMS, 0, "D49");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D49 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D49 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D49 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D49,154);

	}
	par = mxGetField(PARAMS, 0, "D50");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.D50 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.D50 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 14 ) 
	{
    mexErrMsgTxt("PARAMS.D50 must be of size [11 x 14]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.D50,154);

	}
	par = mxGetField(PARAMS, 0, "c02");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c02 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c02 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c02 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c02,11);

	}
	par = mxGetField(PARAMS, 0, "c03");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c03 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c03 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c03 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c03,11);

	}
	par = mxGetField(PARAMS, 0, "c04");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c04 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c04 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c04 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c04,11);

	}
	par = mxGetField(PARAMS, 0, "c05");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c05 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c05 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c05 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c05,11);

	}
	par = mxGetField(PARAMS, 0, "c06");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c06 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c06 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c06 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c06,11);

	}
	par = mxGetField(PARAMS, 0, "c07");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c07 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c07 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c07 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c07,11);

	}
	par = mxGetField(PARAMS, 0, "c08");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c08 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c08 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c08 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c08,11);

	}
	par = mxGetField(PARAMS, 0, "c09");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c09 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c09 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c09 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c09,11);

	}
	par = mxGetField(PARAMS, 0, "c10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c10 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c10 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c10,11);

	}
	par = mxGetField(PARAMS, 0, "c11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c11 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c11 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c11,11);

	}
	par = mxGetField(PARAMS, 0, "c12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c12 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c12 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c12,11);

	}
	par = mxGetField(PARAMS, 0, "c13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c13 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c13 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c13,11);

	}
	par = mxGetField(PARAMS, 0, "c14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c14 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c14 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c14,11);

	}
	par = mxGetField(PARAMS, 0, "c15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c15 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c15 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c15,11);

	}
	par = mxGetField(PARAMS, 0, "c16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c16 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c16 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c16,11);

	}
	par = mxGetField(PARAMS, 0, "c17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c17 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c17 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c17,11);

	}
	par = mxGetField(PARAMS, 0, "c18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c18 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c18 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c18,11);

	}
	par = mxGetField(PARAMS, 0, "c19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c19 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c19 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c19,11);

	}
	par = mxGetField(PARAMS, 0, "c20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c20 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c20 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c20,11);

	}
	par = mxGetField(PARAMS, 0, "c21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c21 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c21 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c21,11);

	}
	par = mxGetField(PARAMS, 0, "c22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c22 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c22 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c22,11);

	}
	par = mxGetField(PARAMS, 0, "c23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c23 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c23 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c23,11);

	}
	par = mxGetField(PARAMS, 0, "c24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c24 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c24 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c24,11);

	}
	par = mxGetField(PARAMS, 0, "c25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c25 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c25 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c25,11);

	}
	par = mxGetField(PARAMS, 0, "c26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c26 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c26 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c26,11);

	}
	par = mxGetField(PARAMS, 0, "c27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c27 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c27 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c27,11);

	}
	par = mxGetField(PARAMS, 0, "c28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c28 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c28 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c28,11);

	}
	par = mxGetField(PARAMS, 0, "c29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c29 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c29 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c29,11);

	}
	par = mxGetField(PARAMS, 0, "c30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c30 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c30 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c30,11);

	}
	par = mxGetField(PARAMS, 0, "c31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c31 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c31 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c31,11);

	}
	par = mxGetField(PARAMS, 0, "c32");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c32 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c32 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c32 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c32,11);

	}
	par = mxGetField(PARAMS, 0, "c33");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c33 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c33 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c33 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c33,11);

	}
	par = mxGetField(PARAMS, 0, "c34");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c34 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c34 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c34 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c34,11);

	}
	par = mxGetField(PARAMS, 0, "c35");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c35 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c35 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c35 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c35,11);

	}
	par = mxGetField(PARAMS, 0, "c36");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c36 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c36 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c36 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c36,11);

	}
	par = mxGetField(PARAMS, 0, "c37");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c37 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c37 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c37 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c37,11);

	}
	par = mxGetField(PARAMS, 0, "c38");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c38 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c38 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c38 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c38,11);

	}
	par = mxGetField(PARAMS, 0, "c39");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c39 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c39 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c39 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c39,11);

	}
	par = mxGetField(PARAMS, 0, "c40");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c40 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c40 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c40 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c40,11);

	}
	par = mxGetField(PARAMS, 0, "c41");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c41 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c41 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c41 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c41,11);

	}
	par = mxGetField(PARAMS, 0, "c42");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c42 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c42 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c42 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c42,11);

	}
	par = mxGetField(PARAMS, 0, "c43");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c43 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c43 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c43 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c43,11);

	}
	par = mxGetField(PARAMS, 0, "c44");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c44 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c44 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c44 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c44,11);

	}
	par = mxGetField(PARAMS, 0, "c45");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c45 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c45 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c45 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c45,11);

	}
	par = mxGetField(PARAMS, 0, "c46");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c46 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c46 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c46 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c46,11);

	}
	par = mxGetField(PARAMS, 0, "c47");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c47 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c47 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c47 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c47,11);

	}
	par = mxGetField(PARAMS, 0, "c48");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c48 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c48 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c48 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c48,11);

	}
	par = mxGetField(PARAMS, 0, "c49");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c49 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c49 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c49 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c49,11);

	}
	par = mxGetField(PARAMS, 0, "c50");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.c50 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.c50 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.c50 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.c50,11);

	}
	par = mxGetField(PARAMS, 0, "minusA_times_x0");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.minusA_times_x0 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.minusA_times_x0 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.minusA_times_x0 must be of size [11 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.minusA_times_x0,11);

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

	/* call solver */
	exitflag = mpcc_solve(&params, &output, &info, fp);
	
	#if SET_PRINTLEVEL_mpcc > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			mexPrintf("%c",i);
		}
		fclose(fp);
	#endif

	/* copy output to matlab arrays */
	plhs[0] = mxCreateStructMatrix(1, 1, 1, outputnames);
		outvar = mxCreateDoubleMatrix(700, 1, mxREAL);
	copyCArrayToM_double( output.u0, mxGetPr(outvar), 700);
	mxSetField(plhs[0], 0, "u0", outvar);

	/* copy exitflag */
	if( nlhs > 1 )
	{
	plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
	*mxGetPr(plhs[1]) = (double)exitflag;
	}

	/* copy info struct */
	if( nlhs > 2 )
	{
	        plhs[2] = mxCreateStructMatrix(1, 1, 16, infofields);
         
		
		/* iterations */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it;
		mxSetField(plhs[2], 0, "it", outvar);

		/* iterations to optimality (branch and bound) */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it2opt;
		mxSetField(plhs[2], 0, "it2opt", outvar);
		
		/* res_eq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_eq;
		mxSetField(plhs[2], 0, "res_eq", outvar);

		/* res_ineq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_ineq;
		mxSetField(plhs[2], 0, "res_ineq", outvar);

		/* pobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.pobj;
		mxSetField(plhs[2], 0, "pobj", outvar);

		/* dobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.dobj;
		mxSetField(plhs[2], 0, "dobj", outvar);

		/* dgap */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.dgap;
		mxSetField(plhs[2], 0, "dgap", outvar);

		/* rdgap */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.rdgap;
		mxSetField(plhs[2], 0, "rdgap", outvar);

		/* mu */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.mu;
		mxSetField(plhs[2], 0, "mu", outvar);

		/* mu_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.mu_aff;
		mxSetField(plhs[2], 0, "mu_aff", outvar);

		/* sigma */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.sigma;
		mxSetField(plhs[2], 0, "sigma", outvar);

		/* lsit_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.lsit_aff;
		mxSetField(plhs[2], 0, "lsit_aff", outvar);

		/* lsit_cc */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.lsit_cc;
		mxSetField(plhs[2], 0, "lsit_cc", outvar);

		/* step_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.step_aff;
		mxSetField(plhs[2], 0, "step_aff", outvar);

		/* step_cc */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.step_cc;
		mxSetField(plhs[2], 0, "step_cc", outvar);

		/* solver time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.solvetime;
		mxSetField(plhs[2], 0, "solvetime", outvar);
	}
}
/*
 * sfuntmpl_basic.c: Basic 'C' template for a level 2 S-function.
 *
 *  -------------------------------------------------------------------------
 *  | See matlabroot/simulink/src/sfuntmpl_doc.c for a more detailed template |
 *  -------------------------------------------------------------------------
 *
 * Copyright 1990-2002 The MathWorks, Inc.
 * $Revision: 1.27 $
 */


/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 */

#define S_FUNCTION_NAME  sfun_iinputport
#define S_FUNCTION_LEVEL 2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#ifndef MATLAB_MEX_FILE
extern "C"
{
#endif
#include "simstruc.h"
#ifndef MATLAB_MEX_FILE
}
#endif
#ifndef MATLAB_MEX_FILE
#include "sfun_taskcontext.hpp"
#endif

/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 * See matlabroot/simulink/src/sfuntmpl_doc.c for more details.
 */

#define DESC_IDX  0
#define DESC_PARAM(S) ssGetSFcnParam(S,DESC_IDX) /* Second parameter */

#define DEFVALUE_IDX   1
#define DEFVALUE_PARAM(S) ssGetSFcnParam(S,DEFVALUE_IDX) /* Third parameter */

// From sfun_dtype_param.c
#define EDIT_OK(S, ARG) \
       (!((ssGetSimMode(S) == SS_SIMMODE_SIZES_CALL_ONLY) && mxIsEmpty(ARG)))

#ifndef MATLAB_MEX_FILE
namespace {
    typedef RTT::InputPort< int > IntInputPort;

    struct IntPort
    {
        IntPort(std::string name, int init )
            : port( new IntInputPort(name) ), d(init), cleanup(false) {} // See bug #579: Simulink component configuration procedure broken when using DeploymentComponent
        IntPort( IntInputPort* dp, int init )
            : port( dp ), d(init), cleanup(false) {
        }
        ~IntPort() {
            if (cleanup)
                delete port;
        }
        IntInputPort* port;
        int d;
        bool cleanup;
    };
}
#endif

/*====================*
 * S-function methods *
 *====================*/

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
  /* Function: mdlCheckParameters =============================================
   * Abstract:
   *    Validate our parameters to verify they are okay.
   */
  static void mdlCheckParameters(SimStruct *S)
  {
      /* Check 1st parameter: name parameter */
      {
          if (!mxIsChar(DESC_PARAM(S))) {
              ssSetErrorStatus(S,"Please provide a description as first "
                               "parameter");
              return;
          }
      }

      // from sfun_dtype_param.c
    if(EDIT_OK(S, DEFVALUE_PARAM(S))) {
        boolean_T isOk = (mxIsNumeric(DEFVALUE_PARAM(S))  &&  
                          !mxIsLogical(DEFVALUE_PARAM(S)) &&
                          !mxIsComplex(DEFVALUE_PARAM(S)) &&  
                          !mxIsSparse(DEFVALUE_PARAM(S))  && 
                          !mxIsEmpty(DEFVALUE_PARAM(S))   &&
                          mxGetNumberOfElements(DEFVALUE_PARAM(S)) == 1);

//         if(isOk){
//             /* 
//              * Translate the mxClassId of an mxArray to one of Simulink's
//              * built-in data type ids. (see simulink.c)
//              */
//             DTypeId  dtype = ssGetDTypeIdFromMxArray(DEFVALUE_PARAM(S));
//             isOk = (dtype == SS_INT8  || 
//                     dtype == SS_INT16 || 
//                     dtype == SS_INT32);
//         }
        if(!isOk){   
            ssSetErrorStatus(S,"Second parameter must be an integer value");
            return;
        }      
    }
  }
#endif /* MDL_CHECK_PARAMETERS */

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
     /* See sfuntmpl_doc.c for more details on the macros below */

    ssSetNumSFcnParams(S, 2);  /* Number of expected parameters */
#if defined(MATLAB_MEX_FILE)
	if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
		mdlCheckParameters(S);
		if (ssGetErrorStatus(S) != NULL) {
			return;
		}
	} else {
        /* Return if number of expected != number of actual parameters */
        ssSetErrorStatus(S,"InputPort: Expected 2 parameters.");
		return;
	}
#endif

    ssSetSFcnParamTunable(S,DESC_IDX,false); /* Not tunable */
    ssSetSFcnParamTunable(S,DEFVALUE_IDX,false);   /* Not tunable */

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 1);

    if (!ssSetNumInputPorts(S, 0)) return;
    if (!ssSetNumOutputPorts(S,1)) return;

    ssSetOutputPortDataType(S, 0, SS_INT32 );

    ssSetOutputPortWidth(S, 0, 1);

    ssSetNumSampleTimes(S, 1);

    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 1);
    ssSetNumPWork(S, 0);

    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, 0);

}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);

}

#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */



#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
#ifndef MATLAB_MEX_FILE
    // the TC is set before InitializeSizes is called and cleared afterwards.
    if ( RTW::currentTC ) {
        char data[255];
        // name
        std::string name = ssGetModelName(S);

        // desc
        mxGetString(DESC_PARAM(S), data, 255);
        std::string desc = data;

        // default value
        IntPort* dport;
        if ( RTW::currentTC->ports()->getPort( name ) == 0 ) {
            dport = new IntPort( name, (int32_T)mxGetScalar( DEFVALUE_PARAM(S) ) );
            RTW::currentTC->ports()->addPort( *(dport->port) ).doc(desc);
        } else {
            dport = new IntPort( RTW::currentTC->ports()->getPortType< InputPort<int> >(name), (int32_T)mxGetScalar( DEFVALUE_PARAM(S) )); 
            if ( !dport->port ) {
                log(Error) <<"InputPort< int > "<< name << " has wrong type in TaskContext." <<endlog();
                ssSetErrorStatus(S,"InputPort: port has wrong type in current TaskContext.");
                delete dport;
                return;
            }
        }

        ssSetUserData(S, (void*) dport );
    }
    else
        ssSetErrorStatus(S,"InputPort: Could not locate current TaskContext.");
#endif
  }
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block. Generally outputs are placed in the output vector, ssGetY(S).
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    int32_T *y     = (int32_T*)ssGetOutputPortSignal(S,0);
    int32_T *iwork = ssGetIWork(S);

    UNUSED_ARG(tid); /* not used in single tasking mode */

#ifndef MATLAB_MEX_FILE
      IntPort* dport = static_cast<IntPort*>(ssGetUserData(S));
      dport->port->readNewest( dport->d );
#endif
      // If in Orocos, use the dataport itself, if in simulink, use the parameter.
#ifndef MATLAB_MEX_FILE
      *iwork = dport->d;
#else
      *iwork = (int32_T)mxGetScalar( DEFVALUE_PARAM(S) );
#endif

    // Always write state to output.
    *y = *iwork;
}



#define MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
#ifndef MATLAB_MEX_FILE
    IntPort* dport = static_cast<IntPort*>(ssGetUserData(S));
    if (dport && dport->cleanup ) {
        RTW::currentTC->ports()->removePort( dport->port->getName() );
        assert( RTW::currentTC->ports()->getPort( dport->port->getName() ) == 0 );
    }
    delete dport;
    ssSetUserData(S, 0 );
#endif
}

#undef MDL_RTW  /* Change to #undef to remove function */
#if defined(MDL_RTW) && defined(MATLAB_MEX_FILE)
  /* Function: mdlRTW =========================================================
   * Abstract:
   *
   *    This function is called when the Real-Time Workshop is generating
   *    the model.rtw file. In this method, you can call the following
   *    functions which add fields to the model.rtw file.
   *
   *       if (!ssWriteRTWWorkVect(S, vectName, nNames,
   *
   *                            name, size,   (must have nNames of these pairs)
   *                                 :
   *                           ) ) {
   *           return;  (error reporting will be handled by SL)
   *       }
   *       Notes:
   *         a) vectName must be either "RWork", "IWork" or "PWork"
   *         b) nNames is an int_T (integer), name is a const char_T* (const
   *            char pointer) and size is int_T, and there must be nNames number
   *            of [name, size] pairs passed to the function.
   *         b) intSize1+intSize2+ ... +intSizeN = ssGetNum<vectName>(S)
   *            Recall that you would have to set ssSetNum<vectName>(S)
   *            in one of the initialization functions (mdlInitializeSizes
   *            or mdlSetWorkVectorWidths).
   *
   *       See simulink/include/simulink.c for the definition (implementation)
   *       of this function, and ... no example yet :(
   *
   */
  static void mdlRTW(SimStruct *S)
  {
  }
#endif /* MDL_RTW */


/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifndef MATLAB_MEX_FILE
extern "C"
{
#endif
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
#ifndef MATLAB_MEX_FILE
}
#endif


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

#define S_FUNCTION_NAME  sfun_vproperty
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

#define OUT_IDX   1
#define OUT_PARAM(S) ssGetSFcnParam(S,OUT_IDX) /* Third parameter */

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
      /* Check 1st parameter: desc parameter */
      {
          int_T     nu;
          char_T    *nameStr;
          if (!mxIsChar(DESC_PARAM(S))) {
              ssSetErrorStatus(S,"Please provide a description as first "
                               "parameter");
              return;
          }
      }
#if 0
      /* Check 3d parameter: value */
      {
          if (!mxIsArray(OUT_PARAM(S)) ||
              mxGetNumberOfElements(OUT_PARAM(S)) == 0) {
              ssSetErrorStatus(S,"Second parameter to S-function must be an "
                               "array of real numbers with at least one element.");
              return;
          }
      }
#endif
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
        /* internal::Return if number of expected != number of actual parameters */
        ssSetErrorStatus(S,"Property: Expected 2 parameters.");
		return;
	}
#endif

    ssSetSFcnParamTunable(S,DESC_IDX,false); /* Not tunable */
    ssSetSFcnParamTunable(S,OUT_IDX,false);   /* Not tunable */

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 0)) return;
    //ssSetInputPortWidth(S, 0, 1);
    //ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     * See matlabroot/simulink/src/sfuntmpl_directfeed.txt.
     */
    //ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, mxGetNumberOfElements(OUT_PARAM(S)) ); // 1 d property.

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
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


#if defined(MATLAB_MEX_FILE)
# define MDL_SET_OUTPUT_PORT_WIDTH
static void mdlSetOutputPortWidth(SimStruct *S, int_T port,
                                  int_T outputPortWidth)
{
    if (outputPortWidth == 0) {
        ssSetErrorStatus(S,"Output width must be at least 1.");
        return;
    }
    ssSetOutputPortWidth(S,port,outputPortWidth);
#ifndef MATLAB_MEX_FILE
    static_cast<RTT::Property<std::vector<double> >*>(ssGetUserData(S))->set().resize(outputPortWidth, 0.0);
#endif
}
#endif


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

#ifndef MATLAB_MEX_FILE
namespace {
    struct PropData {
        PropData(Property<std::vector<double> >* pprop, bool cleanup )
            : prop(pprop), clean(cleanup)
        {}
        ~PropData() {
            if (clean) {
                RTW::currentTC->properties()->removeProperty( prop );
                delete prop;
            }
        }
        Property<std::vector<double> >* prop;
        bool clean;
    };
}
#endif



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
    // the TC is set before mdlStart is called and cleared afterwards.
    if ( RTW::currentTC ) {
        char data[255];
        // name
        std::string name = ssGetModelName(S);

        // desc
        mxGetString(DESC_PARAM(S), data, 255);
        std::string desc = data;

        // value
        // The port width takes precedence over OUT parameter width.
        int_T  ny     = ssGetOutputPortWidth(S,0);
        int_T  np     = mxGetNumberOfElements(OUT_PARAM(S));
        if (ny < np) {
            log(Warning) << "VProperty: setting outputportwidth to "<<np <<endlog();
            ny = np;
            ssSetOutputPortWidth(S, 0, ny);
        }
        std::vector<double> vect( ny, 0.0 );
        for (int i=0; i != ny && i != np; ++i)
            vect[i] = mxGetPr( OUT_PARAM(S) )[i];

        RTT::Property< std::vector<double> >* prop = 0;
        bool own_prop (false);
        if ( RTW::currentTC->properties()->find( name ) == 0 ) {
            prop = new RTT::Property< std::vector<double> >( name, desc, vect );
            RTW::currentTC->properties()->addProperty( prop );
            //own_prop = true; //See bug #579: Simulink component configuration procedure broken when using DeploymentComponent
        } else {
            prop = RTW::currentTC->properties()->getProperty<std::vector<double> >( name );
            if (!prop) {
                log(Error) <<"Property< std::vector<double> > "<< name << " has wrong type in TaskContext." <<endlog();
                ssSetErrorStatus(S,"VProperty has wrong type in current TaskContext.");
                return;
            }
        }
        assert(prop);
        prop->set( vect );
        ssSetUserData(S, (void*) new PropData(prop, own_prop) );

    }
    else
        ssSetErrorStatus(S,"VProperty: Could not locate current TaskContext.");
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
    int_T  i;
    real_T *y     = ssGetOutputPortRealSignal(S,0);
    int_T  ny     = ssGetOutputPortWidth(S,0);

    UNUSED_ARG(tid); /* not used in single tasking mode */

    // Always write state to outputs.
    for (i = 0; i < ny; i++) {
#ifndef MATLAB_MEX_FILE
        *y++ = static_cast<PropData*>(ssGetUserData(S))->prop->rvalue()[i];
#else
        *y++ = mxGetPr( OUT_PARAM(S) )[i];
#endif
    }
}



#undef MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */



#undef MDL_DERIVATIVES  /* Change to #undef to remove function */
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
     PropData* p = static_cast<PropData*>(ssGetUserData(S));
     delete p;
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


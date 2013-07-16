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

#define S_FUNCTION_NAME  sfun_vdataport
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

#ifndef MATLAB_MEX_FILE
namespace {
    typedef RTT::DataPort< std::vector<double> > VectorDataPort;

    struct VectorPort
    {
        VectorPort(std::string name, std::vector<double> init )
            : port( new VectorDataPort(name, init)), vect(init), cleanup(false) {} // See bug #579: Simulink component configuration procedure broken when using DeploymentComponent
        VectorPort(VectorDataPort* dp, std::vector<double> init )
            : port( dp ), vect(init), cleanup(false) {}
        ~VectorPort() {
            if (cleanup)
                delete port;
        }
        VectorDataPort* port;
        std::vector<double> vect;
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
          if (!mxIsArray(DEFVALUE_PARAM(S)) ||
              mxGetNumberOfElements(DEFVALUE_PARAM(S)) == 0) {
              ssSetErrorStatus(S,"Value parameter to S-function must be an "
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
        /* Return if number of expected != number of actual parameters */
        ssSetErrorStatus(S,"DataPort: Expected 2 parameters.");
		return;
	}
#endif

    ssSetSFcnParamTunable(S,DESC_IDX,false); /* Not tunable */
    ssSetSFcnParamTunable(S,DEFVALUE_IDX,false);   /* Not tunable */

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 1);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S,1)) return;

    ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
    ssSetOutputPortWidth(S, 0, DYNAMICALLY_SIZED);

    ssSetNumSampleTimes(S, 1);

    ssSetNumRWork(S, DYNAMICALLY_SIZED);
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
# define MDL_SET_INPUT_PORT_WIDTH
static void mdlSetInputPortWidth(SimStruct *S, int_T port,
                                 int_T inputPortWidth)
{
    if (inputPortWidth == 0) {
        ssSetErrorStatus(S,"Input width must be at least 1.");
        return;
    }
    ssSetInputPortWidth(S,port,inputPortWidth);
    ssSetOutputPortWidth(S,port,inputPortWidth);
}

# define MDL_SET_OUTPUT_PORT_WIDTH
static void mdlSetOutputPortWidth(SimStruct *S, int_T port,
                                  int_T outputPortWidth)
{
    if (outputPortWidth == 0) {
        ssSetErrorStatus(S,"Output width must be at least 1.");
        return;
    }
    ssSetInputPortWidth(S,port,outputPortWidth);
    ssSetOutputPortWidth(S,port,outputPortWidth);
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

    if (!ssGetInputPortConnected(S,0)) {
        ssWarning(S,"input is unconnected or grounded, "
                  "setting input/output width to 1");
#ifndef MATLAB_MEX_FILE
        log(Warning) << ": unconnected input, setting io port width to 1" <<endlog();
        log(Warning) << ": input width was "<<ssGetInputPortWidth(S, 0)
                     << "; output width was "<<ssGetOutputPortWidth(S, 0) <<endlog();
#endif
        ssSetInputPortWidth(S, 0, 1);
        ssSetOutputPortWidth(S, 0, 1);
    }

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
        // The port width takes precedence over DEFVALUE parameter width.
        int_T  nu     = ssGetInputPortWidth(S,0);
        int_T  ny     = ssGetOutputPortWidth(S,0);
        int_T  np     = mxGetNumberOfElements(DEFVALUE_PARAM(S));
        log(Info) << name<<" inputs: " << nu <<endlog();
        log(Info) << name<<" outputs: " << ny <<endlog();
        log(Info) << name<<" params: " << np <<endlog();

        if (ny < np) {
            log(Warning) << name << ": setting outputportwidth to paramwidth " << np <<endlog();
            ny = np;
            ssSetOutputPortWidth(S, 0, ny);
        }
        if (ny < nu) {
            log(Warning) << "VDataPort: setting outputportwidth to inputportwidth " << nu <<endlog();
            ny = nu;
            ssSetOutputPortWidth(S, 0, ny);
        }
            
        std::vector<double> vect( ny, 0.0 );
        for (int i=0; i != ny && i != np; ++i)
            vect[i] = mxGetPr( DEFVALUE_PARAM(S) )[i];

        VectorPort* dport;
        if ( RTW::currentTC->ports()->getPort( name ) == 0 ) {
            dport = new VectorPort( name, vect );
            RTW::currentTC->ports()->addPort( dport->port, desc );
        } else {
            dport = new VectorPort( RTW::currentTC->ports()->getPortType< VectorDataPort >(name), vect ); 
            if ( !dport->port ) {
                log(Error) <<"DataPort< std::vector<double> > "<< name << " has wrong type in TaskContext." <<endlog();
                ssSetErrorStatus(S,"DataPort: port has wrong type in current TaskContext.");
                delete dport;
                return;
            }
        }
        log(Info) << "Initialising port " << name << "." << endlog();
        dport->port->Set( vect );
        ssSetUserData(S, (void*) dport );

    }
    else
        ssSetErrorStatus(S,"DataPort: Could not locate current TaskContext.");
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
    InputRealPtrsType uPtrs  = ssGetInputPortRealSignalPtrs(S,0);
    real_T *y     = ssGetOutputPortRealSignal(S,0);
    int_T  ny     = ssGetOutputPortWidth(S,0);
    real_T *rwork = ssGetRWork(S);

    UNUSED_ARG(tid); /* not used in single tasking mode */

#ifndef MATLAB_MEX_FILE
      VectorPort* vport = static_cast<VectorPort*>(ssGetUserData(S));
      vport->port->Get( vport->vect );
#endif
      for (i = 0; i < ny; i++) {
          // If in Orocos, use the dataport itself, if in simulink, use the parameter.
          if ( ssGetInputPortConnected(S,0) ) {
              *rwork++ = *uPtrs[i];
#ifndef MATLAB_MEX_FILE
              // if input port connected, we publish our state
              vport->vect[i] = *uPtrs[i];
#endif
          } else {
              // if input port not connected, we read our state
#ifndef MATLAB_MEX_FILE
              *rwork++ = vport->vect[i];
#else
              *rwork++ = mxGetScalar( DEFVALUE_PARAM(S) );
#endif
          }
      }
#ifndef MATLAB_MEX_FILE
      if ( ssGetInputPortConnected(S,0) )
          vport->port->Set( vport->vect );
#endif

    // Always write state to outputs.
    for (i = 0; i < ny; i++) {
        *y++ = *rwork++;
    }
}

#if defined(MATLAB_MEX_FILE)
# define MDL_SET_WORK_WIDTHS
  static void mdlSetWorkWidths(SimStruct *S)
  {
      ssSetNumRWork(S, ssGetOutputPortWidth(S,0));
  }
#endif /* defined(MATLAB_MEX_FILE) */



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
      // Also update the state of the Orocos port.

      int_T             i;
      InputRealPtrsType uPtrs  = ssGetInputPortRealSignalPtrs(S,0);
      int_T             ny     = ssGetOutputPortWidth(S,0);
      real_T            *rwork = ssGetRWork(S);

      UNUSED_ARG(tid); /* not used in single tasking mode */

#ifndef MATLAB_MEX_FILE
      VectorPort* vport = static_cast<VectorPort*>(ssGetUserData(S));
      vport->port->Get( vport->vect );
#endif
      for (i = 0; i < ny; i++) {
          // If in Orocos, use the dataport itself, if in simulink, use the parameter.
          if ( ssGetInputPortConnected(S,0) ) {
              *rwork++ = *uPtrs[i];
#ifndef MATLAB_MEX_FILE
              // if input port connected, we publish our state
              vport->vect[i] = *uPtrs[i];
#endif
          } else {
              // if input port not connected, we read our state
#ifndef MATLAB_MEX_FILE
              *rwork++ = vport->vect[i];
#else
              *rwork++ = mxGetScalar( DEFVALUE_PARAM(S) );
#endif
          }
      }
#ifndef MATLAB_MEX_FILE
      if ( ssGetInputPortConnected(S,0) )
          vport->port->Set( vport->vect );
#endif
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
    VectorPort* vport = static_cast<VectorPort*>(ssGetUserData(S));
    if (vport && vport->cleanup ) {
        RTW::currentTC->ports()->removePort( vport->port->getName() );
        assert( RTW::currentTC->ports()->getPort( vport->port->getName() ) == 0 );
    }
    delete vport;
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


#ifndef OCL_RTW_COMPONENT_HPP
#define OCL_RTW_COMPONENT_HPP

extern "C"
{
#include "tmwtypes.h"
#include "rtmodel.h"
#include "rt_sim.h"
#include "rt_nonfinite.h"
#include "mdl_info.h"
#include "bio_sig.h"
#include "simstruc.h"
} // extern "C"

#define EXPAND_CONCAT(name1,name2)	name1 ## name2
#define CONCAT(name1,name2)		EXPAND_CONCAT(name1,name2)
#define RT_MODEL			CONCAT(MODEL,_rtModel)

#if TID01EQ == 1
#define FIRST_TID 1
#else
#define FIRST_TID 0
#endif
#ifndef RT
# error "must define RT"
#endif
#ifndef MODEL
# error "must define MODEL"
#endif
#ifndef NUMST
# error "must define number of sample times, NUMST"
#endif
#ifndef NCSTATES
# error "must define NCSTATES"
#endif

#ifndef SAVEFILE
# define MATFILE2(file) #file ".mat"
# define MATFILE1(file) MATFILE2(file)
# define MATFILE MATFILE1(MODEL)
#else
# define MATFILE QUOTE(SAVEFILE)
#endif

#define RUN_FOREVER		-1.0

/**
 * Allow other bases than TaskContext.
 */
#ifndef BASEHEADER
#define BASEHEADER <rtt/TaskContext.hpp>
#ifndef BASECLASS
#define BASECLASS TaskContext
#endif
#endif

#ifndef RT_MALLOC
#error "Use the RT_MALLOC flag for building components."
#endif


/*====================*
 * External functions *
 *====================*/
/**
 * extern 'C' Function declarations.
 */
extern "C" {

  extern RT_MODEL *MODEL(void);

#if !defined(RT_MALLOC)
  /**
   * This is the GRT compatible call interface for non-malloc
   * targets. The malloc target uses different functions, using
   * the model as function argument.
   */
  extern void MdlInitializeSizes(void);
  extern void MdlInitializeSampleTimes(void);
  extern void MdlStart(void);
  extern void MdlOutputs(int_T tid);
  extern void MdlUpdate(int_T tid);
  extern void MdlTerminate(void);
#endif

#ifndef RT_MALLOC
#if NCSTATES > 0
  extern void rt_ODECreateIntegrationData(RTWSolverInfo *si);
  extern void rt_ODEUpdateContinuousStates(RTWSolverInfo *si);

# define rt_CreateIntegrationData(S)			\
  rt_ODECreateIntegrationData(rtmGetRTWSolverInfo(S));
# define rt_UpdateContinuousStates(S)			\
  rt_ODEUpdateContinuousStates(rtmGetRTWSolverInfo(S));
# else
# define rt_CreateIntegrationData(S)					\
  rtsiSetSolverName(rtmGetRTWSolverInfo(S),"FixedStepDiscrete");
# define rt_UpdateContinuousStates(S) /* Do Nothing */
#endif

#else // !RT_MALLOC:

#if NCSTATES > 0
  extern void rt_ODECreateIntegrationData(RTWSolverInfo *si);
  extern void rt_ODEUpdateContinuousStates(RTWSolverInfo *si);
# if defined(RT_MALLOC)
  extern void rt_ODEDestroyIntegrationData(RTWSolverInfo *si);
# endif
#else
# define rt_ODECreateIntegrationData(si)	\
  rtsiSetSolverName(si, "FixedStepDiscrete");
# define rt_ODEUpdateContinuousStates(si)	\
  rtsiSetT(si,rtsiGetSolverStopTime(si))
#endif
#endif // !RT_MALLOC

} // extern "C"

#ifdef EXT_MODE
#  define rtExtModeSingleTaskUpload(S)                          \
  {								\
    int stIdx;							\
    rtExtModeUploadCheckTrigger(rtmGetNumSampleTimes(S));	\
    for (stIdx=0; stIdx<NUMST; stIdx++) {			\
      if (rtmIsSampleHit(S, stIdx, 0 /*unused*/)) {		\
	rtExtModeUpload(stIdx,rtmGetTaskTime(S,stIdx));		\
      }								\
    }								\
  }
#else
#  define rtExtModeSingleTaskUpload(S) /* Do nothing */
#endif

#define RTW_COMPONENT MODEL
#define RTW_xstr(s) RTW_str(s)
#define RTW_str(s) #s        

#include BASEHEADER
#include <rtt/Attribute.hpp>
#include "taskcontext/sfun_taskcontext.hpp"

namespace RTW
{
  using namespace RTT;

  struct RTWCreationException
  {};

  /**
   * The Auto-generated class for hosting a MatLab Simulink algorithm.
   * The \a RTW_COMPONENT classname is actually a macro which expands to
   * "<modelname>" and is thus unique for each model.
   */
  class RTW_COMPONENT
    : public BASECLASS
  {
  public:
    RTW_COMPONENT(const std::string& name)
      : BASECLASS(name, Stopped),
	rtM(0), 
	FinalTime(RUN_FOREVER)
    {
      log().in( RTW_xstr(RTW_COMPONENT) );
      // We need to create ports and properties asap.
      this->setup();
    }

    virtual ~RTW_COMPONENT()
    {
      if (this->getTaskState() == Running)
	this->stop();
      if (this->getTaskState() == Stopped)
	this->cleanup();
    }
    
    bool setup()
    {
      // initialize and return model.

      rt_InitInfAndNaN(sizeof(real_T));

      rtM = ::MODEL();
      if (rtM == NULL) {
	log(Fatal) << "Memory allocation error during target registration" <<endlog();
	return false;
      }
      if (rtmGetErrorStatus(rtM) != NULL) {
	log(Fatal) << "Error during target registration: " << rtmGetErrorStatus(rtM) <<endlog();
	return false;
      }

      if (FinalTime > 0.0 || FinalTime == RUN_FOREVER) {
	rtmSetTFinal(rtM, (real_T)FinalTime);
      }

#if !defined(RT_MALLOC)
      MdlInitializeSizes();
      MdlInitializeSampleTimes();
#else
      rtmiInitializeSizes(rtmGetRTWRTModelMethodsInfo(rtM));
      rtmiInitializeSampleTimes(rtmGetRTWRTModelMethodsInfo(rtM));
#endif

      log(Info) << "Simulink Target info" << endlog();;
      log(Info) << "====================" << endlog();;
      log(Info) << "  Model name             : " << RTW_xstr(MODEL) << endlog();
      log(Info) << "  Model base rate        : " << rtmGetStepSize(rtM) << endlog();
      log(Info) << "  Number of sample times : " << rtmGetNumSampleTimes(rtM) << endlog();
      for (int j = 0; j < rtmGetNumSampleTimes(rtM); j++) {
	log(Info) << "  Sample time "<< j <<"          : " << rtmGetSampleTimePtr(rtM)[j] << endlog();
      }


      status = rt_SimInitTimingEngine(rtmGetNumSampleTimes(rtM), rtmGetStepSize(rtM), rtmGetSampleTimePtr(rtM), rtmGetOffsetTimePtr(rtM), rtmGetSampleHitPtr(rtM), rtmGetSampleTimeTaskIDPtr(rtM), rtmGetTStart(rtM), &rtmGetSimTimeStep(rtM), &rtmGetTimingData(rtM));
      if (status != NULL) {
	log(Fatal) << "Failed to initialize target sample time engine:" << status <<endlog();
	return false;
      }
#ifdef RT_MALLOC
      rt_ODECreateIntegrationData(rtmGetRTWSolverInfo(rtM));
# if NCSTATES > 0
      if(rtmGetErrorStatus(rtM) != NULL) {
	(void)fprintf(stderr, "Error creating integration data.\n");
	this->cleanup();
      }
# endif
#else
      rt_CreateIntegrationData(rtM);
#endif

      // If we don't do this, we get a segfault in MdlStart().
      const char_T *errmsg = rt_StartDataLogging(rtmGetRTWLogInfo(rtM),
						 rtmGetTFinal(rtM),
						 rtmGetStepSize(rtM),
						 &rtmGetErrorStatus(rtM));
      if ( errmsg != NULL) {
	log(Fatal) << "Error starting data logging: " << errmsg << endlog();
	return false;
      }

      // yeah, it ain't thread-safe... The only alternative is to 'steal' a field
      // from the rtM struct and store 'this' inthere, hoping RTW will never overwrite it.
      assert( RTW::currentTC == 0 );
      RTW::currentTC = this;
      // On this point, the blocks are connected and
      // most data is initialised. In start() you can detect
      // connected ports etc and have the latest chance to allocate memory.
      // In orocos, in start() we create the ports, properties etc.
#if !defined(RT_MALLOC)
      MdlStart();
#else
      rtmiStart(rtmGetRTWRTModelMethodsInfo(rtM));
#endif
      if (rtmGetErrorStatus(rtM) != NULL) {
	return false;
      }

      RTW::currentTC = 0;

      status = rtmGetErrorStatus(rtM);
      if ( status != NULL) {
	log(Fatal) << "Failed in target initialization:" << status << endlog();
	// cleanup MdlStart() left-overs:
	this->cleanupHook();
	return false;
      }

      return true;
    }
    
      

    bool configureHook()
    {
      Logger::In in(RTW_xstr(RTW_COMPONENT));
      
      // detect re-configuration
      if ( this->getTaskState() == Stopped )
	this->cleanupHook(); // force deletion of old model data
             
      log(Info) << "Configuring " << this->getName() <<endlog();

      // Give base class a chance to pre-configure
      if ( BASECLASS::configureHook() == false){
	log(Error) << "Baseclass failed to configure" << endlog();
	return false;
      }

      // Since cleanupHook can reset the rtM pointer to zero, we have to rerun
      // setup to make sure rtM points to something real before continuing
      if (this->setup() == false){
	  log(Error) << "Setup returned false."<<endlog();
	  return false;
      }
      
      // If the user set a baserate, use that one and re-set all smaller sample times.
      // otherwise, create a matching periodic activity.
      double baserate = this->getActivity()->getPeriod();
      if ( baserate != rtmGetStepSize(rtM) ) {
	if ( baserate == 0.0 ) {
	  log(Warning) << "Simulink models must be executed at Periodic frequencies, but you've set a non periodic."<<endlog();
	  // return false;
	} else {
	  log(Warning) << "Orocos Activity Period = " << baserate << ", Simulink Fixed step Size = " << rtmGetStepSize(rtM) << " -> Forcing base rate to   : " << baserate << endlog();
	  rtmSetStepSize( rtM, baserate );
	  rtsiSetFixedStepSize( rtM->solverInfo, baserate );
	  for (int j = 0; j < rtmGetNumSampleTimes(rtM); j++) {
	    if ( rtmGetSampleTimePtr(rtM)[j] != 0.0 
		 && rtmGetSampleTimePtr(rtM)[j] < baserate ) {
	      log(Warning) << "  Forcing sample time "<< j <<" to: " << baserate << endlog();
	      rtmGetSampleTimePtr(rtM)[j] = baserate;
	    }
	  }
	}
      }

      return true;
      
    }

    virtual void cleanupHook()
    {
      assert( RTW::currentTC == 0 );
      RTW::currentTC = this;
      if ( rtM ) {
	rt_StopDataLogging(MATFILE,rtmGetRTWLogInfo(rtM));
	/* timing data */
	rt_SimDestroyTimingEngine(rtmGetTimingData(rtM));
#if NCSTATES > 0
	/* integration data */
	rt_ODEDestroyIntegrationData(rtmGetRTWSolverInfo(rtM));
#endif

#if !defined(RT_MALLOC)
	MdlTerminate();
#else
	rtmiTerminate(rtmGetRTWRTModelMethodsInfo(rtM));
#endif
      }
      RTW::currentTC = 0;
      rtM = 0;

      BASECLASS::cleanupHook();
    }

    virtual bool startHook()
    {
      // Check if model rate matches execution frequency.
      //             if ( this->engine()->getActivity() == 0 )
      //                 return false;
      //             if ( this->engine()->getActivity()->getPeriod() != rtmGetStepSize(rtM) )
      //                 return false;
      return  BASECLASS::startHook();
    }

    void updateHook()
    {
#ifndef RT_MALLOC
      // This snippet comes from grt_main.c, official mathworks example.
      // It is the !MULTI_TASKING setup

      /***********************************************
       * Check and see if error status has been set  *
       ***********************************************/

      if (rtmGetErrorStatus(rtM) != NULL) {
	this->stop();
	return;
      }

#ifdef EXT_MODE
      /*
       * In a multi-tasking environment, this would be removed from the base rate
       * and called as a "background" task.
       */
      rtExtModeOneStep(rtmGetRTWExtModeInfo(rtM),
		       rtmGetNumSampleTimes(rtM),
		       (boolean_T *)&rtmGetStopRequested(rtM));
#endif

      tnext = rt_SimGetNextSampleHit();
      rtsiSetSolverStopTime(rtmGetRTWSolverInfo(rtM),tnext);

      MdlOutputs(0);

#ifdef EXT_MODE
      rtExtModeSingleTaskUpload(rtM);

      GBLbuf.errmsg = rt_UpdateTXYLogVars(rtmGetRTWLogInfo(rtM),rtmGetTPtr(rtM));
      if (GBLbuf.errmsg != NULL) {
	GBLbuf.stopExecutionFlag = 1;
	return;
      }
#endif

      MdlUpdate(0);
      rt_SimUpdateDiscreteTaskSampleHits(rtmGetNumSampleTimes(rtM),
					 rtmGetTimingData(rtM),
					 rtmGetSampleHitPtr(rtM),
					 rtmGetTPtr(rtM));

      if (rtmGetSampleTime(rtM,0) == CONTINUOUS_SAMPLE_TIME) {
	rt_UpdateContinuousStates(rtM);
      }

#ifdef EXT_MODE
      rtExtModeCheckEndTrigger();
#endif
#else // RT_MALLOC

#ifdef EXT_MODE
      rtExtModeOneStep(rtmGetRTWExtModeInfo(rtM),
		       rtmGetNumSampleTimes(rtM),
		       (boolean_T *)&rtmGetStopRequested(rtM));
#endif

      tnext = rt_SimGetNextSampleHit(rtmGetTimingData(rtM),
				     rtmGetNumSampleTimes(rtM));
      rtsiSetSolverStopTime(rtmGetRTWSolverInfo(rtM),tnext);

      rtmiOutputs(rtmGetRTWRTModelMethodsInfo(rtM),0);

#ifdef EXT_MODE
      rtExtModeSingleTaskUpload(rtM);

      GBLbuf.errmsg = rt_UpdateTXYLogVars(rtmGetRTWLogInfo(rtM),
					  rtmGetTPtr(rtM));
      if (GBLbuf.errmsg != NULL) {
	GBLbuf.stopExecutionFlag = 1;
	return;
      }
#endif

      rtmiUpdate(rtmGetRTWRTModelMethodsInfo(rtM),0);

      rt_SimUpdateDiscreteTaskSampleHits(rtmGetNumSampleTimes(rtM),
					 rtmGetTimingData(rtM),
					 rtmGetSampleHitPtr(rtM),
					 rtmGetTPtr(rtM));

      if (rtmGetSampleTime(rtM,0) == CONTINUOUS_SAMPLE_TIME) {
	rt_ODEUpdateContinuousStates(rtmGetRTWSolverInfo(rtM));
      }

      //             GBLbuf.isrOverrun--;

#ifdef EXT_MODE
      rtExtModeCheckEndTrigger();
#endif

#endif
    }

    RT_MODEL *rtM;
    float FinalTime;
    const char *status;
    real_T tnext;
  };
}

#endif

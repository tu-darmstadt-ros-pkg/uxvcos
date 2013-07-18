/***************************************************************************
****************************************************************************
*
*       Humusoft data acquisition library.
*
*       Copyright 2002-2008 Humusoft s.r.o.
*       Copyright 2009-2010 J.Fojtik
*
******************************************************************************
*****************************************************************************/

#ifndef HUDAQLIB_H__
#define HUDAQLIB_H__


/* All the functions have C naming convention in C++. */
#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */


/* Define calling conventions. */
#ifdef _WIN32
  #define HUDAQLIBPUBLIC __declspec(dllimport)
  #ifdef _WIN64
    #define HUDAQAPI
  #else
    #define HUDAQAPI __stdcall
  #endif
#else
  #define HUDAQLIBPUBLIC
  #define HUDAQAPI
#endif



/** Return codes for HUDAQ functions.
 * @ingroup GroupConstants
*/
typedef enum
{
  HUDAQSUCCESS = 0,     ///< Success. @ifnot UNDOCUMENTED All other values mean failure. @endif
/** @cond UNDOCUMENTED */
  HUDAQPARTIAL = 1,     ///< Several A/D or D/A operation succeeded and at least one failed, see ::HudaqAOWriteMultiple.
  HUDAQFAILURE = 2,     ///< Unspecified problem occured (e.g. hardware failure).
  HUDAQBADARG = 3,      ///< Unexpected arguments.
  HUDAQNOTSUPPORTED = 4 ///< Device does not support this functionality.
/** @endcond UNDOCUMENTED */
} HUDAQSTATUS;


/** Subsystem identifiers.
 * Used to identify individual subsystems of the board.
 * @ingroup GroupChannelConfig */
typedef enum
{
  HudaqAI  = 0x1000,              ///< Analog Input
  HudaqAO  = 0x2000,              ///< Analog Output
  HudaqDI  = 0x3000,              ///< Digital Input
  HudaqDO  = 0x4000,              ///< Digital Output
  HudaqEnc = 0x5000,              ///< Encoder
  HudaqCtr = 0x6000,              ///< Counter
  HudaqPWM = 0x7000,              ///< Pulse-Width Modulation Output
/** @cond UNDOCUMENTED */
  HudaqStep= 0x8000,              ///< Stepper motor
  HudaqIRQ = 0x9000,		  ///< IRQ subsystem
  HudaqSubsystemMASK = 0xFFFFF000 ///< Mask that separates subsystem number and its parameter.
/** @endcond UNDOCUMENTED */
} HudaqSubsystem;


/** Parameter identifiers. They are used as the third parameter to
 *  ::HudaqGetParameter and ::HudaqSetParameter functions to specify which parameter should be
 * configured. Please note that a particular
 * device does not neccesarily support all functions.
 *
 * @ingroup GroupChannelConfig
 */
typedef enum
{
     /* Parameter names for Analog Input functions. */
/** @cond UNDOCUMENTED */
  HudaqAIUNITS = HudaqAI,       ///< Set units of channel. See ::HudaqAIOUnits.
/** @endcond UNDOCUMENTED */
  HudaqAIRANGE,                 ///< @brief Select analog input voltage range. @brief The range is selected by its index;
                                ///< the actual voltages corresponding to the range can be obtained by calling ::HudaqQueryRange. @par
/** @cond UNDOCUMENTED */
  HudaqAINUMCHANNELS,           ///< Get number of analog input channels.
/** @endcond UNDOCUMENTED */

     /* Parameter names for Analog Output functions. */
/** @cond UNDOCUMENTED */
  HudaqAOUNITS = HudaqAO,       ///< Set units of channel. See ::HudaqAIOUnits.
/** @endcond UNDOCUMENTED */
  HudaqAORANGE,                 ///< @brief Select analog output voltage range. @brief The range is selected by its index;
                                ///< the actual voltages corresponding to the range can be obtained by calling ::HudaqQueryRange. @par
/** @cond UNDOCUMENTED */
  HudaqAONUMCHANNELS,           ///< Get number of analog output channels.
/** @endcond UNDOCUMENTED */

     /* Parameter names for Digital Output functions. */
/** @cond UNDOCUMENTED */
  HudaqDINUMCHANNELS = HudaqDI, ///< Get number of digital input channels.
  HudaqDINUMBITS,               ///< Get number of bits in one digital output channel. @par
/** @endcond UNDOCUMENTED */

     /* Parameter names for Digital Output functions. */
/** @cond UNDOCUMENTED */
  HudaqDONUMCHANNELS = HudaqDO, ///< Get number of digital output channels.
  HudaqDONUMBITS,               ///< Get number of bits in one digital output channel. @par
  HudaqDOMODE,			///< 0 set channel as input, 1 set channel as output
/** @endcond UNDOCUMENTED */

     /* Parameter names for Encoder Input functions. */
  HudaqEncRESETONREAD = HudaqEnc,///< Automatically reset encoder pulse count after it is read; possible values are 0 (off) or 1 (on).
  HudaqEncFILTER,               ///< Filter encoder inputs with a lowpass filter; possible values are 0 (off) or 1 (on).
  HudaqEncMODE,                 ///< Encoder mode; for possible values see ::HudaqEncMode.
  HudaqEncCOUNTCONTROL,         ///< Encoder count control; for possible values see ::HudaqEncCountControl.
  HudaqEncRESETMODE,            ///< Encoder reset mode; for possible values see ::HudaqEncResetMode. @par
/** @cond UNDOCUMENTED */
  HudaqEncI,                    ///< Read @a I signal. Only ::HudaqGetParameter is supported; option type int.
  HudaqEncNUMCHANNELS,          ///< Get number of encoder channels.
/** @endcond UNDOCUMENTED */

     /* Parameter names for Counter Input functions. */
  HudaqCtrRESETONREAD = HudaqCtr,///< Automatically reset counter pulse count after it is read; possible values are 0 (off) or 1 (on).
  HudaqCtrCLOCKSOURCE,          ///< Counter clock source; for possible values see ::HudaqCtrClockSource.
  HudaqCtrOUTPUTCONTROL,	///< Counter output signal; for possible values see ::HudaqCtrOutputControl
  HudaqCtrREPETITION,		///< 0  - counter stops after terminal count; 1 - counter reloads and continues counting.
  HudaqCtrLOADTOGGLE,		///< 0 - always load from register A; 1 - alternate load registers A and B.
  HudaqCtrDIRECTION,		///< 0 - counter counts down; 1 - counter counts up.
  HudaqCtrOUTTOGGLE,	        ///< 0 - output is directly connected to TC; 1 - use flipflop that is toggled on every TC.
  HudaqCtrTRIGSOURCE,		///< Counter trigger source; for possible values see ::HudaqCtrTrigSource
  HudaqCtrTRIGTYPE,		///< Counter trigger edge; for possible values see ::HudaqCtrTrigType
  HudaqCtrRETRIGGER,		///< Counter can be retriggered: 0 - only when stopped; 1 - anytime
  HudaqCtrGATESOURCE,		///< Counter gate source; for possible values see ::HudaqCtrGateSource
  HudaqCtrGATEPOLARITY,         ///< Counter gate polarity: 0 - gate low disables counting; 1 - gate high disables counting.
  HudaqCtrFILTER,               ///< Filter counter inputs with a lowpass filter; possible values are 0 (off) or 1 (on). @par
/** @cond UNDOCUMENTED */
  HudaqCtrNUMCHANNELS,          ///< Get number of counter channels. @par
/** @endcond UNDOCUMENTED */

     /* Parameter names for PWM Output functions. */
/** @cond UNDOCUMENTED */
  HudaqPwmTHRESHOLD = HudaqPWM, ///< If you change PWM from slow frequency to fast, the last period for old frequency is in progress.
				///< e.g. if you switch from 1Hz to 1MHz you would wait up to 1s to change generated frequency.
			        ///< Frequencies below threshold are fixed for this artefact. Used algorithm ensures that a rest of
				///< pulse will not be larger than a period of new frequency.
  HudaqPwmNUMCHANNELS,          ///< Get number of PWM channels.
  HudaqPwmINPUT,                ///< Read @a Input signal. Only ::HudaqGetParameter is supported.
/** @endcond UNDOCUMENTED */
  HudaqPwmPHASES,		///< Read bits that coresponds to output phases. Only ::HudaqGetParameter is supported. (MF625 only)
  HudaqPwmUPDOWN,		///< Read UpDown flag. Only ::HudaqGetParameter is supported. (MF625 only)
  HudaqPwmINVERSIONS,		///< Read bits, that corresponds to inversions in all phases. (MF625 only)
  HudaqPwmDEADBAND,		///< Set dead band between phase and inverted phase  (MF625 only)
  HudaqPwmOUTPUTCONTROL,        ///< Output phase signal; for possible values see ::HudaqCtrOutputControl
  HudaqPwmCLOCKSOURCE,          ///< Counter clock source; for possible values see ::HudaqCtrClockSource.
  HudaqPwmFILTER,               ///< Filter counter inputs with a lowpass filter; possible values are 0 (off) or 1 (on).
  HudaqPwmGATESOURCE,		///< Counter gate source; for possible values see ::HudaqCtrGateSource
  HudaqPwmTRANSPARENT,          ///< 0 - Dual buffer is turned on, 1 - Dual buffer is turned off. (MF625 only)
  HudaqPwmEMERGENCY,		///< Define condition to block PWM output; for possible values see ::HudaqPwmEmergency (MF625 only)
  HudaqPwmGATEPOLARITY,         ///< Counter gate polarity: 0 - gate low disables counting; 1 - gate high disables counting.
  HudaqPwmOUTPUTUDCONTROL,      ///< Output up/down signal; for possible values see ::HudaqCtrOutputControl  @par

     /* Parameter names for Stepper Output functions. */
/** @cond UNDOCUMENTED */
  HudaqStepFMIN = HudaqStep,    ///< Minimal speed of stepping motor [steps/s]; option type double.
  HudaqStepFMAX,                ///< Maximal speed of stepping motor [steps/s]; option type double.
  HudaqStepACCELERATION,        ///< Acceleration of stepping motor [steps/s^2]; option type double.
  HudaqStepCURRENTPOSITION,     ///< Current position in steps; option type int.
  HudaqStepREMAININGSTEPS,      ///< Amount of unrealised steps. Read only. Option type int.
  HudaqStepTARGETPOSITION,      ///< Target position in steps. Read only. Option type int.
  HudaqStepNUMCHANNELS          ///< Get number of stepper channels. @par
/** @endcond UNDOCUMENTED */

} HudaqParameter;


/** @cond UNDOCUMENTED */
/* lowercase names for backward compatibility */
typedef enum
{
  HudaqAIRange = HudaqAIRANGE,
  HudaqAORange = HudaqAORANGE
} HudaqAIORange;
/** @endcond UNDOCUMENTED */


/** @cond UNDOCUMENTED */
/* this is documented elsewhere to avoid generation of class hierarchy */
/** Structure that contains one range record with low and high limit.
 *
 * @ingroup GroupChannelConfig
 */
typedef struct
{
  double Lo;   ///< Low range limit.
  double Hi;   ///< Target position in steps. Read only. Option type int.
} HudaqRange;
/** @endcond UNDOCUMENTED */


/**
 * The HUDAQ device handle data type.
 * @ingroup GroupConstants
 */
typedef size_t HUDAQHANDLE;



/** @cond UNDOCUMENTED */

/**
 * Resource range structure.
 * @ingroup GroupConstants
 */
typedef struct
{
  size_t Base;                        ///< Base addres/IO port.
  size_t Length;                      ///< Amount resources.
} HudaqResourceRange;


/**
 * Resource information structure. Please note that every device has totally
 * different meaning of this structure.
 * @ingroup GroupConstants
 */
typedef struct
{
  int NumMemResources;                ///< Amount of memory mapped ranges in particular device.
  HudaqResourceRange MemResources[8]; ///< Memory resources available for user access.
  int NumIOResources;                 ///< Amount of IO ranges in particular device.
  HudaqResourceRange IOResources[8];  ///< IO resources available for user access.
  unsigned BusNumber;
  unsigned SlotNumber;
  unsigned VendorID;
  unsigned DeviceID;
  unsigned PropAddr;
} HudaqResourceInfo;


/** Options for ::HudaqOpenDevice parameter options
 * @ingroup GroupConstants
 */
typedef enum
{
  HudaqOpenRESET = 0,         ///< Force initialization of hardware.
  HudaqOpenNOINIT = 1,        ///< Skip initialization of device. Normally
                              ///< you want initialization. This is reserved for multiple
                              ///< access to device from different application or device enumeration.
  //HudaqOpenForceInit = 2
} HudaqOpenOptions;

/** @endcond UNDOCUMENTED */


/**
 * Open a data acquisition device.
 *   The device is put into initial state when being opened.
 * @ingroup GroupBoardInitialization
 * @param[in] devicename Device name.
 *   This parameter is the device type as a string, for example "MF624".
 * @param[in] deviceorder Device order.
 *   One-based index to distinguish between devices of identical type.
 * @param[in] options Reserved, must be zero.
 * @return Device handle or zero on failure.
 */
HUDAQLIBPUBLIC HUDAQHANDLE HUDAQAPI HudaqOpenDevice(const char* devicename, int deviceorder, int options);


/**
 * Reset a data acquisition device.
 *   This function puts the device into initial state.
 * @ingroup GroupBoardInitialization
 * @param[in] handle Device handle.
 * @return ::HUDAQSUCCESS on success, other values on failure.
 */
HUDAQLIBPUBLIC HUDAQSTATUS HUDAQAPI HudaqResetDevice(HUDAQHANDLE handle);


/**
 * Close a data acquisition device handle.
 *   The device handle becomes invalid after this call and must not be used any more.
 *   The device state is not changed when its handle is closed. If it is required that the
 *   device is set to a specific state before closing the application, it must be done explicitly
 *   before calling this function.
 * @ingroup GroupBoardInitialization
 * @param[in] handle Device handle.
 */
HUDAQLIBPUBLIC void HUDAQAPI HudaqCloseDevice(HUDAQHANDLE handle);


/** @cond UNDOCUMENTED */

/**
 * Get device resources.
 * @ingroup GroupBoardInitialization
 * @param[in] handle Device handle.
 * @return Pointer to the resource information structure for the device, zero on error.
*/
HUDAQLIBPUBLIC const HudaqResourceInfo* HUDAQAPI HudaqGetDeviceResources(HUDAQHANDLE handle);

/** @endcond UNDOCUMENTED */


/** This procedure returns full device name. */
HUDAQLIBPUBLIC const char* HUDAQAPI HudaqGetBoardName(HUDAQHANDLE handle);


#ifdef _WIN32
/** This procedure returns system handle to a kernel driver. */
HUDAQLIBPUBLIC HUDAQSTATUS HUDAQAPI HudaqWaitForIrq(HUDAQHANDLE handle, int TimeOut);
#endif


/** @cond UNDOCUMENTED */

/** Possible values for HudaqAOUNITS. Used in both Analog inputs and Analog outputs channels.
 * See ::HudaqAIUNITS and ::HudaqAOUNITS.
 * @ingroup GroupChannelConfig
 */
typedef enum
{
  HudaqAIOVolts = 0,            ///< Use analog values in volts.
  HudaqAIORaw  = 1              ///< Raw value of A/D converter.
} HudaqAIOUnits;

/** @endcond UNDOCUMENTED */



/**
 * Read data from a single analog input channel.
 * @ingroup GroupAnalogInput
 * @param[in] handle  Device handle.
 * @param[in] channel Analog input channel number.
 * @return Value read from the analog input channel.
*/
HUDAQLIBPUBLIC double HUDAQAPI HudaqAIRead(HUDAQHANDLE handle, unsigned channel);

/**
 * Read data from multiple analog input channels.
 *  The analog input channels are read with the minimum possible interval between individual
 *  channels or simultaneously if the device supports this feature.
 * @ingroup GroupAnalogInput
 * @param[in] handle   Device handle.
 * @param[in] number   Number of channels to read.
 * @param[in] channels Array of channel numbers that will be read.
 *   The array must contain the specified number of channel numbers.
 * @param[out] values  Pointer to the array to be filled with values read from the analog input channels.
 *   The array is allocated by the caller and must contain enough space to hold all the values read.
 * @return ::HUDAQSUCCESS on success, other values on failure.
*/
HUDAQLIBPUBLIC HUDAQSTATUS HUDAQAPI HudaqAIReadMultiple(HUDAQHANDLE handle, unsigned number, const unsigned* channels, double* values);


/**
 * Write data to a single analog output channel.
 * @ingroup GroupAnalogOutput
 * @param[in] handle  Device handle.
 * @param[in] channel Analog output channel number.
 * @param[in] value   Value to write to the analog output channel.
 */
HUDAQLIBPUBLIC void HUDAQAPI HudaqAOWrite(HUDAQHANDLE handle, unsigned channel, double value);


/**
 * Write data to multiple analog output channels.
 *  The analog output channels are updated with the minimum possible interval between individual
 *  channels or simultaneously if the device supports this feature.
 * @ingroup GroupAnalogOutput
 * @param[in] handle   Device handle.
 * @param[in] number   Number of channels to be written.
 * @param[in] channels Array of channel numbers that will be written.
 *   The array must contain the specified number of channel numbers.
 * @param[in] values   Array of values to be written to the analog output channels.
 *   The array must contain the specified number of values.
 * @return ::HUDAQSUCCESS on success, other values on failure.
 */
HUDAQLIBPUBLIC HUDAQSTATUS HUDAQAPI HudaqAOWriteMultiple(HUDAQHANDLE handle, unsigned number, const unsigned* channels, const double* values);


/**
 * Read a single bit from a digital input channel.
 * @ingroup GroupDigitalInput
 * @param[in] handle  Device handle.
 * @param[in] channel Digital input channel number.
 * @param[in] bit     Bit position in the channel.
 * @return Bit value read from the specified bit.
 */
HUDAQLIBPUBLIC int HUDAQAPI HudaqDIReadBit(HUDAQHANDLE handle, unsigned channel, unsigned bit);

/**
 * Read data from a single digital input channel.
 *  The number of bits in a digital input channel is device specific.
 * @ingroup GroupDigitalInput
 * @param[in] handle  Device handle.
 * @param[in] channel Digital input channel number.
 * @return Value read from the digital input channel.
 */
HUDAQLIBPUBLIC int HUDAQAPI HudaqDIRead(HUDAQHANDLE handle, unsigned channel);

/**
 * Read data from multiple digital input channels.
 * @ingroup GroupDigitalInput
 * @param[in] handle   Device handle.
 * @param[in] number   Number of channels to read.
 * @param[in] channels Array of channel numbers that will be read.
 *   The array must contain the specified number of channel numbers.
 * @param[out] values  Pointer to the array to be filled with values read from the digital input channels.
 *   The array is allocated by the caller and must contain enough space to hold all the values read.
 * @return ::HUDAQSUCCESS on success, other values on failure.
 */
HUDAQLIBPUBLIC HUDAQSTATUS HUDAQAPI HudaqDIReadMultiple(HUDAQHANDLE handle, unsigned number, const unsigned* channels, unsigned* values);


/**
 * Write data to a single bit of a digital output channel.
 * @ingroup GroupDigitalOutput
 * @param[in] handle  Device handle.
 * @param[in] channel Digital output channel number.
 * @param[in] bit     Bit position in channel specified
 * @param[in] value   Bit value to be written to the specified bit.
 */
HUDAQLIBPUBLIC void HUDAQAPI HudaqDOWriteBit(HUDAQHANDLE handle, unsigned channel, unsigned bit, int value);

/**
 * Write data to a single digital output channel.
 *  The number of bits in a digital output channel is device specific.
 * @ingroup GroupDigitalOutput
 * @param[in] handle  Device handle.
 * @param[in] channel Digital output channel number.
 * @param[in] value   Value to be written into digital output.
 * @return ::HUDAQSUCCESS on success, other values on failure.
 */
HUDAQLIBPUBLIC HUDAQSTATUS HUDAQAPI HudaqDOWrite(HUDAQHANDLE handle, unsigned channel, unsigned value);

/**
 * Modify multiple bits in a single digital output channel.
 *   All the bits are modified simultaneously.
 * @ingroup GroupDigitalOutput
 * @param[in] handle  Device handle.
 * @param[in] channel Digital output channel number.
 * @param[in] mask    Mask that specifies bits that will be modified. Bits that are 1 will be assigned the corresponding bits of @c value,
 *                    bits that are 0 will be left untouched.
 * @param[in] value   Value to write. Only the bits specified by @c mask are modified, the other bits are ignored.
*/
HUDAQLIBPUBLIC void HUDAQAPI HudaqDOWriteMultipleBits(HUDAQHANDLE handle, unsigned channel, unsigned mask, unsigned value);


/**
 * Write data to multiple digital output channels.
 * @ingroup GroupDigitalOutput
 * @param[in] handle   Device handle.
 * @param[in] number   Number of channels to be written.
 * @param[in] channels Array of channel numbers that will be written.
 *   The array must contain the specified number of channel numbers.
 * @param[in] values   Array of values to be written to the digital output channels.
 *   The array must contain the specified number of values.
 * @return ::HUDAQSUCCESS on success, other values on failure.
*/
HUDAQLIBPUBLIC HUDAQSTATUS HUDAQAPI HudaqDOWriteMultiple(HUDAQHANDLE handle, unsigned number, const unsigned* channels, const unsigned* values);


/**
 * Counter clock sources.
 * @ingroup GroupChannelConfig
 */
typedef enum
{
  HudaqCtrCLOCK50MHz = 0,        ///< Internal clock 50MHz.                 (MF624 and MF625)
  HudaqCtrCLOCK10MHz = 1,        ///< Internal clock 10MHz.                 (MF624 and MF625)
  HudaqCtrCLOCK1MHz = 2,         ///< Internal clock 1MHz.                  (MF624 and MF625)
  HudaqCtrCLOCK100kHz = 3,       ///< Internal clock 100kHz.                (MF624 and MF625)
  HudaqCtrCLOCKINRISING = 5,     ///< External input, count on rising edge. (MF614, MF624 and MF625)
  HudaqCtrCLOCKINFALLING = 6,    ///< External input, count on falling edge.(MF614, MF624 and MF625)
  HudaqCtrCLOCKINEITHER = 7,     ///< External input, count on either edge. (MF624 and MF625)
  HudaqCtrCLOCKPREVRISING = 9,   ///< Output of previous counter, count on rising edge.  (MF614, MF624 and MF625)
  HudaqCtrCLOCKPREVFALLING = 10, ///< Output of previous counter, count on falling edge. (MF614, MF624 and MF625)
  HudaqCtrCLOCKPREVEITHER = 11,  ///< Output of previous counter, count on either edge.  (MF624 and MF625)
  HudaqCtrCLOCKNEXTRISING = 13,  ///< Output of next counter, count on rising edge.      (MF624 and MF625)
  HudaqCtrCLOCKNEXTFALLING = 14, ///< Output of next counter, count on falling edge.     (MF624 and MF625)
  HudaqCtrCLOCKNEXTEITHER = 15,  ///< Output of next counter, count on either edge.      (MF624 and MF625)
  HudaqCtrCLOCK20MHz,		 ///< Internal clock 20MHz.                 (MF614 only)
  HudaqCtrCLOCK2MHz,		 ///< Internal clock 2MHz.                  (MF614 only)
  HudaqCtrCLOCK200kHz,		 ///< Internal clock 200kHz.                (MF614 only)
  HudaqCtrCLOCK20kHz,		 ///< Internal clock 20kHz.                 (MF614 only)
  HudaqCtrCLOCK2kHz		 ///< Internal clock 2kHz.                  (MF614 only)
} HudaqCtrClockSource;


/**
 * Counter output control.
 * @ingroup GroupChannelConfig
 */
typedef enum
{
  HudaqCtrOUTPUTNORMAL = 0,	///< Normal counter output.			          (MF614 and MF624)
  HudaqCtrOUTPUTINVERTED = 1,	///< Inverted counter output.			          (MF624 only)
  HudaqCtrOUTPUT_0 = 2,		///< Force 0 on output, counter is still counting.        (MF624 only)
  HudaqCtrOUTPUT_1 = 3          ///< Force 1 on output,counter is still counting.         (MF624 only)
} HudaqCtrOutputControl;


/**
 * Counter trigger source.
 * @ingroup GroupChannelConfig
 */
typedef enum
{
  HudaqCtrTRIGDISABLE = 0,	///< Trigger is disabled.
  HudaqCtrTRIGINPUT = 1,	///< Trigger by counter input (TxIn).
  HudaqCtrTRIGPREV = 2,	        ///< Trigger by previous counter output.
  HudaqCtrTRIGNEXT = 3 	        ///< Trigger by next counter output.
} HudaqCtrTrigSource;


/**
 * Counter trigger type.
 * @ingroup GroupChannelConfig
 */
typedef enum
{
  HudaqCtrTRIGNONE = 0,		///< Trigger is inactive.
  HudaqCtrTRIGRISING = 1,	///< Trigger by rising edge of trigger signal.
  HudaqCtrTRIGFALLING = 2,	///< Trigger by falling edge of trigger signal.
  HudaqCtrTRIGEITHER = 3 	///< Trigger by either edge of trigger signal.
} HudaqCtrTrigType;


/**
 * Counter gate source.	 Please note ::HudaqCtrGATEPOLARITY for full undrestanding
   gate functionality.
 * @ingroup GroupChannelConfig
 */
typedef enum
{
  HudaqCtrGATEHIGH = 0,		///< Gate is forced to logical high.	(MF614, MF624 and MF625)
  HudaqCtrGATEINPUT = 1,	///< Gate by counter input (TxIn).	(MF614, MF624 and MF625)
  HudaqCtrGATEPREV = 2,	        ///< Gate by previous counter output.	(MF624 and MF625)
  HudaqCtrGATENEXT = 3 	        ///< Gate by next counter output.	(MF624 and MF625)
} HudaqCtrGateSource;


/**
 * Reset counter pulse count.
 * If the counter hardware is used by another subsystem, it is switched to counting mode and
 * the default input is selected. If the counter is already in counting mode, its input selection
 * is not changed.
 * @ingroup GroupCounterInput
 * @param[in] handle  Device handle.
 * @param[in] channel Number of counter channel.
 */
HUDAQLIBPUBLIC void HUDAQAPI HudaqCtrReset(HUDAQHANDLE handle, unsigned channel);


/**
 * Read counter pulse count.
 * The returned value is the number of pulses counted by the counter since reset.
 * @ingroup GroupCounterInput
 * @param[in] handle  Device handle.
 * @param[in] channel Counter input channel number.
 * @return Value read from the counter input channel.
 */
HUDAQLIBPUBLIC int HUDAQAPI HudaqCtrRead(HUDAQHANDLE handle, unsigned channel);


/** Encoder counting modes.
 * @ingroup GroupChannelConfig
 */
typedef enum
{
  HudaqEncMODEIRC = 0,     ///< Count IRC pulses on inputs @a A and @a B (MF614, MF624 and MF625)
  HudaqEncMODERISING,      ///< Count pulses on @a A, rising edge, @a B specifies direction (MF614, MF624 and MF625)
  HudaqEncMODEFALLING,     ///< Count pulses on @a A, falling edge, @a B specifies direction (MF624 and MF625)
  HudaqEncMODEEITHER       ///< Count pulses on @a A, either edge, @a B specifies direction (MF624 and MF625)
} HudaqEncMode;


/** Encoder count control.
 * Encoder count control allows enabling or disabling pulse counting based on software or hardware conditions.
 * @ingroup GroupChannelConfig
 */
typedef enum
{
  HudaqEncCOUNTENABLE = 0, ///< Encoder counting is enabled. (MF614, MF624 and MF625)
  HudaqEncCOUNTDISABLE,    ///< Encoder counting is disabled. (MF624 and MF625)
  HudaqEncCOUNTI0,         ///< Encoder counting is enabled when input @a I is at logical low, disabled when input @a I is at logical high. (MF624 and MF625)
  HudaqEncCOUNTI1          ///< Encoder counting is enabled when input @a I is at logical high, disabled when input @a I is at logical low. (MF624 and MF625)
} HudaqEncCountControl;


/** Encoder reset mode.
 * Encoder reset mode allows enabling the functionality of resetting the encoder pulse count by external signal.
 * @ingroup GroupChannelConfig
 */
typedef enum
{
  HudaqEncRESNONE = 0,     ///< Encoder is not being reset by external signal. (MF614, MF624 and MF625)
  HudaqEncRESPERMANENT,    ///< Encoder is permanently reset; the encoder does not count in this mode. (MF624 only)
  HudaqEncRESI0,           ///< Reset encoder pulse count when input @a I is at logical low. (MF614, MF624 and MF625)
  HudaqEncRESI1,           ///< Reset encoder pulse count when input @a I is at logical high. (MF624 and MF625)
  HudaqEncRESIRISING,      ///< Reset encoder pulse count on rising edge of input @a I . (MF614, MF624 and MF625)
  HudaqEncRESIFALLING,     ///< Reset encoder pulse count on falling edge of input @a I . (MF614, MF624 and MF625)
  HudaqEncRESIEITHER       ///< Reset encoder pulse count on either edge of input @a I . (MF624 and MF625)
} HudaqEncResetMode;



/**
 * Reset encoder pulse count.
 * @ingroup GroupEncoderInput
 * @param[in] handle  Device handle.
 * @param[in] channel Encoder input channel number.
 */
HUDAQLIBPUBLIC void HUDAQAPI HudaqEncReset(HUDAQHANDLE handle, unsigned channel);


/**
 * Read encoder pulse count.
 * The returned value is the number of pulses counted by the encoder since reset.
 * @ingroup GroupEncoderInput
 * @param[in] handle  Device handle.
 * @param[in] channel Encoder input channel number.
 * @return Value read from the encoder input channel.
 */
HUDAQLIBPUBLIC int HUDAQAPI HudaqEncRead(HUDAQHANDLE handle, unsigned channel);


/**
 * Emergency PWM output shutdown could be set to the following mode. (MF625 only)
 */
typedef enum
{
  HudaqPwmEMERGENCYOFF = 0,	///< No emergency is employed.
  HudaqPwmEMERGENCYINPUT,       ///< Emergency input in '1' shuts all outputs to '0'.
  HudaqPwmEMERGENCYNOTINPUT,    ///< Emergency input in '0' shuts all outputs to '0'.
  HudaqPwmEMERGENCYON,          ///< All outputs are shut to '0' permanently.
} HudaqPwmEmergency;


/**
 * Generate pulse-width modulation signal on a counter output.
 * @ingroup GroupPWMOutput
 * @param[in] handle    Device handle.
 * @param[in] channel   Counter output channel number.
 * @param[in] frequency Output frequency in Hz.
 * @param[in] duty      Output signal duty.
 *    The duty is the fraction of signal period during which the signal is at high level.@n
 *    Value 0 means permanent logical low on the counter output.@n
 *    Value 0.5 means periodic signal with the counter output for half of the period at logical low and for half of the period at logical high.@n
 *    Value 1 means permanent logical high on the counter output.@n
 * @return ::HUDAQSUCCESS on success, other values on failure.
 */
HUDAQLIBPUBLIC HUDAQSTATUS HUDAQAPI HudaqPWMWrite(HUDAQHANDLE handle, unsigned channel, double frequency, double duty);


/**
 * Generate 3 phases + their inversions pulse-width modulation signal on a specialized counter.
 * @ingroup GroupPWMOutput
 * @param[in] handle    Device handle.
 * @param[in] channel   Counter output channel number.
 * @param[in] frequency Output frequency in Hz.
 * @param[in] duty1     Output signal duty for first phase.
 * @param[in] duty2     Output signal duty for second phase.
 * @param[in] duty3     Output signal duty for third phase.
 *    The duty is the fraction of signal period during which the signal is at high level.@n
 *    Value 0 means permanent logical low on the counter output.@n
 *    Value 0.5 means periodic signal with the counter output for half of the period at logical low and for half of the period at logical high.@n
 *    Value 1 means permanent logical high on the counter output.@n
 * @return ::HUDAQSUCCESS on success, other values on failure.
 */
HUDAQLIBPUBLIC HUDAQSTATUS HUDAQAPI HudaqPWM3Write(HUDAQHANDLE handle, unsigned channel, double frequency, double duty1, double duty2, double duty3);


/** @cond UNDOCUMENTED */

/**
 * Generate multiphase pulse-width modulation signal on a specialized PWM counter.
 * @ingroup GroupPWMOutput
 * @param[in] handle    Device handle.
 * @param[in] channel   Counter output channel number.
 * @param[in] phasenum  Number of phases (sub channels) to be written.
 * @param[in] phases    Array of phase numbers (sub channels) that will be written.
 * @param[in] frequency Output frequency in Hz (same for all phases).
 * @param[in] duties    Array of output signal duties.
 *    The duty is the fraction of signal period during which the signal is at high level.@n
 *    Value 0 means permanent logical low on the counter output.@n
 *    Value 0.5 means periodic signal with the counter output for half of the period at logical low and for half of the period at logical high.@n
 *    Value 1 means permanent logical high on the counter output.@n
 * @return ::HUDAQSUCCESS on success, other values on failure.
 */
HUDAQLIBPUBLIC HUDAQSTATUS HudaqPWMWriteMultiphase(HUDAQHANDLE handle, unsigned channel, unsigned phasenum, const unsigned *phases, double frequency, const double *duties);

/**
 * Move stepping motor to a target position. During a first time call it resets internal
 * position counter and sets 2 internal counters for driving one stepper motor.
 * @ingroup GroupStepperOutput
 * @param[in] handle  Device handle.
 * @param[in] channel number of counter.
 * @param[in] position Target position to move stepping motor. (It is absolute position, not a difference.)
 * @return ::HUDAQSUCCESS on success, other values on failure.
 */
HUDAQLIBPUBLIC HUDAQSTATUS HUDAQAPI HudaqStepWrite(HUDAQHANDLE handle, unsigned channel, int position);

/** @endcond UNDOCUMENTED */



/**
 * Reads single channel configuration for a given subsystem.
 * Not all devices support all the parameters.
 * @ingroup GroupChannelConfig
 * @param[in] handle  Device handle.
 * @param[in] channel Channel number. The channel type is determined by the parameter identifier @c param.
 * @param[in] param Parameter identifier; see ::HudaqParameter for possible values.
 * @return value Value of the parameter; see individual parameter descriptions for description of the values.
 */
HUDAQLIBPUBLIC double HUDAQAPI HudaqGetParameter(HUDAQHANDLE handle, unsigned channel, HudaqParameter param);


/**
 * Configures single channel of a given subsystem.
 * Not all devices support all the parameters and all the parameter values.
 * @ingroup GroupChannelConfig
 * @param[in] handle  Device handle.
 * @param[in] channel Channel number. The channel type is determined by the parameter identifier @c param.
 * @param[in] param Parameter identifier; see ::HudaqParameter for possible values.
 * @param[in] value Value of the parameter; see individual parameter descriptions for possible values.
 * @return ::HUDAQSUCCESS on success, other values on failure.
 */
HUDAQLIBPUBLIC HUDAQSTATUS HUDAQAPI HudaqSetParameter(HUDAQHANDLE handle, unsigned channel, HudaqParameter param, double value);


/**
 * Query voltage ranges by their indices.
 * This function translates the voltage range index to actual voltage range limits for the device.
 * No change is done to device configuration; use ::HudaqSetParameter to set a voltage range by its index; use
 * ::HudaqGetParameter to get the index of the currently set voltage range.

 * @ingroup GroupChannelConfig
 * @param[in] handle Device handle.
 * @param[in] S      Subsystem identifier; possible values are ::HudaqAI or ::HudaqAO.
 * @param[in] item   Zero-based voltage range index.
 * @return    Pointer to a structure containing the voltage range limits for the specified index, or @c NULL for an invalid range index.
 * The structure pointed to by the returned pointer is defined like this:
 * @code
 * typedef struct
 * {
 *   double Lo;
 *   double Hi;
 * } HudaqRange;
 * @endcode
 */
HUDAQLIBPUBLIC const HudaqRange * HUDAQAPI HudaqQueryRange(HUDAQHANDLE handle, HudaqSubsystem S, unsigned item);



/* All the functions have C naming convention in C++. */
#ifdef __cplusplus
}
#endif  /* __cplusplus */


#endif  /* HUDAQLIB_H__ */

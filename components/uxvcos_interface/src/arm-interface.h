#ifndef ARM_INTERFACE_H
#define ARM_INTERFACE_H

#include "stdint.h"

// individual structure alignment (linux- gcc)
#ifdef __GNUC__
 #ifndef GCC3X_PACK8
  #define GCC3X_PACK8 __attribute__((__packed__))
 #endif
#else
 #ifndef GCC3X_PACK8
  // individual structure alignment (windows visual studio 2005)
  #define GCC3X_PACK8
 #endif
 #pragma pack(push,1)
#endif

#define UBX_FSR_SYNC1         			0x37      //!< first protocol sync byte FSR
#define UBX_FSR_SYNC2         			0x42      //!< second protocol sync byte FSR

#define ARM_INTERFACE_CLASS   			0xA1

#define ARM_REQUEST_ID  	  			0x00
#define ARM_AUTO_REQUEST_ID   			0x01

#define ARM_RAW_IMU_ID    	  			0x02
#define ARM_MOTOR_ID  	  	  			0x03
#define ARM_UART_INIT_ID	  			0x04
#define ARM_UART0_ID		  			0x05
#define ARM_UART1_ID		  			0x06
#define ARM_US_SENSOR_ID	  			0x07
#define ARM_RC_ID		      			0x08
#define ARM_RAW_MAG3D_ID	  			0x09
#define ARM_RAW_BARO_ID		  			0x10
#define ARM_RAW_BARO_TEMP_ID  			0x11

#define ARM_RAW_AD0_ID  	  			0x12
#define ARM_RAW_AD1_ID  	  			0x13
#define ARM_INTERNAL_AD_ID	  			0x14
#define ARM_BOARD_VERSION	  			0x15

#define ARM_RAW_AIRSPEED_ID	  			0x16

#define ARM_IMU_ID            			0x20
#define ARM_ATTITUDE_ID       			0x21
#define ARM_BIAS_ID           			0x22
#define ARM_VELOCITY_ID       			0x23

#define ARM_HEIGHT_ID         			0x28
#define ARM_ALTITUDE_ID       			0x29

#define ARM_IMU9DOF_ADXL345_ID      	0x30
#define ARM_IMU9DOF_ITG3200_ID      	0x31
#define ARM_IMU9DOF_HMC5843_ID      	0x32

//--> Configuration message IDs
#define ARM_CONFIG_UART0_ID	  			0x80
#define ARM_CONFIG_UART1_ID	  			0x81

#define ARM_CONFIG_IMU_ID     			0x90
#define ARM_CONFIG_MAG_ID     			0x91
#define ARM_CONFIG_BARO_ID    			0x92
#define ARM_CONFIG_QNH_ID     			0x93

#define ARM_CONFIG_CONTROLLER_ID   		0xa0
#define ARM_CONFIG_AXIS_ID         		0xa1
#define ARM_CONFIG_DIRECT_FEEDBACK_ID   		0xa2
#define ARM_SET_ID                 		0xaa

#define ARM_SERVO_ID		  			0xb0

#define ARMINTERFACEBOARD   			201003
#define PRUEFSTANDBOARD                 201001

typedef uint32_t request_t;

struct GCC3X_PACK8 ArmRequest_t
{
        request_t 	word[2];
};

struct GCC3X_PACK8 ArmAutoRequest_t
{
        uint32_t time;
        uint16_t armXID;
        uint32_t offset;
        uint32_t multipleOfBasetime;
};

struct GCC3X_PACK8 ArmBoardVersion_t
{
        uint32_t time;
        uint32_t versionID;
        uint8_t enc28j60_revision;
};


//--> Sensor messages

#define ARMMOTOR_COUNT 4
struct GCC3X_PACK8 ArmMotor_t
{
        uint32_t	time;                        			//!< timestamp [ms]
        uint8_t   enabled;
        uint8_t	CommandMotorPWM[ARMMOTOR_COUNT];		//!< Commanded motor PWM	[0-255]
        uint8_t	Current[ARMMOTOR_COUNT];				//!< Motor current		[-]
	float 			Frequency[ARMMOTOR_COUNT];		    	//!< Motor frequency	[-]
};

struct GCC3X_PACK8 ArmMotorCommand_t
{
        uint8_t	CommandMotorPWM[ARMMOTOR_COUNT];		//!< Commanded motor PWM	[0-255]
};

#define ARMRC_CHANNELS 10
struct GCC3X_PACK8 ArmRC_t
{
        uint32_t	time;       			//!< timestamp [ms]
        int8_t			status;
        uint16_t value[ARMRC_CHANNELS];   			//!< value
};

#define ARMSERVO_CHANNELS 6
struct GCC3X_PACK8 ArmServo_t
{
        uint16_t value[ARMSERVO_CHANNELS];   			//!< value
};

#define ARMUS_SENSORS 6
struct GCC3X_PACK8 ArmUS_t
{
        uint32_t	time;       			//!< timestamp [ms]
        uint16_t	value[ARMUS_SENSORS];  //!< value
};

struct GCC3X_PACK8 ArmAD16_t
{
        uint32_t	time;
        uint16_t	value[8];
};

struct GCC3X_PACK8 ArmInternalAD16_t
{
        uint32_t	time;
        uint16_t	value[3];
};

struct GCC3X_PACK8 ArmRawMag3D_t
{
        uint32_t	time;
	float	value[3];
};

struct GCC3X_PACK8 ArmRawBaro_t
{
        uint32_t	time;
        uint16_t	value;
};

struct GCC3X_PACK8 ArmRawBaroTemp_t
{
        uint32_t	time;
        uint16_t	value;
};

struct GCC3X_PACK8 ArmRawIMU_t
{
        uint32_t time;
        uint16_t	accel[3];
        uint16_t	omega[3];
};

struct GCC3X_PACK8 ArmIMU_t
{
        uint32_t time;
	float	accelX, accelY, accelZ;      
  float omegaX, omegaY, omegaZ;
};

struct GCC3X_PACK8 ArmRawADXL345_t
{
        uint32_t time;
        uint16_t	accel[3];
};

struct GCC3X_PACK8 ArmRawITG3200_t
{
        uint32_t time;
        uint16_t	temperature;
        uint16_t	omega[3];
};

struct GCC3X_PACK8 ArmRawHMC5843_t
{
        uint32_t time;
        uint16_t	bField[3];
};

// UART Config messages

struct GCC3X_PACK8 ArmUARTConfig_t
{
        uint32_t	baudrate;
        uint8_t	mode;
        uint8_t	fmode;
};

// IMU Config messages

struct GCC3X_PACK8 ArmAnalogChannelConfig_t
{
        int32_t offsetA;
	float factorB;
	float offsetC;
	float min;
	float max;
};

struct GCC3X_PACK8 ArmIMUConfig_t
{
        uint8_t mappingAccel[3];
	struct ArmAnalogChannelConfig_t accel[3];
        uint8_t mappingGyro[3];
	struct ArmAnalogChannelConfig_t gyro[3];
};

// Magnetometer Config messages

struct GCC3X_PACK8 ArmMagChannelConfig_t
{
        uint8_t channel;
	float offsetA;
	float factorB;
	float offsetC;
	float min;
	float max;
};

struct GCC3X_PACK8 ArmMag3DConfig_t
{
        uint8_t mappingMag3D[3];
	struct ArmMagChannelConfig_t channel[3];
};

// Attitude and Heading Reference System (AHRS) messages

struct GCC3X_PACK8 ArmAttitude_t
{
	float roll;
	float pitch;
	float yaw;
};

struct GCC3X_PACK8 ArmBias_t
{
	float accelX;
	float accelY;
	float accelZ;
	float gyroX;
	float gyroY;
	float gyroZ;
};

struct GCC3X_PACK8 ArmVelocity_t
{
	float x;
	float y;
	float z;
};

// Altitude and Height messages

enum ArmHeightMode_t { ULTRASOUND, BARO };
struct GCC3X_PACK8 ArmHeight_t
{
	float height;
        uint8_t mode;
};

struct GCC3X_PACK8 ArmAltitude_t
{
	float altitude;
};

struct GCC3X_PACK8 ArmBaroConfig_t
{
	struct ArmAnalogChannelConfig_t baro;
};

struct GCC3X_PACK8 ArmQNHConfig_t
{
	float qnh;
};

// Quadrotor control messages

struct GCC3X_PACK8 QuadroControllerPID_t
{
	float T_in;
    float KP;
    float KI;
    float KD;
    float Kw;
    float Kdw;
    float min_P, max_P;
    float min_I, max_I;
    float min_D, max_D;
    float min_w, max_w;
    float min_dw, max_dw;
    float min_u, max_u;
};

struct GCC3X_PACK8 QuadroControllerSet_t
{
	float roll;         // \in rad
    float pitch;        // \in rad
    float yaw;          // \in rad
    float height;       // \in m
	float throttle;		// \in [0,1]
};

struct GCC3X_PACK8 QuadroControllerConfig_t
{
        uint8_t enabled;
};

enum QuadroControllerAxis_t { ROLL, PITCH, YAW, HEIGHT };
struct GCC3X_PACK8 QuadroControllerAxisConfig_t
{
        uint8_t axis;
        uint8_t enabled;
	struct QuadroControllerPID_t pid;
};

struct GCC3X_PACK8 QuadroDirectFeedback_t
{
  uint8_t enabled;
  struct GCC3X_PACK8
  {
    float	accelX, accelY, accelZ;
    float omegaX, omegaY, omegaZ;
  } bias, gain[ARMMOTOR_COUNT];
};

// #define REQUEST(bit,request)       ((bit) < 32 ? ((request[0]) |=  BIT(bit)) : ((request[1]) |=  BIT((bit)-32)))
// #define CLEAR_REQUEST(bit,request) ((bit) < 32 ? ((request[0]) &= ~BIT(bit)) : ((request[1]) &= ~BIT((bit)-32)))
// #define IS_REQUESTED(bit,request)  ((bit) < 32 ? ((request[0]) &   BIT(bit)) : ((request[1]) &   BIT((bit)-32)))

#define REQUEST_BITS (sizeof(request_t)*8)

static inline void SET_REQUEST(unsigned bit, request_t request[]) {
  unsigned i = 0;
  while(bit >= REQUEST_BITS) { i++; bit-=REQUEST_BITS; }
  request[i] |= (1ul << bit);
}

static inline void CLEAR_REQUEST(unsigned bit, request_t request[]) {
  unsigned i = 0;
  while(bit >= REQUEST_BITS) { i++; bit-=REQUEST_BITS; }
  request[i] &= ~(1ul << bit);
}

static inline bool IS_REQUESTED(unsigned bit, request_t request[]) {
  unsigned i = 0;
  while(bit >= REQUEST_BITS) { i++; bit-=REQUEST_BITS; }
  return request[i] & (1ul << bit);
}

#ifndef __GNUC__
  #pragma pack(pop)
#endif

#endif // ARM_INTERFACE_H

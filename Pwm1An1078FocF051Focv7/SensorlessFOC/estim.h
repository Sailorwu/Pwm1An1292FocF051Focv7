
#ifndef smcpos_H
#define smcpos_H

#include "system_define.h"

typedef struct 
{
	short   		qDeltaT;      // Integration constant
	short   		qRho;    	    // angle of estimation
	long  			qRhoStateVar; // internal variable for angle
	short   		qOmegaMr;     // primary speed estimation
	short   		qLastIalpha;  // last value for Ialpha
	short   		qLastIbeta;   // last value for Ibeta
	short   		qDIalpha;     // difference Ialpha
	short   		qDIbeta;      // difference Ibeta
	short				qEsa;			// BEMF alpha
	short				qEsb;			// BEMF beta
	short				qEsd;			// BEMF d
	short				qEsq;			// BEMF q
	short				qDiCounter;	// counter in Last DI tables
	short				qVIndalpha;   // dI*Ls/dt alpha
	short				qVIndbeta;    // dI*Ls/dt beta
	short				qEsdf;        // BEMF d filtered
	long			  qEsdStateVar; // state var for BEMF d Filtered
	short				qEsqf;        // BEMF q filtered
	long			  qEsqStateVar; // state var for BEMF q Filtered
	short				qKfilterEsdq; // filter constant for d-q BEMF
	short   		qVelEstim; 			// Estimated speed 
	short   		qVelEstimFilterK; 	// Filter Konstant for Estimated speed 
	long   			qVelEstimStateVar; 	// State Variable for Estimated speed 
  short   		qLastValpha;  // Value from last control step Ialpha 
  short   		qLastVbeta;   // Value from last control step Ibeta
	short				qDIlimitLS;			// dIalphabeta/dt
	short				qDIlimitHS;			// dIalphabeta/dt
	short				qLastIalphaHS[8];		//  last  value for Ialpha
	short				qLastIbetaHS[8];			// last  value for Ibeta
	short       RhoOffset;            // estima angle init offset

} tEstimParm;

typedef struct 
{
	int				   qRs;			// Rs value - stator resistance
	short				   qLsDt;		// Ls/dt value - stator inductand / dt - variable with speed
	short				   qLsDtBase;	// Ls/dt value - stator inductand / dt for base speed (nominal)
	short				   qInvKFi;	    // InvKfi constant value ( InvKfi = Omega/BEMF )
	short				   qInvKFiBase; // InvKfi constant - base speed (nominal) value
} tMotorEstimParm;

//Fdweak.h
typedef struct 
{
	short		qIdRef;          // d-current reference
	short		qFwOnSpeed;      // flux weakening on speed -
	short		qIndex;          // lookup tables index
  short		qFwCurve[18];	 // Curve for magnetizing current variation with speed
  short		qInvKFiCurve[18];// Curve for InvKfi constant InvKfi = Omega/BEMF variation with speed
  short   qLsCurve[18];    // Curve for Ls variation with speed
} tFdWeakParm;   

extern tEstimParm 	EstimParm;
extern tMotorEstimParm 	MotorEstimParm;
extern tFdWeakParm FdWeakParm;
//------------------  C API for FdWeak routine ---------------------
void 	InitFWParams(void);
short FieldWeakening( short qMotorSpeed );
void	AdaptEstimParm(short  qMotorSpeed);

//------------------  C API for Control routine ---------------------
void	Estim(void);
void	InitEstimParm(void);
#endif
		

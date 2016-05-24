#ifndef FdWeak_H
#define FdWeak_H
#include "system_define.h"

typedef struct 
{
	SFRAC16	qK1;            // < Nominal speed value
	SFRAC16	qIdRef;
	SFRAC16	qFwOnSpeed;
	SFRAC16	qFwActiv;
	SFRAC16	qIndex;
	SFRAC16	qFWPercentage;
	SFRAC16	qInterpolPortion;
  int16_t		qFwCurve[16];	// Curve for magnetizing current
} tFdWeakParm;
    
extern tFdWeakParm FdWeakParm;

SFRAC16 FieldWeakening(SFRAC16 qMotorSpeed);
void FWInit (void);

#endif




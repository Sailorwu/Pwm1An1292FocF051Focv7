
#include "system_define.h"

tFdWeakParm	FdWeakParm;

#define FWONSPEED NOMINAL_SPEED_RPM*NOPOLESPAIRS

void InitFWParams(void)
{
    
  // Field Weakening constant for constant torque range
	FdWeakParm.qIdRef = IDREF_BASESPEED;       	// Flux reference value
	FdWeakParm.qFwOnSpeed=FWONSPEED;		// Startspeed for Fieldweakening
	
	/* initialize magnetizing curve values */
	FdWeakParm.qFwCurve[0]	= IDREF_SPEED0;
	FdWeakParm.qFwCurve[1]	= IDREF_SPEED1;
	FdWeakParm.qFwCurve[2]	= IDREF_SPEED2;
	FdWeakParm.qFwCurve[3]	= IDREF_SPEED3;
	FdWeakParm.qFwCurve[4]	= IDREF_SPEED4;
	FdWeakParm.qFwCurve[5]	= IDREF_SPEED5;
	FdWeakParm.qFwCurve[6]	= IDREF_SPEED6;
	FdWeakParm.qFwCurve[7]	= IDREF_SPEED7;
	FdWeakParm.qFwCurve[8]	= IDREF_SPEED8;
	FdWeakParm.qFwCurve[9]	= IDREF_SPEED9;
	FdWeakParm.qFwCurve[10]	= IDREF_SPEED10;
	FdWeakParm.qFwCurve[11]	= IDREF_SPEED11;
	FdWeakParm.qFwCurve[12]	= IDREF_SPEED12;
	FdWeakParm.qFwCurve[13]	= IDREF_SPEED13;
	FdWeakParm.qFwCurve[14]	= IDREF_SPEED14;
	FdWeakParm.qFwCurve[15]	= IDREF_SPEED15;	
	FdWeakParm.qFwCurve[16]	= IDREF_SPEED16;
	FdWeakParm.qFwCurve[17]	= IDREF_SPEED17;
	
	
	/* init inverse Kfi curve values */
	FdWeakParm.qInvKFiCurve[0] = INVKFI_SPEED0;
	FdWeakParm.qInvKFiCurve[1] = INVKFI_SPEED1;
	FdWeakParm.qInvKFiCurve[2] = INVKFI_SPEED2;
	FdWeakParm.qInvKFiCurve[3] = INVKFI_SPEED3;
	FdWeakParm.qInvKFiCurve[4] = INVKFI_SPEED4;
	FdWeakParm.qInvKFiCurve[5] = INVKFI_SPEED5;
	FdWeakParm.qInvKFiCurve[6] = INVKFI_SPEED6;
	FdWeakParm.qInvKFiCurve[7] = INVKFI_SPEED7;
	FdWeakParm.qInvKFiCurve[8] = INVKFI_SPEED8;
	FdWeakParm.qInvKFiCurve[9] = INVKFI_SPEED9;
	FdWeakParm.qInvKFiCurve[10] = INVKFI_SPEED10;
	FdWeakParm.qInvKFiCurve[11] = INVKFI_SPEED11;
	FdWeakParm.qInvKFiCurve[12] = INVKFI_SPEED12;
	FdWeakParm.qInvKFiCurve[13] = INVKFI_SPEED13;
	FdWeakParm.qInvKFiCurve[14] = INVKFI_SPEED14;
	FdWeakParm.qInvKFiCurve[15] = INVKFI_SPEED15;
	FdWeakParm.qInvKFiCurve[16] = INVKFI_SPEED16;
	FdWeakParm.qInvKFiCurve[17] = INVKFI_SPEED17;
	
	/* init Ls variation curve */
	FdWeakParm.qLsCurve[0]= LS_OVER2LS0_SPEED0;
	FdWeakParm.qLsCurve[1]= LS_OVER2LS0_SPEED1;
	FdWeakParm.qLsCurve[2]= LS_OVER2LS0_SPEED2;
	FdWeakParm.qLsCurve[3]= LS_OVER2LS0_SPEED3;
	FdWeakParm.qLsCurve[4]= LS_OVER2LS0_SPEED4;
	FdWeakParm.qLsCurve[5]= LS_OVER2LS0_SPEED5;
	FdWeakParm.qLsCurve[6]= LS_OVER2LS0_SPEED6;
	FdWeakParm.qLsCurve[7]= LS_OVER2LS0_SPEED7;
	FdWeakParm.qLsCurve[8]= LS_OVER2LS0_SPEED8;
	FdWeakParm.qLsCurve[9]= LS_OVER2LS0_SPEED9;
	FdWeakParm.qLsCurve[10]= LS_OVER2LS0_SPEED10;
	FdWeakParm.qLsCurve[11]= LS_OVER2LS0_SPEED11;
	FdWeakParm.qLsCurve[12]= LS_OVER2LS0_SPEED12;
	FdWeakParm.qLsCurve[13]= LS_OVER2LS0_SPEED13;
	FdWeakParm.qLsCurve[14]= LS_OVER2LS0_SPEED14;
	FdWeakParm.qLsCurve[15]= LS_OVER2LS0_SPEED15;
	FdWeakParm.qLsCurve[16]= LS_OVER2LS0_SPEED16;
	FdWeakParm.qLsCurve[17]= LS_OVER2LS0_SPEED17;
	
}	

short	FieldWeakening(short	qMotorSpeed)
{
    /* if the speed is less than one for activating the FW */
	if (qMotorSpeed<=FdWeakParm.qFwOnSpeed)
	{
		/* set Idref as first value in magnetizing curve */
		FdWeakParm.qIdRef=FdWeakParm.qFwCurve[0];
	} 
	else
	{
		// Index in FW-Table
		FdWeakParm.qIndex=(qMotorSpeed-FdWeakParm.qFwOnSpeed)>>10;		
	
		// Interpolation betwen two results from the Table
		FdWeakParm.qIdRef= FdWeakParm.qFwCurve[FdWeakParm.qIndex]-
						   (((long)(FdWeakParm.qFwCurve[FdWeakParm.qIndex]-FdWeakParm.qFwCurve[FdWeakParm.qIndex+1])
							*
						   (long)(qMotorSpeed-((FdWeakParm.qIndex<<10)+FdWeakParm.qFwOnSpeed)))>>10);
	}
	return FdWeakParm.qIdRef;
}


void	AdaptEstimParm(short  qMotorSpeed)
{

	short	iIndex;
	short qInvKFi;
	short qLsDt;
	short temp;
	
	qLsDt=MotorEstimParm.qLsDtBase;
	if (qMotorSpeed<=FdWeakParm.qFwOnSpeed)
	{
		/* adapt fileter parameter */
		EstimParm.qKfilterEsdq = KFILTER_ESDQ;
		
		qInvKFi=MotorEstimParm.qInvKFiBase;	
	    
  } 
	else
	{
		/* adapt filer parameter */
		EstimParm.qKfilterEsdq = KFILTER_ESDQ_FW;
		
		// Index in FW-Table
		iIndex=(qMotorSpeed-FdWeakParm.qFwOnSpeed)>>10;		

		// Interpolation betwen two results from the Table
		temp = FdWeakParm.qInvKFiCurve[iIndex]-FdWeakParm.qInvKFiCurve[iIndex+1];
		qInvKFi = 	(short)(((long)(temp)
								*(long)(qMotorSpeed-((iIndex<<10)+FdWeakParm.qFwOnSpeed)))>>10);
	    qInvKFi = FdWeakParm.qInvKFiCurve[iIndex] - qInvKFi;


		// Interpolation betwen two results from the Table
		temp = 	FdWeakParm.qLsCurve[iIndex]-
						   		(((long)(FdWeakParm.qLsCurve[iIndex]-FdWeakParm.qLsCurve[iIndex+1])
								*
						   		(long)(qMotorSpeed-((iIndex<<10)+FdWeakParm.qFwOnSpeed)))>>10);
        // Lsdt = Lsdt0*Ls/Ls0
	    qLsDt = (short) ( ((long)((long)qLsDt*(long)temp))>>14);


	}
	MotorEstimParm.qInvKFi = qInvKFi;
  MotorEstimParm.qLsDt = qLsDt;	
}

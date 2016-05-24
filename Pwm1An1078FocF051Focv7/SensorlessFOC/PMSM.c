#define INITIALIZE
#include "system_define.h"

uGFt uGF;
			
tPIParm     PIParmQ;        /* parms for PI controlers */
tPIParm     PIParmD;        /* parms for PI controlers */
tPIParm     PIParmQref;     /* parms for PI controlers */

tReadADCParm ReadADCParm;   /* adc values structure */

/******************************************************************************/
/******************************************************************************/
/*                                                                            */
/*                    local variables                                         */
/*                                                                            */
/******************************************************************************/
/******************************************************************************/

uint32_t Startup_Ramp = 0; /* ramp angle variable for initial ramp */
uint16_t Startup_Lock = 0; /* lock variable for initial ramp */       
uint16_t iDispLoopCnt; /* display loop count */

void SensorlessFOCinit(void)
{
	/* init PI control parameters */
	InitControlParameters(); 
	
	/* init estim parameters */
	InitEstimParm();
	
	/* init flux weakening params */
	InitFWParams();
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: DoControl                                                   */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description:                                                               */
/* Executes one PI itteration for each of the three loops Id,Iq,Speed         */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

void DoControl( void )
{
    /* temporary vars for sqrt calculation of q reference */
    long temp_qref_pow1,temp_qref_pow2;

    static short tuning_add_rampup = 0; // tuning speed ramp value 
    static short tuning_delay_rampup;   // tuning speed ramp increase delay

	if( uGF.bit.OpenLoop )
	{
			// OPENLOOP:  force rotating angle,Vd,Vq
			if( uGF.bit.ChangeMode )
			{
					// just changed to openloop
					uGF.bit.ChangeMode = 0;
					// synchronize angles

					// VqRef & VdRef not used
					CtrlParm.qVqRef = 0;
					CtrlParm.qVdRef = 0;

					/* reinit vars for initial speed ramp */
					Startup_Lock = 0;
					Startup_Ramp = 0;
					tuning_add_rampup = 0;
			}
			
			/* speed reference */
			CtrlParm.qVelRef = Q_CURRENT_REF_OPENLOOP;
			/* q current reference is equal to the vel reference */
			/* while d current reference is equal to 0 */
			/* for maximum startup torque, set the q current to maximum acceptable */
			/* value represents the maximum peak value */
			CtrlParm.qVqRef    = CtrlParm.qVelRef;
			
			// PI control for Q
			PIParmQ.qInMeas = ParkParm.qIq;
			PIParmQ.qInRef  = CtrlParm.qVqRef;
			CalcPI(&PIParmQ);
			ParkParm.qVq    = PIParmQ.qOut;       

			// PI control for D
			PIParmD.qInMeas = ParkParm.qId;
			PIParmD.qInRef  = CtrlParm.qVdRef;
			CalcPI(&PIParmD);
			ParkParm.qVd    = PIParmD.qOut;


	} 
	else
	// Closed Loop Vector Control
	{

		/* unsigned values */
		ReadADC0( &ReadADCParm );
		/* ADC values are shifted with 2 */
		ReadADCParm.qAnRef=ReadADCParm.qADValue;		// Speed pot ref max value +-8190
	
		// Ramp generator to limit the change of the speed reference
		// the rate of change is defined by CtrlParm.qRefRamp
    	
    CtrlParm.qDiff=CtrlParm.qVelRef - ReadADCParm.qAnRef;		

		/* if delay is not completed */
		if(tuning_delay_rampup > TUNING_DELAY_RAMPUP)	tuning_delay_rampup = 0;
		/* while speed less than maximum and delay is complete */
		if(tuning_delay_rampup == 0)
		{
			if(CtrlParm.qVelRef>ReadADCParm.qAnRef) 
			{
				if(tuning_add_rampup) tuning_add_rampup--;
			}
			else if(CtrlParm.qVelRef<ReadADCParm.qAnRef) tuning_add_rampup++;
			else{}
		}
		tuning_delay_rampup++;
		/* the reference is continued from the open loop speed up ramp */
		CtrlParm.qVelRef =   ENDSPEED_ELECTR +  tuning_add_rampup;
	    
		if( uGF.bit.ChangeMode )
		{
			// just changed from openloop
			uGF.bit.ChangeMode = 0;
			PIParmQref.qdSum = (long)CtrlParm.qVqRef << 14;
		}               

/* if TORQUE MODE skip the speed controller */                
#ifndef	TORQUE_MODE
		// Execute the velocity control loop
		PIParmQref.qInMeas = EstimParm.qVelEstim;
		PIParmQref.qInRef  = CtrlParm.qVelRef;
		CalcPI(&PIParmQref);
		CtrlParm.qVqRef = PIParmQref.qOut;

#else
		CtrlParm.qVqRef = CtrlParm.qVelRef;
#endif       
		/* Flux weakenign control - the actual speed is replaced */
		/* with the reference speed for stability */
		/* reference for d curent component */
		CtrlParm.qVdRef=0;//FieldWeakening(_Q15abs(CtrlParm.qVelRef));
		/* adapt the estimator parameters in concordance with the speed */
		AdaptEstimParm(_Q15abs(CtrlParm.qVelRef));
		
		// PI control for D
		PIParmD.qInMeas = ParkParm.qId;
		PIParmD.qInRef  = CtrlParm.qVdRef;
		CalcPI(&PIParmD);
		ParkParm.qVd    = PIParmD.qOut;

		/* dynamic d-q adjustment */
		/* with d component priority */
		/* vq=sqrt (vs^2 - vd^2) */
		/* limit vq maximum to the one resulting from the calculation above */
		temp_qref_pow1 = (long)PIParmD.qOut * (long)PIParmD.qOut;
		temp_qref_pow2 = (long)Q15(0.95)<<15 ;
		PIParmQ.qOutMax = _Q15sqrt (temp_qref_pow2-temp_qref_pow1);

		// PI control for Q
		PIParmQ.qInMeas = ParkParm.qIq;
		PIParmQ.qInRef  = CtrlParm.qVqRef;
		CalcPI(&PIParmQ);
		ParkParm.qVq    = PIParmQ.qOut;       
  }
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: ADC1Interrupt                                               */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Does speed calculation and executes the vector update loop    */
/* The ADC sample and conversion is triggered by the PWM period.              */
/* The speed calculation assumes a fixed time interval between calculations.  */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void SensorlessFOCRUN(void)
{  
    // Increment count variable that controls execution
    // of display and button functions.
    iDispLoopCnt++;
             
    if( uGF.bit.RunMotor )
    {
			// Calculate qIa,qIb
			MeasCompCurr(&ParkParm);

			// Calculate qId,qIq from qSin,qCos,qIa,qIb
			ClarkePark(&ParkParm);


			// Speed and field angle estimation
			//****************************            
			Estim();

			// Calculate control values
			DoControl();

			// Calculate qAngle from QEI Module
			CalculateParkAngle();
        
					/* if open loop */
			if(uGF.bit.OpenLoop == 1)
			{
				/* the angle is given by parkparm */
				SincosParm.qAngle = ParkParm.qAngle;
			} 
			else
			{
				/* if closed loop, angle generated by estim */
				SincosParm.qAngle = EstimParm.qRho;
			}
						
			// Calculate qSin,qCos from qAngle
			SinCos(&SincosParm);
	
			ParkParm.qSin=SincosParm.qSin;
			ParkParm.qCos=SincosParm.qCos;
			
			// Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq
			InvPark(&ParkParm);     

			// Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta 
			// Calculate and set PWM duty cycles from Vr1,Vr2,Vr3
			CalcSVGen(&ParkParm); 					
		} 
		else Close_PWM(); 
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: CalculateParkAngle                                          */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Generate the start sinwaves feeding the motor's terminals     */
/* Open loop control, forcing the motor to allign and to start speeding up    */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void CalculateParkAngle(void)
{
    /* if open loop */
	if(uGF.bit.OpenLoop)	
	{
		/* begin wiht the lock sequence, for field alligniament */
		if (Startup_Lock < LOCK_TIME)  Startup_Lock+=1;
	    /* then ramp up till the end speed */
		else if (Startup_Ramp < END_SPEED)
			Startup_Ramp+=OPENLOOP_RAMPSPEED_INCREASERATE;
		else /* switch to closed loop */
		{
#ifndef OPEN_LOOP_FUNCTIONING
			uGF.bit.ChangeMode = 1;
			uGF.bit.OpenLoop = 0;
#endif
		}
		/* the angle set depends on startup ramp */
		ParkParm.qAngle += (short)(Startup_Ramp >> 10);
	}
	else /* switched to closed loop */
	{
		/* in closed loop slowly decrease the offset add to */
		/* the estimated angle */
		if(EstimParm.RhoOffset>0)EstimParm.RhoOffset--; 
	}
	return;
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: InitControlParameters                                       */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description:                                                               */
/* Init control parameters: PI coefficients, scalling consts etc.             */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void InitControlParameters(void)
{
    // ============= ADC - Measure Current & Pot ======================

    // Scaling constants: Determined by calibration or hardware design.
    ReadADCParm.qK      = KPOT;    
    MeasCurrParm.qKa    = KCURRA;    
    MeasCurrParm.qKb    = KCURRB;   
    
    CtrlParm.qRefRamp = SPEEDREFRAMP;
    
    // ============= SVGen ===============
    // Set PWM period to Loop Time 
    SVGenParm.iPWMPeriod = LOOPTIME_TCY;
    
    // ============= PI D Term ===============      
    PIParmD.qKp = D_CURRCNTR_PTERM;       
    PIParmD.qKi = D_CURRCNTR_ITERM;              
    PIParmD.qKc = D_CURRCNTR_CTERM;       
    PIParmD.qOutMax = D_CURRCNTR_OUTMAX;
    PIParmD.qOutMin = -PIParmD.qOutMax;

    InitPI(&PIParmD);

    // ============= PI Q Term ===============
    PIParmQ.qKp = Q_CURRCNTR_PTERM;    
    PIParmQ.qKi = Q_CURRCNTR_ITERM;
    PIParmQ.qKc = Q_CURRCNTR_CTERM;
    PIParmQ.qOutMax = Q_CURRCNTR_OUTMAX;
    PIParmQ.qOutMin = -PIParmQ.qOutMax;

    InitPI(&PIParmQ);

    // ============= PI Qref Term ===============
    PIParmQref.qKp = SPEEDCNTR_PTERM;       
    PIParmQref.qKi = SPEEDCNTR_ITERM;       
    PIParmQref.qKc = SPEEDCNTR_CTERM;       
    PIParmQref.qOutMax = SPEEDCNTR_OUTMAX;   
    PIParmQref.qOutMin = -PIParmQref.qOutMax;

    InitPI(&PIParmQref);
	return;
}


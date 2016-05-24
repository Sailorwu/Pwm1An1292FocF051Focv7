#include "estim.h"

#define DECIMATE_NOMINAL_SPEED NOMINAL_SPEED_RPM*NOPOLESPAIRS/10
/******************************************************************************/
/******************************************************************************/
/*                                                                            */
/*                    typedef definitions                                     */
/*                                                                            */
/******************************************************************************/
/******************************************************************************/

tSincosParm			SincosParm;
tEstimParm 			EstimParm;
tMotorEstimParm MotorEstimParm;

/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: Estim                                                       */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Estimation of the speed of the motor and field angle based on */
/* inverter voltages and motor currents.                                      */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void Estim(void)
{
    //*******************************
    // dIalpha = Ialpha-oldIalpha,  dIbeta  = Ibeta-oldIbeta
    // for lower speed the granularity of differnce is higher - the 
    // difference is made between 2 sampled values @ 8 ADC ISR cycles
    if (_Q15abs(EstimParm.qVelEstim)<NOMINAL_SPEED_RPM*NOPOLESPAIRS)
    {
    
    	EstimParm.qDIalpha	=	(ParkParm.qIalpha-EstimParm.qLastIalphaHS[(EstimParm.qDiCounter-7)&0x0007]);
    	/* the current difference can exceed the maximum value per 8 ADC ISR cycle */
    	/* the following limitation assures a limitation per low speed - up to the nominal speed */
    	if (EstimParm.qDIalpha>EstimParm.qDIlimitLS) EstimParm.qDIalpha=EstimParm.qDIlimitLS;
    	if (EstimParm.qDIalpha<-EstimParm.qDIlimitLS) EstimParm.qDIalpha=-EstimParm.qDIlimitLS;
    	EstimParm.qVIndalpha = ( (long) MotorEstimParm.qLsDt * (long)(EstimParm.qDIalpha) )>>10;
     
    	EstimParm.qDIbeta	=	(ParkParm.qIbeta-EstimParm.qLastIbetaHS[(EstimParm.qDiCounter-7)&0x0007]);
    	/* the current difference can exceed the maximum value per 8 ADC ISR cycle */
    	/* the following limitation assures a limitation per low speed - up to the nominal speed */
    	if (EstimParm.qDIbeta>EstimParm.qDIlimitLS) EstimParm.qDIbeta=EstimParm.qDIlimitLS;
    	if (EstimParm.qDIbeta<-EstimParm.qDIlimitLS) EstimParm.qDIbeta=-EstimParm.qDIlimitLS;
    	EstimParm.qVIndbeta= ( (long)MotorEstimParm.qLsDt * (long)(EstimParm.qDIbeta))>>10;
    
    }
    else
    {
    	EstimParm.qDIalpha	=	(ParkParm.qIalpha-EstimParm.qLastIalphaHS[(EstimParm.qDiCounter)]);
    	/* the current difference can exceed the maximum value per 1 ADC ISR cycle */
    	/* the following limitation assures a limitation per high speed - up to the maximum speed */
    	if (EstimParm.qDIalpha>EstimParm.qDIlimitHS) EstimParm.qDIalpha=EstimParm.qDIlimitHS;
    	if (EstimParm.qDIalpha<-EstimParm.qDIlimitHS) EstimParm.qDIalpha=-EstimParm.qDIlimitHS;
    	EstimParm.qVIndalpha= (  (long)MotorEstimParm.qLsDt * (EstimParm.qDIalpha) )>>7;
    
    	EstimParm.qDIbeta	=	(ParkParm.qIbeta-EstimParm.qLastIbetaHS[(EstimParm.qDiCounter)]);
    	/* the current difference can exceed the maximum value per 1 ADC ISR cycle */
    	/* the following limitation assures a limitation per high speed - up to the maximum speed */
    	if (EstimParm.qDIbeta>EstimParm.qDIlimitHS) EstimParm.qDIbeta=EstimParm.qDIlimitHS;
    	if (EstimParm.qDIbeta<-EstimParm.qDIlimitHS) EstimParm.qDIbeta=-EstimParm.qDIlimitHS;
    	EstimParm.qVIndbeta= ((long)MotorEstimParm.qLsDt * (EstimParm.qDIbeta))>>7;
    
    }
    
    //*******************************
    // update  LastIalpha and LastIbeta
    EstimParm.qDiCounter=(EstimParm.qDiCounter+1) & 0x0007;
    EstimParm.qLastIalphaHS[EstimParm.qDiCounter]	=	ParkParm.qIalpha;
    EstimParm.qLastIbetaHS[EstimParm.qDiCounter] 	=	ParkParm.qIbeta;
    
    //*******************************
    // Stator voltage eqations
    // Ualpha = Rs * Ialpha + Ls dIalpha/dt + BEMF
    // BEMF = Ualpha - Rs Ialpha - Ls dIalpha/dt   
    
		EstimParm.qEsa		= 	EstimParm.qLastValpha -
							(((long) MotorEstimParm.qRs  * (long) ParkParm.qIalpha)	>>14)
							-EstimParm.qVIndalpha;
    /* the multiplication between the Rs and Ialpha was shifted by 14 instead of 15 */
    /* because the Rs value normalized exceeded Q15 range, so it was divided by 2 */
    /* immediatelky after the normalization - in userparms.h */

    // Ubeta = Rs * Ibeta + Ls dIbeta/dt + BEMF
    // BEMF = Ubeta - Rs Ibeta - Ls dIbeta/dt   
		EstimParm.qEsb		= 	EstimParm.qLastVbeta -
							(((long) MotorEstimParm.qRs  * (long) ParkParm.qIbeta )	>>14)
							- EstimParm.qVIndbeta;
							
    /* the multiplication between the Rs and Ibeta was shifted by 14 instead of 15 */
    /* because the Rs value normalized exceeded Q15 range, so it was divided by 2 */
    /* immediatelky after the normalization - in userparms.h */
    
    //*******************************
    // update  LastValpha and LastVbeta
		EstimParm.qLastValpha = ParkParm.qValpha;
		EstimParm.qLastVbeta = ParkParm.qVbeta;


			// Calculate Sin(Rho) and Cos(Rho)
		SincosParm.qAngle 	=	EstimParm.qRho + EstimParm.RhoOffset; 
		
		SinCos(&SincosParm);

			//*******************************
			//    Esd =  Esa*cos(Angle) + Esb*sin(Rho)
		EstimParm.qEsd		=	(((long) EstimParm.qEsa * (long)SincosParm.qCos)>>15)
								+
								(((long)EstimParm.qEsb * (long)SincosParm.qSin)>>15);
			//*******************************
			//   Esq = -Esa*sin(Angle) + Esb*cos(Rho)
		EstimParm.qEsq		=	(((long) EstimParm.qEsb * (long)SincosParm.qCos)>>15)
								-
								(((long)EstimParm.qEsa * (long)SincosParm.qSin)>>15);

    //*******************************
    //*******************************
    // Filter first order for Esd and Esq
    // EsdFilter = 1/TFilterd * Intergal{ (Esd-EsdFilter).dt }
 
		EstimParm.qEsdStateVar			= EstimParm.qEsdStateVar+
									( (long)(EstimParm.qEsd - EstimParm.qEsdf) * (long)EstimParm.qKfilterEsdq) ;
		EstimParm.qEsdf					= (short)(EstimParm.qEsdStateVar>>15);

		EstimParm.qEsqStateVar			= EstimParm.qEsqStateVar+
									( (long)(EstimParm.qEsq - EstimParm.qEsqf) * (long)EstimParm.qKfilterEsdq) ;
		EstimParm.qEsqf					= (short)(EstimParm.qEsqStateVar>>15);

		// OmegaMr= InvKfi * (Esqf -sgn(Esqf) * Esdf)
		// For stability the conditio for low speed
		if (_Q15abs(EstimParm.qVelEstim)>DECIMATE_NOMINAL_SPEED)
		{
			if(EstimParm.qEsqf>0)
			{
				EstimParm.qOmegaMr	=	(((long)MotorEstimParm.qInvKFi*(long)(EstimParm.qEsqf- EstimParm.qEsdf)) >>15) ;
			} 
			else
			{
				EstimParm.qOmegaMr	=	(((long)MotorEstimParm.qInvKFi*(long)(EstimParm.qEsqf + EstimParm.qEsdf))>>15);
			}
		} else // if est speed<10% => condition VelRef<>0
		{
			if(EstimParm.qVelEstim>0)
			{
				EstimParm.qOmegaMr	=	(((long)MotorEstimParm.qInvKFi*(long)(EstimParm.qEsqf- EstimParm.qEsdf))>>15) ;
			} 
			else
			{
				EstimParm.qOmegaMr	=	(((long)MotorEstimParm.qInvKFi*(long)(EstimParm.qEsqf+ EstimParm.qEsdf))>>15) ;
			}
		}
		/* the result of the calculation above is shifted left by one because initally the value of InvKfi */
		/* was shifted by 2 after normalizing - assuring that extended range of the variable is possible in the lookup table */
		/* the initial value of InvKfi is defined in userparms.h */
		EstimParm.qOmegaMr=EstimParm.qOmegaMr<<1;
		
				
			/* the integral of the angle is the estimated angle */
		EstimParm.qRhoStateVar	= 	EstimParm.qRhoStateVar+
								(long)(EstimParm.qOmegaMr)*(long)(EstimParm.qDeltaT);
		EstimParm.qRho 		= 	(short) (EstimParm.qRhoStateVar>>15);


			/* the estiamted speed is a filter value of the above calculated OmegaMr. The filter implementation */
			/* is the same as for BEMF d-q components filtering */
		EstimParm.qVelEstimStateVar=EstimParm.qVelEstimStateVar+
							( (long)(EstimParm.qOmegaMr-EstimParm.qVelEstim)*(long)EstimParm.qVelEstimFilterK );
		EstimParm.qVelEstim=	(short)(EstimParm.qVelEstimStateVar>>15);

}	// End of Estim()

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: InitEstimParm                                               */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Initialisation of the parameters of the estimator.            */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void	InitEstimParm(void)  
{
    // Constants are defined in usreparms.h

		MotorEstimParm.qLsDtBase = NORM_LSDTBASE;
		MotorEstimParm.qLsDt = MotorEstimParm.qLsDtBase;
		MotorEstimParm.qRs = NORM_RS;

		MotorEstimParm.qInvKFiBase = NORM_INVKFIBASE;
		MotorEstimParm.qInvKFi = MotorEstimParm.qInvKFiBase;

		EstimParm.qRhoStateVar=0;
		EstimParm.qOmegaMr=0;
		EstimParm.qDiCounter=0;
		EstimParm.qEsdStateVar=0;
		EstimParm.qEsqStateVar=0;
			
		EstimParm.qDIlimitHS = D_ILIMIT_HS;
		EstimParm.qDIlimitLS = D_ILIMIT_LS;
				
		EstimParm.qKfilterEsdq = KFILTER_ESDQ;
		EstimParm.qVelEstimFilterK = KFILTER_VELESTIM;

		EstimParm.qDeltaT = NORM_DELTAT;
		EstimParm.RhoOffset = INITOFFSET_TRANS_OPEN_CLSD;
}

int32_t _Q15sqrt(int32_t value)
{
	  long root = 0;
	
    step( 0);
    step( 2);
    step( 4);
    step( 6);
    step( 8);
    step(10);
    step(12);
    step(14);
    step(16);
    step(18);
    step(20);
    step(22);
    step(24);
    step(26);
    step(28);
    step(30);

    // round to the nearest integer, cuts max error in half
	if(root < value)++root;
	return root;	
}



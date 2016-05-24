#ifndef UserParms_H
#define UserParms_H
#include "system_define.h"


#define PWMFREQUENCY		15000		// PWM Frequency in Hertz
#define DEADTIMESEC			0.000001	// Deadtime in seconds

#define LOCKTIMEINSEC  1.00		// Initial rotor lock time in seconds
								// Make sure LOCKTIMEINSEC*(1.0/LOOPTIMEINSEC)
								// is less than 65535.

//************** Derived Parameters ****************
#define DFCY        SystemCoreClock		// Instruction cycle frequency (Hz)
#define DTCY        (1.0/DFCY)		// Instruction cycle period (sec)
#define DDEADTIME   (unsigned short)(DEADTIMESEC*DFCY)	// Dead time in dTcys
#define LOOPTIMEINSEC (1.0/PWMFREQUENCY) // PWM Period = 1.0 / PWMFREQUENCY

/* For DEMO purpose, a special definition enables bidirectional functioning */
/* Activating the macro below, a speed reverse will be possible */
/* turning the potentiometer across the median point */
/* In this mode the speed doubling is no longer possible */
#undef BIDIRECTIONAL_SPEED

/* definition for tuning - if active the speed reference is a ramp with a 
 constant slope. The slope is determined by TUNING_DELAY_RAMPUP constant.
 */ 
/* the software ramp implementing the speed increase has a constant slope, */
/* adjusted by the delay TUNING_DELAY_RAMPUP when the speed is incremented.*/
/* The potentiometer speed reference is overwritten. The speed is          */
/* increased from 0 up to the END_SPEED_RPM in open loop ?with the speed  */
/* increase typical to open loop, the transition to closed loop is done    */
/* and the software speed ramp reference is continued up to MAXIMUM_SPEED_RPM. */
//#define TUNING

/* open loop continuous functioning */
/* closed loop transition disabled  */
//#define OPEN_LOOP_FUNCTIONING


/* definition for torque mode - for a separate tuning of the current PI
 controllers, tuning mode will disable the speed PI controller */
//#define TORQUE_MODE
	
#undef DMCI_DEMO	// Define this if a demo with DMCI is done. Start/stop of motor
					// and speed variation is done with DMCI instead of push button
					// and POT. Undefine "DMCI_DEMO" is user requires operating
					// this application with no DMCI

/* static snapshots can be taken activating the SNAPSHOT definition */
/* load dmci_snapshot.dmci window set from DMCI window menu. It contains
 in the Dynamic data view tab the proper variables settup */			
#undef SNAPSHOT

#define DATA_BUFFER_SIZE 50  		//Size in 16-bit Words of the snap */
																// the value depends on the dsPIC mem
#define SNAPDELAY	5 						// In number of PWM Interrupts


#define TUNING_DELAY_RAMPUP   0x10      /* the smaller the value, the quicker the ramp */


#define LOOPTIME_SEC  				LOOPTIMEINSEC           // PWM Period - 50 uSec, 20Khz PWM
#define	DISPLOOPTIME_SEC			0.100		// button polling loop period in sec
#define LOOPTIME_TCY  				(LOOPTIME_SEC/DTCY)   // Basic loop period in units of Tcy



//************** ADC Scaling **************
// Scaling constants: Determined by calibration or hardware design. 
/* the scalling factor for currents are negative because the acquisition for shunts
 is getting reverse sense from LEMs (initialy designed for). */
/* the value of Q15(0.5) represents a 1 multiplication. */

#define     KPOT              Q15(0.99999)   /* scaling factor for pot */
#define     KCURRA            Q15(0.99999)  /* scaling factor for current phase A */
#define     KCURRB            Q15(0.99999)  /* scaling factor for current phase B */

//**************  support xls file definitions begin **************
/* the following values are given in the xls attached file */

//**************  Motor Parameters **************
/* motor's number of pole pairs */
#define NOPOLESPAIRS 2
/* Nominal speed of the motor in RPM */
#define NOMINAL_SPEED_RPM   	2000 // Value in RPM
/* Maximum speed of the motor in RPM - given by the motor's manufacturer */
#define MAXIMUM_SPEED_RPM    	6000 // Value in RPM  


#define NORM_CURRENT_CONST     0.000201
/* normalized rs value */
#define NORM_RS  12774
/* normalized ls/dt value */
#define NORM_LSDTBASE 2248
/* the calculation of Rs gives a value exceeding the Q15 range so,
the normalized value is further divided by 2 to fit the 32768 limit */
/* this is taken care in the estim.c where the value is implied */
/* normalized inv kfi at base speed */
#define NORM_INVKFIBASE  3027
/* the calculation of InvKfi gives a value which not exceed the Q15 limit */
/* to assure that an increase of the term with 5 is possible in the lookup table */
/* for high flux weakening the normalized is initially divided by 2 */
/* this is taken care in the estim.c where the value is implied */
/* normalized dt value */
#define NORM_DELTAT  2384

// Limitation constants 
/* di = i(t1)-i(t2) limitation */ 
/* high speed limitation, for dt 50us */
/* the value can be taken from attached xls file */
#define D_ILIMIT_HS 873
/* low speed limitation, for dt 8*50us */
#define D_ILIMIT_LS 4656

//**************  support xls file definitions end **************


// Filters constants definitions  
/* BEMF filter for d-q components @ low speeds */
#define KFILTER_ESDQ 1200
/* BEMF filter for d-q components @ high speed - Flux Weakening case */
#define KFILTER_ESDQ_FW 164
/* estimated speed filter constatn */
#define KFILTER_VELESTIM 2*374


/* initial offset added to estimated value, */
/* when transitioning from open loop to closed loop */
/* the value represents 45deg and should satisfy both */
/* open loop and closed loop functioning */
/* normally this value should not be modified, but in */
/* case of fine tuning of the transition, depending on */
/* the load or the rotor moment of inertia */
#define INITOFFSET_TRANS_OPEN_CLSD 0x2000

/* current transformation macro, used below */
#define NORM_CURRENT(current_real) (Q15(current_real/NORM_CURRENT_CONST/32768))

/* open loop startup constants */
/* the following values depends on the PWM frequency, */
/* lock time is the time needed for motor's poles alligniament 
 previous the open loop speed ramp up */
#define LOCK_TIME	(unsigned short)(LOCKTIMEINSEC*(1.0/LOOPTIMEINSEC))
/* open loop speed ramp up end value */
#define END_SPEED_RPM 400 // Value in RPM
/* open loop speed ramp up speed of increase */
#define OPENLOOP_RAMPSPEED_INCREASERATE 2
/* open loop q current setup - */
#define Q_CURRENT_REF_OPENLOOP NORM_CURRENT(1.0)

/* in case of the potentimeter speed reference, a reference ramp
 is needed for assuring the motor can follow the reference imposed */
#define    SPEEDREFRAMP   Q15(0.00003)  /*minimum value accepted */

/* PI controllers tuning values - */
//******** D Control Loop Coefficients *******
#define     D_CURRCNTR_PTERM           Q15(0.5)
#define     D_CURRCNTR_ITERM           Q15(0.01)
#define     D_CURRCNTR_CTERM           Q15(0.999)
#define     D_CURRCNTR_OUTMAX          0x7FFF

//******** Q Control Loop Coefficients *******
#define     Q_CURRCNTR_PTERM           Q15(0.5)
#define     Q_CURRCNTR_ITERM           Q15(0.01)
#define     Q_CURRCNTR_CTERM           Q15(0.999)
#define     Q_CURRCNTR_OUTMAX          0x5000

//*** Velocity Control Loop Coefficients *****
#define     SPEEDCNTR_PTERM        Q15(0.9999)
#define     SPEEDCNTR_ITERM        Q15(0.0009)
#define     SPEEDCNTR_CTERM        Q15(0.9999)
#define     SPEEDCNTR_OUTMAX       0x7FFF


//************** Field Weakening **************

/// Field Weakening constant for constant torque range
#define     IDREF_BASESPEED            NORM_CURRENT(0.0)       // Flux reference value

/* the follwing values indicate the d-current variation with speed */
/* please consult app note for details on tuning */
#define	IDREF_SPEED0	NORM_CURRENT(0)       // up to 3000 RPM
#define	IDREF_SPEED1	NORM_CURRENT(-0.24)   // ~3000 RPM
#define	IDREF_SPEED2	NORM_CURRENT(-0.56)   // ~3200 RPM
#define	IDREF_SPEED3	NORM_CURRENT(-0.828)  // ~3400 RPM
#define	IDREF_SPEED4	NORM_CURRENT(-1.121)  // ~3600 RPM
#define	IDREF_SPEED5	NORM_CURRENT(-1.385)  // ~3800 RPM
#define	IDREF_SPEED6	NORM_CURRENT(-1.6)    // ~4000 RPM
#define	IDREF_SPEED7	NORM_CURRENT(-1.8)    // ~4200 RPM
#define	IDREF_SPEED8	NORM_CURRENT(-1.923)  // ~4400 RPM
#define	IDREF_SPEED9	NORM_CURRENT(-2.055)  // ~4600 RPM
#define	IDREF_SPEED10	NORM_CURRENT(-2.15)   // ~4800 RPM
#define	IDREF_SPEED11	NORM_CURRENT(-2.2)    // ~5000 RPM
#define	IDREF_SPEED12	NORM_CURRENT(-2.25)   // ~5200 RPM
#define	IDREF_SPEED13	NORM_CURRENT(-2.3)    // ~5400 RPM
#define	IDREF_SPEED14	NORM_CURRENT(-2.35)   // ~5600 RPM
#define	IDREF_SPEED15	NORM_CURRENT(-2.4)    // ~5800 RPM
#define	IDREF_SPEED16	NORM_CURRENT(-2.423)  // ~6000 RPM
#define	IDREF_SPEED17	NORM_CURRENT(-2.442)  // ~6200 RPM


/* the follwing values indicate the invKfi variation with speed */
/* please consult app note for details on tuning */
#define	INVKFI_SPEED0	7956     // up to 3000 RPM
#define	INVKFI_SPEED1	9300     // ~3000 RPM
#define	INVKFI_SPEED2	10820    // ~3200 RPM
#define	INVKFI_SPEED3	11900    // ~3400 RPM
#define	INVKFI_SPEED4	13110    // ~3600 RPM
#define	INVKFI_SPEED5	14000    // ~3800 RPM
#define	INVKFI_SPEED6	14800    // ~4000 RPM
#define	INVKFI_SPEED7	15350    // ~4200 RPM
#define	INVKFI_SPEED8	15720    // ~4400 RPM
#define	INVKFI_SPEED9	16120    // ~4600 RPM 
#define	INVKFI_SPEED10	16520    // ~4800 RPM
#define	INVKFI_SPEED11	16740    // ~5000 RPM
#define	INVKFI_SPEED12	16920    // ~5200 RPM
#define	INVKFI_SPEED13	17140    // ~5400 RPM
#define	INVKFI_SPEED14	17430    // ~5600 RPM
#define	INVKFI_SPEED15	17650    // ~5800 RPM
#define	INVKFI_SPEED16	17840    // ~6000 RPM
#define	INVKFI_SPEED17	17950    // ~6200 RPM

/* the follwing values indicate the Ls variation with speed */
/* please consult app note for details on tuning */
#define     LS_OVER2LS0_SPEED0            Q15(0.5)   // up to 3000 RPM
#define     LS_OVER2LS0_SPEED1            Q15(0.45)  // ~3000 RPM
#define     LS_OVER2LS0_SPEED2            Q15(0.4)   // ~3200 RPM
#define     LS_OVER2LS0_SPEED3            Q15(0.35)  // ~3400 RPM
#define     LS_OVER2LS0_SPEED4            Q15(0.3)   // ~3600 RPM
#define     LS_OVER2LS0_SPEED5            Q15(0.25)  // ~3800 RPM
#define     LS_OVER2LS0_SPEED6            Q15(0.25)  // ~4000 RPM
#define     LS_OVER2LS0_SPEED7            Q15(0.25)  // ~4200 RPM
#define     LS_OVER2LS0_SPEED8            Q15(0.25)  // ~4400 RPM
#define     LS_OVER2LS0_SPEED9            Q15(0.25)  // ~4600 RPM
#define     LS_OVER2LS0_SPEED10           Q15(0.25)  // ~4800 RPM
#define     LS_OVER2LS0_SPEED11           Q15(0.25)  // ~5000 RPM
#define     LS_OVER2LS0_SPEED12           Q15(0.25)  // ~5200 RPM
#define     LS_OVER2LS0_SPEED13           Q15(0.25)  // ~5400 RPM
#define     LS_OVER2LS0_SPEED14           Q15(0.25)  // ~5600 RPM
#define     LS_OVER2LS0_SPEED15           Q15(0.25)  // ~5800 RPM
#define     LS_OVER2LS0_SPEED16           Q15(0.25)  // ~6000 RPM
#define     LS_OVER2LS0_SPEED17           Q15(0.25)  // ~6200 RPM


 
#endif

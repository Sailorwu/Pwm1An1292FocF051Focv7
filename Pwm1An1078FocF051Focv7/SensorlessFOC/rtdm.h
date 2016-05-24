#include "system_define.h"

#ifndef RTDM_H
#define RTDM_H

//RTDM_User.h
#define YES  1
#define NO 	 0
/************************************** RTDM DEFINITIONS  ***************************************/
#define RTDM_FCY	 	40000000    //This define has to be the system operating freq, this 
									//value is used to calculate the value of the BRG register
#define RTDM_BAUDRATE	38400		//This is the desired baudrate for the UART module to be 
									//used by RTDM
#define RTDM_UART			1		// This is the UART module to be used by RTDM. It has only
									// two possible values: 1 or 2
#define RTDM_UART_PRIORITY	2		//This the UART RX interrupt priority assigned to receive
									// the RTDM messages
#define RTDM_RXBUFFERSIZE	32		// This is the buffer size used by RTDM to handle messaages 
#define RTDM_MAX_XMIT_LEN   0x1000	//This the size in bytes of the max num of bytes allowed in 
									//the RTDM protocol Frame
#define RTDM_POLLING		YES		// This defines the mode that RTDM will be operating in 
									//user's application. If it is YES then the user should place the 
									//RTDM_ProcessMsgs()	function in the main loop. 
									//In order to make sure that the messages are being preoccessed
									// it is recommended that the main loop always polls this 
									//function as fast as possible						
#define RTDM_MIN_CODE_SIZE	YES		//When defined causes the RTDM library to build  without 
									//including a pre-calculated polynomial  table for the CRC algorythim. 
									//This saves 768  bytes of code space. 
/*************************************************************************************************/		

		
 #if defined RTDM_FCY
	#if defined RTDM_BAUDRATE
	 #define RTDM_BRG	(RTDM_FCY/(16*RTDM_BAUDRATE))-1
	#else
	  #error Cannot calculate BRG value. Please define RTDM_BAUDRATE in RTDMUSER.h file
	#endif
 #else
	 #error Cannot calculate RTDM_BRG value. Please define RTDM_FCY in RTDMUSER.h file
 #endif

 #define RTDM_BAUDRATE_ACTUAL	(RTDM_FCY/(16*(RTDM_BRG+1)))
 #define RTDM_BAUD_ERROR		((RTDM_BAUDRATE_ACTUAL > RTDM_BAUDRATE) ? RTDM_BAUDRATE_ACTUAL - RTDM_BAUDRATE : RTDM_BAUDRATE - RTDM_BAUDRATE_ACTUAL)
 #define RTDM_BAUD_ERROR_PERCENT	(((RTDM_BAUD_ERROR*100)+(RTDM_BAUDRATE/2))/RTDM_BAUDRATE)

 #if	(RTDM_BAUD_ERROR_PERCENT > 2)
	 #error The value loaded to the BRG register produces a baud rate error higher than 2%
 #endif


/**********************  RTDM FUNCTIONS **************************/
int RTDM_ProcessMsgs();
int RTDM_Close();
int RTDM_Start();
unsigned int RTDM_CumulativeCrc16 (unsigned char *buf, unsigned int u16Length, unsigned int u16CRC);

#endif


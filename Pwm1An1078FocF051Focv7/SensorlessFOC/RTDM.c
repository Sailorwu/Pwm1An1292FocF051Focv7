#include "RTDM.h"

#ifdef RTDM

/*+++++++++++++++++++++++++++++++ RTDM Variables ++++++++++++++++++++++++++++++++++++++++*/
/* Received data is stored in array RTDMRxBuffer  */
unsigned char RTDMRxBuffer[RTDM_RXBUFFERSIZE];
unsigned char * RTDMRxBufferLoLimit = RTDMRxBuffer;
unsigned char * RTDMRxBufferHiLimit = RTDMRxBuffer + RTDM_RXBUFFERSIZE - 1;
unsigned char * RTDMRxBufferIndex = RTDMRxBuffer;
unsigned char * RTDMRxBufferStartMsgPointer;
unsigned char * RTDMRxBufferEndMsgPointer;

/* Data to be transmitted using UART communication module */
const unsigned char RTDMTxdata[] = {'R','T','D','M','\0'};
const unsigned char RTDMSanityCheckOK[] = {'+','$','R','T','D','M','#',0x1B,0x86,'\0'}; 
const unsigned char RTDMWriteMemoryOK[] = {'+','$','O','K','#',0x4C,0x08,'\0'};
const unsigned char RTDMErrorIllegalFunction[] = {'-','$','E',0x01,'#',0xD3,0x6A,'\0'};
unsigned char RTDMErrorFrame[] = {'-','$','E',0,'#',0,0,'\0'};

/* Temp variables used to calculate the CRC16*/
unsigned int RTDMcrcTemp,RTDMcrcTempH,RTDMcrcTempL;

/*Structure enclosing the RTDM flags*/
struct {
			unsigned MessageReceived  :		1;
			unsigned TransmitNow	  :		1;
			unsigned unused :		14;     
		}	RTDMFlags; 


/* UART Configuration data */
/* Holds the value of uart config reg */
unsigned int RTDM_UART_MODE_VALUE;
/* Holds the information regarding uart	TX & RX interrupt modes */    	
unsigned int RTDM_UART_STA_VALUE; 

#if (RTDM_UART == 1)
/******************************************************************************
* Function:     RTDM_Start()
*
* Output:		return 0 if no errors
*
* Overview:	Here is where the RTDM code initilizes the UART to be used to
*			exchange data wiht the host PC
*
* Note:		Some processors may have 2 UART modules, that is why it is required to
*			specify wich UART module is going to be used by RTDM	
*******************************************************************************/
#if defined (RTDM_UART_V2)
int RTDM_Start()
{

	/********************** UART CONFIGURATIION ***************************/
	/* Turn off UART1 module */	
	CloseUART1();
	/* Configure UART2 receive and transmit interrupt */
	ConfigIntUART1(UART_RX_INT_EN & (UART_RX_INT_PR0+RTDM_UART_PRIORITY) & UART_TX_INT_DIS & UART_TX_INT_PR2);
	/* Configure UART1 module to transmit 8 bit data with one stopbit.  */ 
	RTDM_UART_MODE_VALUE = UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE &
				UART_MODE_FLOW & UART_UEN_00 & UART_DIS_WAKE & 
				UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & 
				UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT;



	
	RTDM_UART_STA_VALUE  = UART_INT_TX_BUF_EMPTY  & UART_IrDA_POL_INV_ZERO &                  
	            UART_SYNC_BREAK_DISABLED & UART_TX_ENABLE & UART_INT_RX_CHAR & 
	            UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR;
	
	
	OpenUART1(RTDM_UART_MODE_VALUE, RTDM_UART_STA_VALUE, RTDM_BRG);
	
	/************* RTDM Flags Configuration & Initial Values *****************/
	RTDMFlags.MessageReceived = 0;
	RTDMFlags.MessageReceived = 0;
	RTDMRxBufferIndex = RTDMRxBufferLoLimit;
	RTDMRxBufferStartMsgPointer = 	RTDMRxBufferLoLimit;		
	RTDMRxBufferEndMsgPointer = RTDMRxBufferLoLimit;

    return 0;
}
#elif defined (RTDM_UART_V1)
int RTDM_Start()
{

	/********************** UART CONFIGURATIION ***************************/
	/* Turn off UART1 module */	
	CloseUART1();
	/* Configure UART2 receive and transmit interrupt */
	ConfigIntUART1(UART_RX_INT_EN & (UART_RX_INT_PR0+RTDM_UART_PRIORITY) & UART_TX_INT_DIS & UART_TX_INT_PR2);
	/* Configure UART1 module to transmit 8 bit data with one stopbit.  */ 
	RTDM_UART_MODE_VALUE = UART_EN & UART_IDLE_CON & 
                         UART_DIS_WAKE & UART_DIS_LOOPBACK & 
                         UART_DIS_ABAUD & UART_NO_PAR_8BIT & 
                         UART_1STOPBIT;
	
	RTDM_UART_STA_VALUE  = UART_INT_TX_BUF_EMPTY & UART_TX_PIN_NORMAL & 
                         UART_TX_ENABLE & UART_INT_RX_CHAR & 
	                       UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR;
	
	
	OpenUART1(RTDM_UART_MODE_VALUE, RTDM_UART_STA_VALUE, RTDM_BRG);
	
	/************* RTDM Flags Configuration & Initial Values *****************/
	RTDMFlags.MessageReceived = 0;
	RTDMFlags.MessageReceived = 0;
	RTDMRxBufferIndex = RTDMRxBufferLoLimit;
	RTDMRxBufferStartMsgPointer = 	RTDMRxBufferLoLimit;		
	RTDMRxBufferEndMsgPointer = RTDMRxBufferLoLimit;

    return 0;
}
#endif

/******************************************************************************
* Function:     	CloseRTDM()
*
* Output:		return 0 if no errors
*
* Overview:	Here is where the RTDM code closes the UART used to
*			exchange data wiht the host PC
*
* Note:		Some processors may have 2 UART modules, that is why it is required to
*			specify wich UART module is going to be used by RTDM	
*******************************************************************************/
int CloseRTDM()
{
	int nRet = 0;
	CloseUART1();
	return nRet;
		
}

/******************************************************************************
* Function:     	RTDM_ProcessMsgs()
*
* Output:		return 0 if no errors
*
* Overview:	Here is where the RTDM code process the message received and then 
*			executes the required task. These tasks are reading an specified memory
*			location, writing an specified memory location, receive a communication
*			link sanity check command, or being asked for the size of the bufffers.
*
* Note:		Some processors may have 2 UART modules, that is why it is required to
*			specify wich UART module is going to be used by RTDM	
*******************************************************************************/
int RTDM_ProcessMsgs()
{
	
	//Local pointer management variables
	unsigned long int * RTDMpu32AddressTemp;
	unsigned char     * RTDMpucWrData;
    unsigned char     * RTDMpucRdData;
    unsigned char     * RTDMpucWrAddr;	
    unsigned short      RTDMNumBytes;
    unsigned char       RTDMPacketBuf[16];
    
	unsigned int        RTDMProcessMsgsTemp1, RTDMProcessMsgsTemp2;
	unsigned int        N;

    if (!RTDMFlags.MessageReceived)
	{
		return -1;
	}


	RTDMcrcTemp = 
		RTDM_CumulativeCrc16
			(RTDMRxBufferStartMsgPointer, 
			(unsigned int)(RTDMRxBufferEndMsgPointer-RTDMRxBufferStartMsgPointer)+1, 
			0xFFFF);
			
	RTDMcrcTempH = (RTDMcrcTemp & 0xFF00)>>8;
	RTDMcrcTempL = RTDMcrcTemp & 0x00FF;
	RTDMProcessMsgsTemp1 = (unsigned int)*((RTDMRxBufferEndMsgPointer)+2);
	RTDMProcessMsgsTemp2 = (unsigned int)*((RTDMRxBufferEndMsgPointer)+1);



	RTDMRxBufferStartMsgPointer +=2;
	if((RTDMProcessMsgsTemp1 == (unsigned)RTDMcrcTempH) && (RTDMProcessMsgsTemp2 == RTDMcrcTempL))
	  {

		switch(*((RTDMRxBufferLoLimit)+1))
		  {
		  case 'm':
			/*************** Extract Address **************/
			//Capture address as 32 bit quantity to match protocol definition. 
			RTDMpu32AddressTemp = ((unsigned long *) RTDMRxBufferStartMsgPointer);
			
			//Increment receive buffer pointer to length field.
			RTDMRxBufferStartMsgPointer += sizeof(unsigned long);
			 
			//Init a byte oriented data pointer  
			RTDMpucRdData = (unsigned char *) ((unsigned int) *RTDMpu32AddressTemp);

			/********* Extract Number of Bytes ************/			
			//Capture address as 16 bit quantity to match protocol definition. 
			RTDMNumBytes = *((unsigned short *) RTDMRxBufferStartMsgPointer);
			
			//Increment receive buffer pointer to start of data payload.
			RTDMRxBufferStartMsgPointer += sizeof(unsigned short);
			
            //Init the CRC seed for the cumulative checksum calculation. 					
			RTDMcrcTemp = 0xffff;
			
			//Add packet header prefix
			RTDMPacketBuf[0] = '+';
			RTDMPacketBuf[1] = '$';
			//Add null terminator for putsUARTx function...
			RTDMPacketBuf[2] = 0; 
			
			//Calc header prefix checksum piece
			RTDMcrcTemp = RTDM_CumulativeCrc16(RTDMPacketBuf, 2, RTDMcrcTemp);	
			//Calc data payload checksum
			RTDMcrcTemp = RTDM_CumulativeCrc16(RTDMpucRdData, RTDMNumBytes, RTDMcrcTemp);			
			
			//Send packet header. Use string function to save code space... 
			putsUART1((unsigned int *)RTDMPacketBuf);
			while(BusyUART1()); 
			
			//Send data portion of message... 
			while(RTDMNumBytes--)
			 {
				WriteUART1(*RTDMpucRdData++);
				while(BusyUART1()); 
			 }
			
			//Add packet trailer   
		    RTDMPacketBuf[0] = '#';
		    RTDMcrcTemp = RTDM_CumulativeCrc16(RTDMPacketBuf, 1, RTDMcrcTemp);			
		    
		    //Add checksum bytes to packet
			RTDMPacketBuf[1] = RTDMcrcTemp & 0x00FF;
			RTDMPacketBuf[2] = (RTDMcrcTemp & 0xFF00) >> 8;
			
			//Send packet trailer and checksum. 			
			for (N=0; N < 3; N++)
			{
				WriteUART1(RTDMPacketBuf[N]); 
				while(BusyUART1()); 
			}	
		  break;
		  
		  case 'M':
			{
			/*************** Extract Address **************/						
		    //Capture address as 32 bit quantity to match protocol definition. 
		    RTDMpu32AddressTemp = (unsigned long *) RTDMRxBufferStartMsgPointer;
		    
		    //Increment receive buffer pointer to length field.
		    RTDMRxBufferStartMsgPointer += sizeof(unsigned long);
		    		    
			// Init a byte oriented address pointer for use in incrementing 
			//through the address range properly as we write each byte of data
			//in the range (length) of this write request.   
			RTDMpucWrAddr = (unsigned char *) ((unsigned int) *RTDMpu32AddressTemp);

			/********* Extract Number of Bytes ************/
			//MEllis Capture length as 16 bit quantity to match protocol definition.
			RTDMNumBytes = *((unsigned short *) RTDMRxBufferStartMsgPointer);
			
			//MEllis Increment receive buffer pointer to start of data payload.
			RTDMRxBufferStartMsgPointer += sizeof(unsigned short);
			
			/********** Extract Data ************/
			
			//Init a byte oriented data pointer so that we can increment a byte at at 
			//time for as many bytes as are in the range for this write. 
			RTDMpucWrData = RTDMRxBufferStartMsgPointer;
			
			//*** Write Data in specified RAM location *****			
			//Important to increment through address range using byte oriented address and data
			//pointers. Otherwise, single byte or odd byte ranges do not get written correctly. 
			while(RTDMNumBytes--)
			  {
    			*RTDMpucWrAddr++ = *RTDMpucWrData++;
    		  }
			  			
			//Transmit OK message
			putsUART1((unsigned int *)RTDMWriteMemoryOK);
			/* Wait for  transmission to complete */
			while(BusyUART1()); 
			break;
			}
		  case 's':
			{
			/* Load transmit buffer and transmit the same till null character is encountered */
			//Transmit OK message
			putsUART1((unsigned int *)RTDMSanityCheckOK);
			/* Wait for  transmission to complete */
			while(BusyUART1()); 
		    break;
			}
		  
		  case 'L':
		    RTDMcrcTemp = 0xffff; //Init the CRC seed.
			
			RTDMPacketBuf[0] = '+';
			RTDMPacketBuf[1] = '$';
			//Size of the RTDM Receive buffer.
			RTDMPacketBuf[2] = (sizeof(RTDMRxBuffer) & 0x00FF);
			RTDMPacketBuf[3] = (sizeof(RTDMRxBuffer) & 0xFF00) >> 8;
			//Note: We dod not utilize a transmit buffer since any data memory source is 
			//essentially already buffered. So the transmit limit is now just a way to 
			//limit the total message length that a client make with any single read request.
			RTDMPacketBuf[4] = (RTDM_MAX_XMIT_LEN & 0x00FF);
			RTDMPacketBuf[5] = (RTDM_MAX_XMIT_LEN & 0xFF00) >> 8;
			RTDMPacketBuf[6] = '#';		
			RTDMcrcTemp = RTDM_CumulativeCrc16(RTDMPacketBuf, 7, RTDMcrcTemp);			
			RTDMPacketBuf[7] = (RTDMcrcTemp & 0x00FF);
			RTDMPacketBuf[8] = (RTDMcrcTemp & 0xFF00) >> 8;

			//Send completed message which is 9 bytes in length.
			for (N=0; N < 9; N++)
			{
				WriteUART1(RTDMPacketBuf[N]);
				while(BusyUART1()); 
			}	
			break;
		  
		  default:
			// ---> COMMAND SUPPORTED?? IF NOT ERROR HANDLER
			//Transmit ERROR message 1
			putsUART1 ((unsigned int *)RTDMErrorIllegalFunction);
			/* Wait for  transmission to complete */
			while(BusyUART1()); 
			break;
		  }
				   
	  }

	  memset(&RTDMRxBuffer, 0, sizeof(RTDMRxBuffer));
	  
	  RTDMFlags.MessageReceived = 0;
	  RTDMRxBufferIndex             = RTDMRxBufferLoLimit;
	  RTDMRxBufferStartMsgPointer   = RTDMRxBufferLoLimit;		
	  RTDMRxBufferEndMsgPointer     = RTDMRxBufferLoLimit;
	  
	  return 0;
}	

/******************************************************************************
* Function:     	_U1RXInterrupt(void)
*
* Output:		void
*
* Overview:	Here is where the RTDM receives the messages using the UART receiver 
*			interrupt, If polling method is selected in the RTDMUSER.h file then
*			the user application should call the RTDM_ProcessMsgs routine in order 
*			to precess up comming messages. If polling method is disabled then the 
*			RTDM_ProcessMsgs routine is called in the UART received interrupt
*			routine.
*
* Note:		Some processors may have 2 UART modules, that is why it is required to
*			specify wich UART module is going to be used by RTDM	
*******************************************************************************/
#if (RTDM_POLLING == YES)
/* This is UART1 receive ISR Polling RTDM Messages*/ 
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) 
{
    
    _U1RXIF = 0;
    
	/* Read the receive buffer until at least one or more character can be read */  
    while(DataRdyUART1())
      *(RTDMRxBufferIndex++) = ReadUART1();		

	RTDMRxBufferEndMsgPointer = RTDMRxBufferIndex-3;
	if(RTDMRxBufferIndex > (RTDMRxBufferHiLimit-1))
	  {
	  RTDMRxBufferIndex = RTDMRxBufferLoLimit;
	  RTDMRxBufferEndMsgPointer = RTDMRxBufferHiLimit-1;
	  }
	
	if(*(RTDMRxBufferStartMsgPointer) == '$')
	{
	  if(*(RTDMRxBufferEndMsgPointer) == '#')
	  {
      	RTDMFlags.MessageReceived = 1;
      }
    }  
	else
	{
		RTDMRxBufferIndex = RTDMRxBufferLoLimit;
	}
	
}  

/******************************************************************************
* Function:     	_U1RXInterrupt(void)
*
* Output:		void
*
* Overview:	Here is where the RTDM receives the messages using the UART receiver 
*			interrupt, If polling method is selected in the RTDMUSER.h file then
*			the user application should call the RTDM_ProcessMsgs routine in order 
*			to precess up comming messages. If polling method is disabled then the 
*			RTDM_ProcessMsgs routine is called in the UART received interrupt
*			routine.
*
* Note:		Some processors may have 2 UART modules, that is why it is required to
*			specify wich UART module is going to be used by RTDM	
*******************************************************************************/
#else
/* This is UART1 receive ISR without polling RTDM Messages */ 
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) 
{
    
    _U1RXIF = 0;
    
	/* Read the receive buffer until at least one or more character can be read */  
    while(DataRdyUART1())
      *(RTDMRxBufferIndex++) = ReadUART1();		

	RTDMRxBufferEndMsgPointer = RTDMRxBufferIndex-3;
	if(RTDMRxBufferIndex > (RTDMRxBufferHiLimit-1))
	  {
	  RTDMRxBufferIndex = RTDMRxBufferLoLimit;
	  RTDMRxBufferEndMsgPointer = RTDMRxBufferHiLimit-1;
	  }
	
	if(*(RTDMRxBufferStartMsgPointer) == '$')
	{
	  if(*(RTDMRxBufferEndMsgPointer) == '#')
	  {
      	RTDMFlags.MessageReceived = 1;
      	RTDM_ProcessMsgs();
      }
    }  
	else
	{
		RTDMRxBufferIndex = RTDMRxBufferLoLimit;
	}
	
}  
#endif

#elif (RTDM_UART == 2)
/******************************************************************************
* Function:    	 RTDM_Start()
*
* Output:		return 0 if no errors
*
* Overview:	Here is where the RTDM code initilizes the UART to be used to
*			exchange data wiht the host PC
*
* Note:		Some processors may have 2 UART modules, that is why it is required to
*			specify wich UART module is going to be used by RTDM	
*******************************************************************************/
#if defined (RTDM_UART_V2)
int RTDM_Start()
{

	/********************** UART CONFIGURATIION ******************************/
	/* Turn off UART2 module */	
	CloseUART2();
	
	/* Configure UART2 receive and transmit interrupt */
	ConfigIntUART2(UART_RX_INT_EN & (UART_RX_INT_PR0+RTDM_UART_PRIORITY)& UART_TX_INT_DIS & UART_TX_INT_PR2);
	
	/* Configure UART2 module to transmit 8 bit data with one stopbit.  */ 
	RTDM_UART_MODE_VALUE = UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE &
				UART_MODE_FLOW & UART_UEN_00 & UART_DIS_WAKE & 
				UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & 
				UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT;



	
	RTDM_UART_STA_VALUE  = UART_INT_TX_BUF_EMPTY  & UART_IrDA_POL_INV_ZERO &                  
	            UART_SYNC_BREAK_DISABLED & UART_TX_ENABLE & UART_INT_RX_CHAR & 
	            UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR;
	
	
	OpenUART2(RTDM_UART_MODE_VALUE, RTDM_UART_STA_VALUE, RTDM_BRG);
	
	
	/************* RTDM Flags Configuration & Initial Values *****************/
	RTDMFlags.MessageReceived = 0;
	RTDMFlags.MessageReceived = 0;
	RTDMRxBufferIndex = RTDMRxBufferLoLimit;
	RTDMRxBufferStartMsgPointer = 	RTDMRxBufferLoLimit;		
	RTDMRxBufferEndMsgPointer = RTDMRxBufferLoLimit;

    return 0;
}
#elif defined (RTDM_UART_V1)
int RTDM_Start()
{

	/********************** UART CONFIGURATIION ******************************/
	/* Turn off UART2 module */	
	CloseUART2();
	
	/* Configure UART2 receive and transmit interrupt */
	ConfigIntUART2(UART_RX_INT_EN & (UART_RX_INT_PR0+RTDM_UART_PRIORITY)& UART_TX_INT_DIS & UART_TX_INT_PR2);
	
	/* Configure UART2 module to transmit 8 bit data with one stopbit.  */ 
	RTDM_UART_MODE_VALUE = UART_EN & UART_IDLE_CON & 
                       UART_DIS_WAKE & UART_DIS_LOOPBACK & 
                       UART_DIS_ABAUD & UART_NO_PAR_8BIT & 
                       UART_1STOPBIT;
	
	RTDM_UART_STA_VALUE  = UART_INT_TX_BUF_EMPTY & UART_TX_PIN_NORMAL & 
                       UART_TX_ENABLE & UART_INT_RX_CHAR & 
                       UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR;
	
	
	OpenUART2(RTDM_UART_MODE_VALUE, RTDM_UART_STA_VALUE, RTDM_BRG);
	
	
	/************* RTDM Flags Configuration & Initial Values *****************/
	RTDMFlags.MessageReceived = 0;
	RTDMFlags.MessageReceived = 0;
	RTDMRxBufferIndex = RTDMRxBufferLoLimit;
	RTDMRxBufferStartMsgPointer = 	RTDMRxBufferLoLimit;		
	RTDMRxBufferEndMsgPointer = RTDMRxBufferLoLimit;

    return 0;
}
#endif
/******************************************************************************
* Function:     	CloseRTDM()
*
* Output:		return 0 if no errors
*
* Overview:	Here is where the RTDM code closes the UART used to
*			exchange data wiht the host PC
*
* Note:		Some processors may have 2 UART modules, that is why it is required to
*			specify wich UART module is going to be used by RTDM	
*******************************************************************************/
int CloseRTDM()
{
	int nRet = 0;
	CloseUART2();	
	return nRet;
}

/******************************************************************************
* Function:     	RTDM_ProcessMsgs()
*
* Output:		return 0 if no errors
*
* Overview:	Here is where the RTDM code process the message received and then 
*			executes the required task. These tasks are reading an specified memory
*			location, writing an specified memory location, receive a communication
*			link sanity check command, or being asked for the size of the bufffers.
*
* Note:		Some processors may have 2 UART modules, that is why it is required to
*			specify wich UART module is going to be used by RTDM	
*******************************************************************************/
int RTDM_ProcessMsgs()
{
	
	//Local pointer management variables
	unsigned long int * RTDMpu32AddressTemp;
	unsigned char     * RTDMpucWrData;
    unsigned char     * RTDMpucRdData;
    unsigned char     * RTDMpucWrAddr;	
    unsigned short      RTDMNumBytes;
    unsigned char       RTDMPacketBuf[16];
    
	unsigned int        RTDMProcessMsgsTemp1, RTDMProcessMsgsTemp2;
	unsigned int        N;

    if (!RTDMFlags.MessageReceived)
	{
		return -1;
	}


	RTDMcrcTemp = 
		RTDM_CumulativeCrc16
		(RTDMRxBufferStartMsgPointer, 
		(unsigned int)(RTDMRxBufferEndMsgPointer-RTDMRxBufferStartMsgPointer)+1, 
		0xFFFF);
		
	RTDMcrcTempH = (RTDMcrcTemp & 0xFF00)>>8;
	RTDMcrcTempL = RTDMcrcTemp & 0x00FF;
	RTDMProcessMsgsTemp1 = (unsigned int)*((RTDMRxBufferEndMsgPointer)+2);
	RTDMProcessMsgsTemp2 = (unsigned int)*((RTDMRxBufferEndMsgPointer)+1);

	RTDMRxBufferStartMsgPointer +=2;
	if((RTDMProcessMsgsTemp1 == (unsigned)RTDMcrcTempH) && (RTDMProcessMsgsTemp2 == RTDMcrcTempL))
	  {

		switch(*((RTDMRxBufferLoLimit)+1))
		  {
		  case 'm':
			/*************** Extract Address **************/
			//Capture address as 32 bit quantity to match protocol definition. 
			RTDMpu32AddressTemp = ((unsigned long *) RTDMRxBufferStartMsgPointer);
			
			//Increment receive buffer pointer to length field.
			RTDMRxBufferStartMsgPointer += sizeof(unsigned long);
			 
			//Init a byte oriented data pointer  
			RTDMpucRdData = (unsigned char *) ((unsigned int) *RTDMpu32AddressTemp);

			/********* Extract Number of Bytes ***********/			
			//Capture address as 16 bit quantity to match protocol definition. 
			RTDMNumBytes = *((unsigned short *) RTDMRxBufferStartMsgPointer);
			
			//Increment receive buffer pointer to start of data payload.
			RTDMRxBufferStartMsgPointer += sizeof(unsigned short);
			
            //Init the CRC seed for the cumulative checksum calculation. 					
			RTDMcrcTemp = 0xffff;
			
			//Add packet header prefix
			RTDMPacketBuf[0] = '+';
			RTDMPacketBuf[1] = '$';
			//Add null terminator for putsUARTx function...
			RTDMPacketBuf[2] = 0; 
			
			//Calc header prefix checksum piece
			RTDMcrcTemp = RTDM_CumulativeCrc16(RTDMPacketBuf, 2, RTDMcrcTemp);	
			//Calc data payload checksum
			RTDMcrcTemp = RTDM_CumulativeCrc16(RTDMpucRdData, RTDMNumBytes, RTDMcrcTemp);			
			
			//Send packet header. Use string function to save code space... 
			putsUART2 ((unsigned int *)RTDMPacketBuf);
			while(BusyUART2()); 
			
			//Send data portion of message... 
			while(RTDMNumBytes--)
			 {
				WriteUART2(*RTDMpucRdData++);
				while(BusyUART2()); 
			 }
			
			//Add packet trailer   
		    RTDMPacketBuf[0] = '#';
		    RTDMcrcTemp = RTDM_CumulativeCrc16(RTDMPacketBuf, 1, RTDMcrcTemp);			
		    
		    //Add checksum bytes to packet
			RTDMPacketBuf[1] = RTDMcrcTemp & 0x00FF;
			RTDMPacketBuf[2] = (RTDMcrcTemp & 0xFF00) >> 8;
			
			//Send packet trailer and checksum. 			
			for (N=0; N < 3; N++)
			{
				WriteUART2(RTDMPacketBuf[N]); 
				while(BusyUART2()); 
			}	
		  break;
		  
		  case 'M':
			{
			/*************** Extract Address **************/						
		    //Capture address as 32 bit quantity to match protocol definition. 
		    RTDMpu32AddressTemp = (unsigned long *) RTDMRxBufferStartMsgPointer;
		    
		    //Increment receive buffer pointer to length field.
		    RTDMRxBufferStartMsgPointer += sizeof(unsigned long);
		    		    
			//Init a byte oriented address pointer for use in incrementing 
			//through the address range properly as we write each byte of data
			//in the range (length) of this write request.   
			RTDMpucWrAddr = (unsigned char *) ((unsigned int) *RTDMpu32AddressTemp);

			/********* Extract Number of Bytes ************/
			//Capture length as 16 bit quantity to match protocol definition.
			RTDMNumBytes = *((unsigned short *) RTDMRxBufferStartMsgPointer);
			
			//Increment receive buffer pointer to start of data payload.
			RTDMRxBufferStartMsgPointer += sizeof(unsigned short);
			
			/********** Extract Data ************/			
			//Init a byte oriented data pointer so that we can increment a byte at at 
			//time for as many bytes as are in the range for this write. 
			RTDMpucWrData = RTDMRxBufferStartMsgPointer;
			
			//*** Write Data in specified RAM location *****			
			//Important to increment through address range using byte oriented address and data
			//pointers. Otherwise, single byte or odd byte ranges do not get written correctly. 
			while(RTDMNumBytes--)
			  {
    			*RTDMpucWrAddr++ = *RTDMpucWrData++;
    		  }
			  			
			//Transmit OK message
			putsUART2((unsigned int *)RTDMWriteMemoryOK);
			/* Wait for  transmission to complete */
			while(BusyUART2()); 
			break;
			}
		  case 's':
			{
			/* Load transmit buffer and transmit the same till null character is encountered */
			//Transmit OK message
			putsUART2((unsigned int *)RTDMSanityCheckOK);
			/* Wait for  transmission to complete */
			while(BusyUART2()); 
		    break;
			}
		  
		  case 'L':
		    RTDMcrcTemp = 0xffff; //Init the CRC seed.
			
			RTDMPacketBuf[0] = '+';
			RTDMPacketBuf[1] = '$';
			//Size of the RTDM Receive buffer.
			RTDMPacketBuf[2] = (sizeof(RTDMRxBuffer) & 0x00FF);
			RTDMPacketBuf[3] = (sizeof(RTDMRxBuffer) & 0xFF00) >> 8;
			//Note: We dod not utilize a transmit buffer since any data memory source is 
			//essentially already buffered. So the transmit limit is now just a way to 
			//limit the total message length that a client make with any single read request.
			RTDMPacketBuf[4] = (RTDM_MAX_XMIT_LEN & 0x00FF);
			RTDMPacketBuf[5] = (RTDM_MAX_XMIT_LEN & 0xFF00) >> 8;
			RTDMPacketBuf[6] = '#';		
			RTDMcrcTemp = RTDM_CumulativeCrc16(RTDMPacketBuf, 7, RTDMcrcTemp);			
			RTDMPacketBuf[7] = (RTDMcrcTemp & 0x00FF);
			RTDMPacketBuf[8] = (RTDMcrcTemp & 0xFF00) >> 8;

			//Send completed message which is 9 bytes in length.
			for (N=0; N < 9; N++)
			{
				WriteUART2(RTDMPacketBuf[N]);
				while(BusyUART2()); 
			}	
			break;

		  default:
			// ---> COMMAND SUPPORTED?? IF NOT ERROR HANDLER
			//Transmit ERROR message 1
			putsUART2 ((unsigned int *)RTDMErrorIllegalFunction);
			/* Wait for  transmission to complete */
			while(BusyUART2()); 
			break;
		  }
				   
	  }

	  memset(&RTDMRxBuffer, 0, sizeof(RTDMRxBuffer));
	  
	  RTDMFlags.MessageReceived = 0;
	  RTDMRxBufferIndex             = RTDMRxBufferLoLimit;
	  RTDMRxBufferStartMsgPointer   = RTDMRxBufferLoLimit;		
	  RTDMRxBufferEndMsgPointer     = RTDMRxBufferLoLimit;
	  
	  return 0;
}	
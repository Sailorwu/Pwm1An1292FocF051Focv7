#include "Contrl.h"

uint8_t Motor_State,FOC_Flag,ShortI_Flag=0,ShortI_Counter;
int16_t hPhaseAOffset,hPhaseBOffset;	
int16_t SPOffset,VoltageOffset;
uint16_t SB_FilterHe,SB_Temp,SpeedReference=3000;
// ADC 0: NC    1:current      2: SP   3:DC

LPF_16PARAMETERS SB_Filter_t,DC_Filter_t,MotorSpeed_t;
	
void Main_Loop(void)
{
	static uint16_t DelayStartT;
	if(T2ms_Flag)
	{		
		T2ms_Flag=0;
		SB_FilterHe=SP_16LPF(&SB_Filter_t,ADC_Tab[SP_Channl],8);
		SB_Temp=SB_FilterHe;					//12bit
		
		if(SB_Temp>100) 
		{
			if((Motor_State!=RUN)||(DelayStartT>1000)) 
			{
				Motor_Start();
				DelayStartT=0;
				uGF.bit.MotorFail=0;
			}
			
			if(uGF.bit.MotorFail) DelayStartT++;			
		}	
		else if(SB_Temp>90) ;
		else 
		{
			if(Motor_State!=WAIT) Motor_Stop();

		}
		
		//if(Motor_State==RUN) Motor_Run();	
	}
	
	if(T100ms_Flag)
	{
		T100ms_Flag=0;
		LED2Toggle();	
	}
}


void HardwareInit(void)
{
	GPIO_Iinitialization();
	TMER_Iinitialization();
	UART_Iinitialization();
	ADC_Iinitialization();
	Motor_Init();
	//DAC_Iinitialization();
	
	SensorlessFOCinit();
}

void Motor_Init(void)
{
	Motor_State=INIT;
	SB_Filter_t.acc=0;
}

void Motor_Start(void)
{
	Motor_Stop();
	
	Open_PWM();
	Motor_State=RUN;
	ShortI_Flag=0;
// init Mode
	uGF.bit.ChangeSpeed = 0;
	uGF.bit.OpenLoop = 1;           // start in openloop
  uGF.bit.RunMotor = 1;           //then start motor
// Run the motor
  uGF.bit.ChangeMode = 1;	// Ensure variable initialization when open loop is	
}

void Motor_Stop(void)
{
	Close_PWM();
	Motor_State=WAIT;
	
	uGF.Word = 0;                   // clear flags

	/* setup to openloop */
	uGF.bit.OpenLoop = 1;
	/* set the reference speed value to 0 */
	CtrlParm.qVelRef = 0;
	// change speed 
	uGF.bit.ChangeSpeed =0;
	// change mode 
	uGF.bit.ChangeMode =0;
	// begin stop sequence
	uGF.bit.RunMotor = 0;
	
	/* init PI control parameters */
	InitControlParameters();        
	/* init estim parameters */
	InitEstimParm();
	/* init flux weakening params */
	InitFWParams();
	
	// zero out i sums 
	PIParmD.qdSum = 0;
	PIParmQ.qdSum = 0;
	PIParmQref.qdSum = 0;
}

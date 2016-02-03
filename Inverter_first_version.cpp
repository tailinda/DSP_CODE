//###########################################################################
// Description:
//! \addtogroup f2803x_example_list
//! <h1>Cpu Timer (cpu_timer)</h1>
//!
//! This example configures CPU Timer0, 1, and 2 and increments
//! a counter each time the timer asserts an interrupt.
//!
//! \b Watch \b Variables \n
//! - CpuTimer0.InterruptCount
//! - CpuTimer1.InterruptCount
//! - CpuTimer2.InterruptCount
//
//###########################################################################
// $TI Release: F2803x C/C++ Header Files and Peripheral Examples V130 $
// $Release Date: May  8, 2015 $
// $Copyright: Copyright (C) 2009-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

// Prototype statements for functions found within this file.
__interrupt void cpu_timer0_isr(void);
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitPIvalue(void);
int ADCValue[256] , ADCCount = 0;
float v1, v2, verr, vk1, vk2, vout, vr, z1=0, z2=0, vko1, vko2, vou1, vou2;
float kP, kI ;
int i ; // Loop Count
int sintable1[256]=
{
	 0,	    13,	    25,	    38,	    50,	    63,	    75,	    88,	   100,	   112,	   124,	   137,	   149,	   161,	   172,	   184,
   196,	   207,	   219,	   230,	   241,	   252,	   263,	   274,	   284,	   295,	   305,	   315,	   325,	   334,	   344,	   353,
   362,	   371,	   379,	   388,	   396,	   404,	   411,	   419,	   426,	   433,	   439,	   445,	   452,	   457,	   463,	   468,
   473,	   478,	   482,	   486,	   490,	   493,	   497,	   500,	   502,	   504,	   506,	   508,	   510,	   511,	   511,	   512,
   512,	   512,	   511,	   511,	   510,	   508,	   506,	   504,	   502,	   500,	   497,	   493,	   490,	   486,	   482,	   478,
   473,	   468,	   463,	   457,	   452,	   445,	   439,	   433,	   426,	   419,	   411,	   404,	   396,	   388,	   379,	   371,
   362,	   353,	   344,	   334,	   325,	   315,	   305,	   295,	   284,	   274,	   263,	   252,	   241,	   230,	   219,	   207,
   196,	   184,	   172,	   161,	   149,	   137,	   124,	   112,	   100,	    88,	    75,	    63,	    50,	    38,	    25,	    13,
     0,	   -13,	   -25,	   -38,	   -50,	   -63,	   -75,	   -88,	  -100,	  -112,	  -124,	  -137,	  -149,	  -161,	  -172,	  -184,
  -196,	  -207,	  -219,	  -230,	  -241,	  -252,	  -263,	  -274,	  -284,	  -295,	  -305,	  -315,	  -325,	  -334,	  -344,	  -353,
  -362,	  -371,	  -379,	  -388,	  -396,	  -404,	  -411,	  -419,	  -426,	  -433,	  -439,	  -445,	  -452,	  -457,	  -463,	  -468,
  -473,	  -478,	  -482,	  -486,	  -490,	  -493,	  -497,	  -500,	  -502,	  -504,	  -506,	  -508,	  -510,	  -511,	  -511,	  -512,
  -512,	  -512,	  -511,	  -511,	  -510,	  -508,	  -506,	  -504,	  -502,	  -500,	  -497,	  -493,	  -490,	  -486,	  -482,	  -478,
  -473,	  -468,	  -463,	  -457,	  -452,	  -445,	  -439,	  -433,	  -426,	  -419,	  -411,	  -404,	  -396,	  -388,	  -379,	  -371,
  -362,	  -353,	  -344,	  -334,	  -325,	  -315,	  -305,	  -295,	  -284,	  -274,	  -263,	  -252,	  -241,	  -230,	  -219,	  -207,
  -196,	  -184,	  -172,	  -161,	  -149,	  -137,	  -124,	  -112,	  -100,	   -88,	   -75,	   -63,	   -50,	   -38,	   -25,	   -13,
};
void main(void)
{

   InitPIvalue();
   InitSysCtrl();
   DINT;
   InitPieCtrl();
   IER = 0x0000;
   IFR = 0x0000;
   InitPieVectTable();


   //Configure Epwm1 , 2
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
   InitEPwm1Gpio();//Configure GPIO0 as EPWM1A  ,   GPIO1 as EPWM1B
   InitEPwm2Gpio();//Configure GPIO2 as EPWM1A  ,   GPIO3 as EPWM1B
   InitEPwm1Example();
   InitEPwm2Example();
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;
   SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;

   // configure ADCSCOC0
   EALLOW;
   AdcRegs.ADCCTL1.bit.ADCENABLE = 0;  // 不致能ADC (在initadc中已經致能)
   AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1; //ADCINT1 trips after AdcResults latch

   AdcRegs.INTSEL1N2.bit.INT1E = 0;	//disabled ADCINT1
   AdcRegs.INTSEL1N2.bit.INT1CONT = 0;	//Disable ADCINT1 Continuous mode
  // AdcRegs.INTSEL1N2.bit.INT1SEL = 0;	//setup EOC0 to trigger ADCINT1 to fire

   AdcRegs.ADCSOC0CTL.bit.CHSEL = 1;	//set SOC0 channel select to ADCINA1
   AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 1;	//set SOC0 start trigger on CPUtimer
   AdcRegs.ADCSOC0CTL.bit.ACQPS = 6;//set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
   	//AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
   EDIS;


   // Configure Gpio control
   EALLOW;
   GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;         // GPIO ADC開關
   GpioCtrlRegs.GPADIR.bit.GPIO4 = 0;          // input
   GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;

   GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;         // GPIO
   GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;          // OUTput
   GpioDataRegs.GPACLEAR.bit.GPIO6 = 1; 		// 當ADC ON/OFF 標示燈

   GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;			// GPIO ePWM開關
   GpioCtrlRegs.GPADIR.bit.GPIO8 = 0;
   //GpioCtrlRegs.GPAQSEL1.bit.GPIO3 = 0;        // XINT2 Synch to SYSCLKOUT only
  // GpioIntRegs.GPIOXINT2SEL.bit.GPIOSEL = 3;   // XINT2 is GPIO3
   GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;  		//

   GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;         // GPIO
   GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;          // OUTput
   GpioDataRegs.GPACLEAR.bit.GPIO10 = 1; 		// 當 ePWM ON/OFF 標示燈

   GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;         // GPIO
   GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;          // OUTput 閃爍用
   GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;
   EDIS;


   // Configure Cputimer
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.TINT0 = &cpu_timer0_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers
   InitCpuTimers();   // For this example, only initialize the Cpu Timer
   ConfigCpuTimer(&CpuTimer0, 1, 3906);
   CpuTimer0Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0

   IER |= M_INT1;
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;


   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

   for(;;);
}


void InitPIvalue(){
	for( i=0;i<255;i++) ADCValue[i] = 0;

}

void InitEPwm1Example() {
	SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;

	// Setup TBCLK
	EPwm1Regs.TBPRD = 1500;           // Set timer period 801 TBCLKs
	// EPwm1Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
	EPwm1Regs.TBCTR = 0x0000;                      // Clear counter

	// Set Compare values
	EPwm1Regs.CMPA.half.CMPA = 1500;     // Set compare A value
	EPwm1Regs.CMPB = 1500;               // Set Compare B value

	// Setup counter mode
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
	EPwm1Regs.TBCTL.bit.PHSEN = 0x00;        // Enable phase loading
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;       // Clock ratio to SYSCLKOUT
	EPwm1Regs.TBCTL.bit.CLKDIV = 0;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = 0x01;

	// Setup shadowing
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// Set actions
	EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;         // Set PWM1A on event A, up count
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;   // Clear PWM1A on event A, down count

	EPwm1Regs.DBCTL.bit.IN_MODE = 0x00;
	EPwm1Regs.DBCTL.bit.POLSEL = 0x01;
	EPwm1Regs.DBCTL.bit.OUT_MODE = 0x01;
	EPwm1Regs.DBCTL.bit.HALFCYCLE = 0x00;
	EPwm1Regs.DBRED = 0;
	EPwm1Regs.DBFED = 0;

}


void InitEPwm2Example() {
	SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;

	// Setup TBCLK
	EPwm2Regs.TBPRD = 1500;           // Set timer period 801 TBCLKs
	//EPwm2Regs.TBPHS.half.TBPHS = 0;           // Phase is 0
	EPwm2Regs.TBCTR = 0x0000;                      // Clear counter

	// Set Compare values
	EPwm2Regs.CMPA.half.CMPA = 1050;     // Set compare A value
	EPwm2Regs.CMPB = 1050;               // Set Compare B value

	// Setup counter mode
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
	EPwm2Regs.TBCTL.bit.PHSEN = 0x01;        // Enable phase loading
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;       // Clock ratio to SYSCLKOUT
	EPwm2Regs.TBCTL.bit.CLKDIV = 0;
	EPwm2Regs.TBCTL.bit.PHSDIR = TB_DOWN;
	EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	// EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;

	// Setup shadowing
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// Set actions
	EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Set PWM2A on event A, up count             // Set PWM1A on event A, up count
	EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;

	EPwm2Regs.DBCTL.bit.IN_MODE = 0x00;
	EPwm2Regs.DBCTL.bit.POLSEL = 0x01;
	EPwm2Regs.DBCTL.bit.OUT_MODE = 0x01;
	EPwm1Regs.DBCTL.bit.HALFCYCLE = 0x00;
	EPwm2Regs.DBRED = 0;
	EPwm2Regs.DBFED = 0;

}

__interrupt void cpu_timer0_isr(void)
{
   DELAY_US(10);
   ADCValue[ADCCount] = AdcResult.ADCRESULT0;
   // Configure protect
   if(ADCValue[ADCCount] >= 3583){
	   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	   EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	   EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	   EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	   while(1);
   }

   // Detect the Epwm Switch
   if(GpioDataRegs.GPADAT.bit.GPIO8 == 0){
		EPwm1Regs.CMPA.half.CMPA = 1500 ;
		EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
		EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	   	EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	    for( i=0;i<256;i++) ADCValue[i] =0 ;
	   	z1=0; z2=0;
	   	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
   }else{
	   	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;
	   	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	   	EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
	   	EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	   	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	   	GpioDataRegs.GPASET.bit.GPIO10 = 1;
   }

   // Detect the ADC Switch
   if(GpioDataRegs.GPADAT.bit.GPIO4 ==0){
	   	EPwm1Regs.CMPA.half.CMPA = 1500 ;
	    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
	    for( i=0;i<256;i++) ADCValue[i] =0 ;
	    z1=0; z2=0;

   }else if(GpioDataRegs.GPADAT.bit.GPIO4 ==1 && GpioDataRegs.GPADAT.bit.GPIO8 == 1 ){

	    v1 = 3174;
	    v2 = 0;
	    for(i=0;i<256;i++) v2 += ADCValue[i] ;
	   	kP = 0.05;
	   	kI = 0.0005;
	   	verr = v1 - v2;
	   	vk2 = verr + z1;
	   	z1 = verr;
	   	vko2 =(int)( vk2 * kI );
	   	if (vko2 >= 1500)
	   	{vko2 = 1500;}
	   	else if (vko2 <= -1300) /////////////////////// 變頻要改
	   	{vko2 = -1300;}
	    vou2 = vko2 + z2;
	   	z2 = vou2;
	   	vk1 = verr;
	   	vko1 =(int) (vk1 * kP) ;
	   	vout =(int)( vou2 + vko1);
	   	if (vout >= 750) {					///////// 變頻要改
	   	vout = 750;
	   	} else if (vout <= 0) {
	   	vout = 0;
	   	}
	   	

	   	EPwm1Regs.CMPA.half.CMPA = 1500 - (750 + ( vout*( sintable1[ADCCount] ) )>>9 ); 
	   	EPwm1Regs.CMPA.half.CMPA = 1500 - (750 + ( ( vout*( sintable1[ADCCount] ) )>>9 )*(-1));
	 
	   	GpioDataRegs.GPASET.bit.GPIO6 = 1;
   }else{
		GpioDataRegs.GPASET.bit.GPIO6 = 1;
   }

   if( ADCCount != 255) ADCCount++;
   else ADCCount = 0;

   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


//===========================================================================
// No more.
//===========================================================================



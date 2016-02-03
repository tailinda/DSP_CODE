
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

// Configure which ePWM timer interrupts are enabled at the PIE level:
// 1 = enabled,  0 = disabled
typedef struct
{
   volatile struct EPWM_REGS *EPwmRegHandle;
   Uint16 EPwm_CMPA_Direction;
   Uint16 EPwm_CMPB_Direction;
   Uint16 EPwmTimerIntCount;
   Uint16 EPwmMaxCMPA;
   Uint16 EPwmMinCMPA;
   Uint16 EPwmMaxCMPB;
   Uint16 EPwmMinCMPB;
}EPWM_INFO;

// Prototype statements for functions found within this file.
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);
interrupt void epwm1_isr(void);
interrupt void epwm2_isr(void);
interrupt void epwm3_isr(void);
void update_compare(EPWM_INFO*);

// Global variables used in this example
EPWM_INFO epwm1_info;
EPWM_INFO epwm2_info;
EPWM_INFO epwm3_info;

// Configure the period for each timer
#define EPWM1_TIMER_TBPRD  2000  // Period register
#define EPWM1_MAX_CMPA     1950
#define EPWM1_MIN_CMPA       50
#define EPWM1_MAX_CMPB     1950
#define EPWM1_MIN_CMPB       50

#define EPWM2_TIMER_TBPRD  2000  // Period register
#define EPWM2_MAX_CMPA     1950
#define EPWM2_MIN_CMPA       50
#define EPWM2_MAX_CMPB     1950
#define EPWM2_MIN_CMPB       50

#define EPWM3_TIMER_TBPRD  2000  // Period register
#define EPWM3_MAX_CMPA      950
#define EPWM3_MIN_CMPA       50
#define EPWM3_MAX_CMPB     1950
#define EPWM3_MIN_CMPB     1050

// To keep track of which way the compare value is moving
#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0

void main(void)
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2803x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initalize GPIO:
// This example function is found in the DSP2803x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example

// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
// These functions are in the DSP2803x_EPwm.c file
   InitEPwm1Gpio();
   InitEPwm2Gpio();


// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2803x_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

   GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;  // GPIO0 = PWM1A
   GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;  // GPIO1 = PWM1B
   GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;  // GPIO2 = PWM2A
   GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;  // GPIO3 = PWM2B
   GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;   //GPIO0 = output
   GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;	//GPIO0 = output
   GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;   //GPIO0 = output
   GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;   //GPIO0 = output


// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2803x_DefaultIsr.c.
// This function is found in DSP2803x_PieVect.c.
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
   EALLOW;  // This is needed to write to EALLOW protected registers
  // PieVectTable.EPWM1_INT = &epwm1_isr;
  // PieVectTable.EPWM2_INT = &epwm2_isr;

   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP2803x_InitPeripherals.c
// InitPeripherals();  // Not required for this example

// For this example, only initialize the ePWM

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
   EDIS;

   InitEPwm1Example();
   InitEPwm2Example();


   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   EDIS;

// Step 5. User specific code, enable interrupts:

// Enable CPU INT3 which is connected to EPWM1-3 INT:
   IER |= M_INT3;


//   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
//   PieCtrlRegs.PIEIER3.bit.INTx2 = 1;

   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

// Step 6. IDLE loop. Just sit and loop forever (optional):
   for(;;)
   {
       asm("          NOP");
   }

}

void InitEPwm1Example()
{
   SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;

   // Setup TBCLK
   EPwm1Regs.TBPRD = 1500;           // Set timer period 801 TBCLKs
   EPwm1Regs.TBCTL.bit.PHSEN = 0x00 ;        //  關閉角度同步(所以下一行不重要) 
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                      // Clear counter

   // Set Compare values
   EPwm1Regs.CMPA.half.CMPA = 1050;     // Set compare A value
   EPwm1Regs.CMPB = 1050;               // Set Compare B value

   // Setup counter mode
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
   
   // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;       
   EPwm1Regs.TBCTL.bit.CLKDIV = 0; 
   // !!!!!!!!!!!!!!!! 相移重點所在 !!!!!!!!!!!!!!!!! 
   EPwm1Regs.TBCTL.bit.SYNCOSEL = 0x01; // CTR = 0時 , 輸出 EPWMxSYNCO 訊號以控制 EPWM2 !!  

   // Setup shadowing (使用陰影模式)
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set actions
   EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on event A, up count
   EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;           // Clear PWM1A on event A, down count

   // 設置 EPWM1A 與 EPWM1B 之關係 (互補就是這裡做出來的) 參考 ePWM P.53   
   EPwm1Regs.DBCTL.bit.IN_MODE = 0 ;
   EPwm1Regs.DBCTL.bit.POLSEL  = 2 ;
   EPwm1Regs.DBCTL.bit.OUT_MODE= 3 ;
   EPwm1Regs.DBRED = 300;
   EPwm1Regs.DBFED = 300;
  
  /*
   epwm1_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA &
   epwm1_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // decreasing CMPB
   epwm1_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
   epwm1_info.EPwmRegHandle = &EPwm1Regs;          // Set the pointer moduleto the ePWM
   epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA;        // Setup min/max CMPA/CMPB values
   epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
   epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
   epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;
 */
}

void InitEPwm2Example()
{
   SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;

   // Setup TBCLK
   EPwm2Regs.TBPRD = 1500;           // Set timer period 801 TBCLKs
   
   EPwm2Regs.TBCTR = 0x0000;                      // Clear counter

   // Set Compare values
   EPwm2Regs.CMPA.half.CMPA =1050;     // Set compare A value
   EPwm2Regs.CMPB = 1050;               // Set Compare B value

   // Setup counter mode
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
   
   //  !!!!!!!!!!!!!使其受控於  ePWM1 , 當 ePWM1 輸出  EPWM1SYNCO 訊號 , 則 EPWM2的 TBCTL值 變成 750 並且往下計數 
   EPwm2Regs.TBCTL.bit.PHSEN = 0x01 ;    //Enable phase loading      
   EPwm2Regs.TBPHS.half.TBPHS = 750;     // Phase is 750
   EPwm2Regs.TBCTL.bit.PHSDIR = TB_DOWN; // Count down after the synchronization event 
   
   // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;    
   EPwm2Regs.TBCTL.bit.CLKDIV = 0;
   EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
  // EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;

   // Setup shadowing
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set actions
   EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on event A, up count             // Set PWM1A on event A, up count
   EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;

   EPwm2Regs.DBCTL.bit.IN_MODE = 0 ;
   EPwm2Regs.DBCTL.bit.POLSEL  = 2 ;
   EPwm2Regs.DBCTL.bit.OUT_MODE= 3 ;
   EPwm2Regs.DBRED = 300;
   EPwm2Regs.DBFED = 300;
   
   /*
   epwm2_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA &
   epwm2_info.EPwm_CMPB_Direction = EPWM_CMP_UP;   // increasing CMPB
   epwm2_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
   epwm2_info.EPwmRegHandle = &EPwm2Regs;          // Set the pointer to the ePWM module
   epwm2_info.EPwmMaxCMPA = EPWM2_MAX_CMPA;        // Setup min/max CMPA/CMPB values
   epwm2_info.EPwmMinCMPA = EPWM2_MIN_CMPA;
   epwm2_info.EPwmMaxCMPB = EPWM2_MAX_CMPB;
   epwm2_info.EPwmMinCMPB = EPWM2_MIN_CMPB;
  */

}

/*
void update_compare(EPWM_INFO *epwm_info)
{


   // Every 10'th interrupt, change the CMPA/CMPB values
   if(epwm_info->EPwmTimerIntCount == 10)
   {
       epwm_info->EPwmTimerIntCount = 0;

       // If we were increasing CMPA, check to see if
       // we reached the max value.  If not, increase CMPA
       // else, change directions and decrease CMPA
	   if(epwm_info->EPwm_CMPA_Direction == EPWM_CMP_UP)
	   {
	       if(epwm_info->EPwmRegHandle->CMPA.half.CMPA < epwm_info->EPwmMaxCMPA)
	       {
	          epwm_info->EPwmRegHandle->CMPA.half.CMPA++;
	       }
	       else
	       {
	          epwm_info->EPwm_CMPA_Direction = EPWM_CMP_DOWN;
              epwm_info->EPwmRegHandle->CMPA.half.CMPA--;
	       }
	   }

	   // If we were decreasing CMPA, check to see if
       // we reached the min value.  If not, decrease CMPA
       // else, change directions and increase CMPA
	   else
	   {
	       if(epwm_info->EPwmRegHandle->CMPA.half.CMPA == epwm_info->EPwmMinCMPA)
	       {
	          epwm_info->EPwm_CMPA_Direction = EPWM_CMP_UP;
	          epwm_info->EPwmRegHandle->CMPA.half.CMPA++;
	       }
	       else
	       {
	          epwm_info->EPwmRegHandle->CMPA.half.CMPA--;
	       }
	   }

	   // If we were increasing CMPB, check to see if
       // we reached the max value.  If not, increase CMPB
       // else, change directions and decrease CMPB
	   if(epwm_info->EPwm_CMPB_Direction == EPWM_CMP_UP)
	   {
	       if(epwm_info->EPwmRegHandle->CMPB < epwm_info->EPwmMaxCMPB)
	       {
	          epwm_info->EPwmRegHandle->CMPB++;
	       }
	       else
	       {
	          epwm_info->EPwm_CMPB_Direction = EPWM_CMP_DOWN;
	          epwm_info->EPwmRegHandle->CMPB--;
	       }
	   }

	   // If we were decreasing CMPB, check to see if
       // we reached the min value.  If not, decrease CMPB
       // else, change directions and increase CMPB

	   else
	   {
	       if(epwm_info->EPwmRegHandle->CMPB == epwm_info->EPwmMinCMPB)
	       {
	          epwm_info->EPwm_CMPB_Direction = EPWM_CMP_UP;
	          epwm_info->EPwmRegHandle->CMPB++;
	       }
	       else
	       {
	          epwm_info->EPwmRegHandle->CMPB--;
	       }
	   }
   }
   else
   {
      epwm_info->EPwmTimerIntCount++;
   }

   return;
}
*/

//===========================================================================
// No more.
//===========================================================================



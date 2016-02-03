/* 
利用ADC抓取60HZ弦波信號 , 使用cpuTimer觸發 , 
並且做一開關控制 ADC是否啟動 , 若啟動 , 則燈亮 , 否則燈暗 
*/






#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

// Prototype statements for functions found within this file.
__interrupt void adc_isr(void);
__interrupt void xint_isr(void);
__interrupt void cputimer_isr(void);
void Adc_Config(void);

// Global variables used in this example:
Uint16 LoopCount;
Uint16 ConversionCount;
Uint16 Voltage1[255];
int xintcount = 0;

main() {
	// 初始化system
	InitSysCtrl();
	DINT;
	// 初始化PIE
	InitPieCtrl();
	// 初始化ADC
	InitAdc();

	AdcOffsetSelfCal();
    // 初始化中斷向量表 
	InitPieVectTable();

	IER = 0x0000;
	IFR = 0x0000;
	
	// 設定中斷向量表位置
	EALLOW;
	PieVectTable.XINT1 = &xint_isr;
	PieVectTable.ADCINT1 = &adc_isr;
	PieVectTable.TINT0 = &cputimer_isr;
	EDIS;

	// 設置外部中斷腳位及燈號腳位
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;         // GPIO 功能為一般I/O
	GpioCtrlRegs.GPADIR.bit.GPIO0 = 0;          // GPIO0 = input
	GpioCtrlRegs.GPAQSEL1.bit.GPIO0 = 0;        // XINT1 Synch to SYSCLKOUT only
	GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 0;   // XINT1 is GPIO0

	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;			// GPIO 功能為一般I/O 
	GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;          // GPIO1 = output
	GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;		// GPIO1 輸出為0 
	EDIS;

	IER |= M_INT1;  // 開啟GROUP1中斷
	EINT;
	ERTM;
	EALLOW;
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;  // 致能 PIE(非必要)
	PieCtrlRegs.PIEIER1.bit.INTx1 = 0;  // 致能 ADCINT1 
	PieCtrlRegs.PIEIER1.bit.INTx4 = 1;  // 致能 XINT 
//	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  // 致能 CPUTIMER !!!!!!!! 不能同時開啟  , 否則打架  !!!!!!!!! (本題不需要)
	XIntruptRegs.XINT1CR.bit.POLARITY = 3; // interrupt occur on both falling and rising edge
	XIntruptRegs.XINT1CR.bit.ENABLE = 1; // Enable Xint1
	EDIS;

	LoopCount = 0;
	ConversionCount = 0;

// Configure ADC
// Note: Channel ADCINA4  will be double sampled to workaround the ADC 1st sample issue for rev0 silicon errata

	EALLOW;
	//AdcRegs.ADCCTL1.bit.ADCENABLE = 1;  // 致能ADC (在initadc中已經致能)
	AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1; // ADCINT1 trips after AdcResults latch

	AdcRegs.INTSEL1N2.bit.INT1E = 1;	// Enabled ADCINT1
	AdcRegs.INTSEL1N2.bit.INT1CONT = 0;	// Disable ADCINT1 Continuous mode
	AdcRegs.INTSEL1N2.bit.INT1SEL = 0;	// setup EOC0 to trigger ADCINT1 to fire

	AdcRegs.ADCSOC0CTL.bit.CHSEL = 1;	// set SOC0 channel select to ADCINA1
	AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 1;	// set SOC0 start trigger on CPUtimer
	AdcRegs.ADCSOC0CTL.bit.ACQPS = 6;// set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	//AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; (非必要)
	EDIS;

	InitCpuTimers();  // 初始化 cpuTimer 
	ConfigCpuTimer(&CpuTimer0, 1, 3906); // !!!!!!!!!!!!!!!!!!!!!!!!!!必須先配置!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	EALLOW;
	CpuTimer0Regs.TCR.all = 0x4000;   // !!!!!!!!!!!!!!!!!!!!!!!!!!!再致能CPUTIMER0!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
	EDIS;


	for (;;) {
		LoopCount++;
	}

}

__interrupt void xint_isr(void) {
	if (xintcount == 0) {  // xintcount 用來判斷目前燈號暗或亮 
		xintcount++;
	} else {
		xintcount = 0;
	}
	if (xintcount == 0) {   
		PieCtrlRegs.PIEIER1.bit.INTx1 = 0; // Disable ADC convertion 
		GpioDataRegs.GPACLEAR.bit.GPIO1 = 1; // 關閉燈號 
	} else {

		PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable ADC convertion 
		GpioDataRegs.GPASET.bit.GPIO1 = 1; // 開啟燈號 
	}

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE   !!!!!!!一定要打 否則只會中斷一次 !!!!!! 
}

__interrupt void cputimer_isr(void) {
/*	Voltage1[ConversionCount] = AdcResult.ADCRESULT0;

	//If 20 conversions have been logged, start over
	if (ConversionCount == 255) {
		ConversionCount = 0;
	} else

		AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //Clear ADCINT1 flag reinitialize for next SOC
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

	return;
*/
}

interrupt void adc_isr(void) {

	Voltage1[ConversionCount] = AdcResult.ADCRESULT0;

// If 20 conversions have been logged, start over
	if (ConversionCount == 255) {
		ConversionCount = 0;
	} else
		ConversionCount++;

	AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //Clear ADCINT1 flag reinitialize for next SOC
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

	return;
}



/////////////////////////////////結論////////////////////////////////////////

// cpuTimer 中斷在本題不可與 Xint 中斷同時開啟 , 因為其頻率太快 , 同時開啟 Xint 將會無法進中斷 

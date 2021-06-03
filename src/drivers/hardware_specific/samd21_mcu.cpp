


#include "./samd_mcu.h"
#include "../../common/hardware_specific/samd_mcu.h"


#ifdef _SAMD21_



#ifndef TCC3_CH0
#define TCC3_CH0 NOT_ON_TIMER
#endif
#ifndef TCC3_CH1
#define TCC3_CH1 NOT_ON_TIMER
#endif
#ifndef TCC3_CH2
#define TCC3_CH2 NOT_ON_TIMER
#endif
#ifndef TCC3_CH3
#define TCC3_CH3 NOT_ON_TIMER
#endif
#ifndef TCC3_CH4
#define TCC3_CH4 NOT_ON_TIMER
#endif
#ifndef TCC3_CH5
#define TCC3_CH5 NOT_ON_TIMER
#endif
#ifndef TCC3_CH6
#define TCC3_CH6 NOT_ON_TIMER
#endif
#ifndef TCC3_CH7
#define TCC3_CH7 NOT_ON_TIMER
#endif
#ifndef TC6_CH0
#define TC6_CH0 NOT_ON_TIMER
#endif
#ifndef TC6_CH1
#define TC6_CH1 NOT_ON_TIMER
#endif
#ifndef TC7_CH0
#define TC7_CH0 NOT_ON_TIMER
#endif
#ifndef TC7_CH1
#define TC7_CH1 NOT_ON_TIMER
#endif



#define NUM_WO_ASSOCIATIONS 48

/*
 * For SAM D21 A/B/C/D Variant Devices and SAM DA1 A/B Variant Devices
 * Good for SAMD2xE, SAMD2xG and SAMD2xJ devices. Other SAMD21s currently not supported in arduino anyway?
 *
 * Note: only the pins which have timers associated are listed in this table.
 * You can use the values from g_APinDescription.ulPort and g_APinDescription.ulPin to find the correct row in the table.
 *
 * See Microchip Technology datasheet DS40001882F-page 30
 */
struct wo_association WO_associations[] = {

		{ PORTA,  0, TCC2_CH0, 		0, NOT_ON_TIMER, 	0},
		{ PORTA,  1, TCC2_CH1, 		1, NOT_ON_TIMER, 	0},
		{ PORTA,  2, NOT_ON_TIMER, 	0, TCC3_CH0, 		0},
		{ PORTA,  3, NOT_ON_TIMER,	0, TCC3_CH1, 		1},
		// PB04, PB05, PB06, PB07 - no timers
		{ PORTB,  8, TC4_CH0, 		0, TCC3_CH6, 		6},
		{ PORTB,  9, TC4_CH1, 		1, TCC3_CH7, 		7},
		{ PORTA,  4, TCC0_CH0, 		0, TCC3_CH2, 		2},
		{ PORTA,  5, TCC0_CH1, 		1, TCC3_CH3, 		3},
		{ PORTA,  6, TCC1_CH0, 		0, TCC3_CH4, 		4},
		{ PORTA,  7, TCC1_CH1, 		1, TCC3_CH5, 		5},
		{ PORTA,  8, TCC0_CH0, 		0, TCC1_CH2, 		2},
		{ PORTA,  9, TCC0_CH1, 		1, TCC1_CH3, 		3},
		{ PORTA, 10, TCC1_CH0, 		0, TCC0_CH2, 		2},
		{ PORTA, 11, TCC1_CH1, 		1, TCC0_CH3, 		3},
		{ PORTB, 10, TC5_CH0, 		0, TCC0_CH4, 		4},
		{ PORTB, 11, TC5_CH1, 		1, TCC0_CH5, 		5},
		{ PORTB, 12, TC4_CH0, 		0, TCC0_CH6, 		6},
		{ PORTB, 13, TC4_CH1, 		1, TCC0_CH7, 		7},
		{ PORTB, 14, TC5_CH0, 		0, NOT_ON_TIMER,	0},
		{ PORTB, 15, TC5_CH1, 		1, NOT_ON_TIMER, 	0},
		{ PORTA, 12, TCC2_CH0, 		0, TCC0_CH6, 		6},
		{ PORTA, 13, TCC2_CH1, 		1, TCC0_CH7, 		7},
		{ PORTA, 14, TC3_CH0, 		0, TCC0_CH4, 		4},
		{ PORTA, 15, TC3_CH1, 		1, TCC0_CH5, 		5},
		{ PORTA, 16, TCC2_CH0, 		0, TCC0_CH6, 		6},
		{ PORTA, 17, TCC2_CH1, 		1, TCC0_CH7, 		7},
		{ PORTA, 18, TC3_CH0, 		0, TCC0_CH2, 		2},
		{ PORTA, 19, TC3_CH1, 		1, TCC0_CH3, 		3},
		{ PORTB, 16, TC6_CH0, 		0, TCC0_CH4, 		4},
		{ PORTB, 17, TC6_CH1, 		1, TCC0_CH5, 		5},
		{ PORTA, 20, TC7_CH0, 		0, TCC0_CH6, 		6},
		{ PORTA, 21, TC7_CH1, 		1, TCC0_CH7, 		7},
		{ PORTA, 22, TC4_CH0, 		0, TCC0_CH4, 		4},
		{ PORTA, 23, TC4_CH1, 		1, TCC0_CH5, 		5},
		{ PORTA, 24, TC5_CH0, 		0, TCC1_CH2, 		2},
		{ PORTA, 25, TC5_CH1, 		1, TCC1_CH3, 		3},
		{ PORTB, 22, TC7_CH0, 		0, TCC3_CH0, 		0},
		{ PORTB, 23, TC7_CH1, 		1, TCC3_CH1, 		1},
		{ PORTA, 27, NOT_ON_TIMER, 	0, TCC3_CH6, 		6},
		{ PORTA, 28, NOT_ON_TIMER, 	0, TCC3_CH7, 		7},
		{ PORTA, 30, TCC1_CH0, 		0, TCC3_CH4, 		4},
		{ PORTA, 31, TCC1_CH1, 		1, TCC3_CH5, 		5},
		{ PORTB, 30, TCC0_CH0, 		0, TCC1_CH2, 		2},
		{ PORTB, 31, TCC0_CH1, 		1, TCC1_CH3, 		3},
		{ PORTB,  0, TC7_CH0, 		0, NOT_ON_TIMER, 	0},
		{ PORTB,  1, TC7_CH1, 		1, NOT_ON_TIMER, 	0},
		{ PORTB,  2, TC6_CH0, 		0, TCC3_CH2, 		2},
		{ PORTB,  3, TC6_CH1, 		1, TCC3_CH3, 		3}
};
wo_association ASSOCIATION_NOT_FOUND = { NOT_A_PORT, 0, NOT_ON_TIMER, 0, NOT_ON_TIMER, 0};



struct wo_association& getWOAssociation(EPortType port, uint32_t pin) {
	for (int i=0;i<NUM_WO_ASSOCIATIONS;i++) {
		if (WO_associations[i].port==port && WO_associations[i].pin==pin)
			return WO_associations[i];
	}
	return ASSOCIATION_NOT_FOUND;
};



EPioType getPeripheralOfPermutation(int permutation, int pin_position) {
	return ((permutation>>pin_position)&0x01)==0x1?PIO_TIMER_ALT:PIO_TIMER;
}





void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);       // Wait for synchronization of registers between the clock domains
}



/**
 * Configure Clock 4 - we want all simplefoc PWMs to use the same clock. This ensures that
 * any compatible pin combination can be used without having to worry about configuring different
 * clocks.
 */
void configureSAMDClock() {

	// TODO investigate using the FDPLL96M clock to get 96MHz timer clocks... this
	//      would enable 48KHz PWM clocks, and setting the frequency between 24Khz with resolution 2000, to 48KHz with resolution 1000

	if (!SAMDClockConfigured) {
		SAMDClockConfigured = true;						// mark clock as configured
		for (int i=0;i<TCC_INST_NUM;i++)				// mark all the TCCs as not yet configured
			tccConfigured[i] = false;
		REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor N=1: 48MHz/1=48MHz
						GCLK_GENDIV_ID(4);            	// Select Generic Clock (GCLK) 4
		while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

		REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
						 GCLK_GENCTRL_GENEN |         	// Enable GCLK4
						 GCLK_GENCTRL_SRC_DFLL48M |   	// Set the 48MHz clock source
						 GCLK_GENCTRL_ID(4);          	// Select GCLK4
		while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

#ifdef SIMPLEFOC_SAMD_DEBUG
		SIMPLEFOC_SAMD_DEBUG_SERIAL.println("Configured clock...");
#endif
	}
}




/**
 * Configure a TCC unit
 * pwm_frequency is fixed at 24kHz for now. We could go slower, but going
 * faster won't be possible without sacrificing resolution.
 */
void configureTCC(tccConfiguration& tccConfig, long pwm_frequency, bool negate, float hw6pwm, bool enableOVFEO) {
	// TODO for the moment we ignore the frequency...
	if (!tccConfigured[tccConfig.tcc.tccn]) {
		uint32_t GCLK_CLKCTRL_ID_ofthistcc = -1;
		switch (tccConfig.tcc.tccn>>1) {
		case 0: GCLK_CLKCTRL_ID_ofthistcc = GCLK_CLKCTRL_ID(GCM_TCC0_TCC1); break; //GCLK_CLKCTRL_ID_TCC0_TCC1;
		case 1: GCLK_CLKCTRL_ID_ofthistcc = GCLK_CLKCTRL_ID(GCM_TCC2_TC3); break;  //GCLK_CLKCTRL_ID_TCC2_TC3;
		case 2: GCLK_CLKCTRL_ID_ofthistcc = GCLK_CLKCTRL_ID(GCM_TC4_TC5); break;   //GCLK_CLKCTRL_ID_TC4_TC5;
		case 3: GCLK_CLKCTRL_ID_ofthistcc = GCLK_CLKCTRL_ID(GCM_TC6_TC7); break;
		default: return;
		}

		// Feed GCLK4 to TCC
		REG_GCLK_CLKCTRL = (uint16_t)  GCLK_CLKCTRL_CLKEN |         // Enable GCLK4
									   GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
									   GCLK_CLKCTRL_ID_ofthistcc;   // Feed GCLK4 to tcc
		while (GCLK->STATUS.bit.SYNCBUSY);              			// Wait for synchronization


		static bool isOVFEOConfigured = false;
		if(enableOVFEO)
		{
			if(isOVFEOConfigured)
			{
				debugPrintln("Fatal error! OVFEO already configured, you can only configure it once.");
			}
			else
			{
				
				debugPrintf("Configuring EVSYS for TCC channel %d\n\r", tccConfig.tcc.tccn);

				uint8_t evgen_tcc = 0;
				switch (tccConfig.tcc.tccn){
				case 0: evgen_tcc = EVSYS_ID_GEN_TCC0_OVF; break;
				case 1: evgen_tcc = EVSYS_ID_GEN_TCC1_OVF; break;
				case 2: evgen_tcc = EVSYS_ID_GEN_TCC2_OVF; break;
				// case 3: channel.bit.EVGEN = EVSYS_ID_GEN_TC3_OVF; break; // TODO: we need the TC equivalent of TCC_WAVEB_WAVEGENB_DSBOTTOM 
				// case 4: channel.bit.EVGEN = EVSYS_ID_GEN_TC4_OVF; break; // TODO: we need the TC equivalent of TCC_WAVEB_WAVEGENB_DSBOTTOM
				// case 5: channel.bit.EVGEN = EVSYS_ID_GEN_TC5_OVF; break; // TODO: we need the TC equivalent of TCC_WAVEB_WAVEGENB_DSBOTTOM
				default: 
					debugPrintf("Error in configureTCC() [EVSYS] Bad tccn number: %d\n\r", tccConfig.tcc.tccn);
					return;
				}

				EVSYS_USER_Type userADC;
				userADC.bit.CHANNEL = tccConfig.tcc.tccn + 1; /* use channel tccn p421: "Note that to select channel n, the value (n+1) must be written to the USER.CHANNEL bit group." */
				userADC.bit.USER = EVSYS_ID_USER_ADC_SYNC; /* ADC Start*/
				EVSYS->USER.reg = userADC.reg;
				// configuration of the event system so that it triggers an "EVSYS_ID_USER_ADC_SYNC" at each timer overflow (underflow)
				EVSYS_CHANNEL_Type channelTCC_ADC;
				channelTCC_ADC.bit.EVGEN = evgen_tcc;
				channelTCC_ADC.bit.EDGSEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT_Val;
				channelTCC_ADC.bit.PATH = EVSYS_CHANNEL_PATH_ASYNCHRONOUS_Val; //EVSYS_ID_USER_ADC_SYNC is asynchronous only
				channelTCC_ADC.bit.SWEVT = 0;   /* no software trigger */
				channelTCC_ADC.bit.CHANNEL = userADC.bit.CHANNEL - 1;
				EVSYS->CHANNEL.reg = channelTCC_ADC.reg;

				EVSYS_USER_Type userDMA3;
				userDMA3.bit.CHANNEL = tccConfig.tcc.tccn + 6 + 1; /* use channel tccn p421: "Note that to select channel n, the value (n+1) must be written to the USER.CHANNEL bit group." */
				userDMA3.bit.USER = EVSYS_ID_USER_DMAC_CH_3; /* DMA_Start*/
				EVSYS->USER.reg = userDMA3.reg;

				EVSYS_CHANNEL_Type channelTCC_DMA3;
				channelTCC_DMA3.bit.EVGEN = evgen_tcc;
				channelTCC_DMA3.bit.EDGSEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT_Val;
				channelTCC_DMA3.bit.PATH = EVSYS_CHANNEL_PATH_RESYNCHRONIZED_Val; //DMA and TCC use different clocks (I think?)
				channelTCC_DMA3.bit.SWEVT = 0;   /* no software trigger */
				channelTCC_DMA3.bit.CHANNEL = userDMA3.bit.CHANNEL - 1;
				EVSYS->CHANNEL.reg = channelTCC_DMA3.reg;
			}
		}

		tccConfigured[tccConfig.tcc.tccn] = true;


		if (tccConfig.tcc.tccn>=TCC_INST_NUM) {
			Tc* tc = (Tc*)GetTC(tccConfig.tcc.chaninfo);
			// disable
			tc->COUNT8.CTRLA.bit.ENABLE = 0;
			while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
			// unfortunately we need the 8-bit counter mode to use the PER register...
			tc->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 | TC_CTRLA_WAVEGEN_NPWM ;
			while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
			// meaning prescaler of 8, since TC-Unit has no up/down mode, and has resolution of 250 rather than 1000...
			tc->COUNT8.CTRLA.bit.PRESCALER = TC_CTRLA_PRESCALER_DIV8_Val ;
			while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
			// period is 250, period cannot be higher than 256!
			tc->COUNT8.PER.reg = SIMPLEFOC_SAMD_PWM_TC_RESOLUTION-1;
			while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
			// initial duty cycle is 0
			tc->COUNT8.CC[tccConfig.tcc.chan].reg = 0;
			while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
			// enable
			tc->COUNT8.CTRLA.bit.ENABLE = 1;
			while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
#ifdef SIMPLEFOC_SAMD_DEBUG
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print("Initialized TC ");
			SIMPLEFOC_SAMD_DEBUG_SERIAL.println(tccConfig.tcc.tccn);
#endif
		}
		else {
			Tcc* tcc = (Tcc*)GetTC(tccConfig.tcc.chaninfo);

			uint8_t invenMask = ~(1<<tccConfig.tcc.chan);	// negate (invert) the signal if needed
			uint8_t invenVal = negate?(1<<tccConfig.tcc.chan):0;
			tcc->DRVCTRL.vec.INVEN = (tcc->DRVCTRL.vec.INVEN&invenMask)|invenVal;
			syncTCC(tcc); // wait for sync

			// Set wave form configuration
			// We do phase-correct PWM (up-down counting) - so the TOP value represents exactly the middle of the PWM on-period for the high-side, 
			// and the BOTTOM value represents the middle of the PWM on-period for the low-side (which has inverted output).
			// Personally, I would just attach to the high side, then the code can probably remain the same for 3-PWM and 6-PWM.
			// So basically for low side sensing I think you always want to generate events when the counter reaches the bottom value (turns around). 
			// You do this by setting PWM mode “DSBOTTOM” 
			if(enableOVFEO)
				tcc->WAVE.reg |= TCC_WAVE_POL(0xF)|TCC_WAVEB_WAVEGENB_DSTOP; 
			else
				tcc->WAVE.reg |= TCC_WAVE_POL(0xF)|TCC_WAVEB_WAVEGENB_DSBOTH;   

			while ( tcc->SYNCBUSY.bit.WAVE == 1 ); // wait for sync

			if (hw6pwm>0.0) {
				tcc->WEXCTRL.vec.DTIEN |= (1<<tccConfig.tcc.chan);
				tcc->WEXCTRL.bit.DTLS = hw6pwm*(SIMPLEFOC_SAMD_PWM_RESOLUTION-1);
				tcc->WEXCTRL.bit.DTHS = hw6pwm*(SIMPLEFOC_SAMD_PWM_RESOLUTION-1);
				syncTCC(tcc); // wait for sync
			}

			tcc->PER.reg = SIMPLEFOC_SAMD_PWM_RESOLUTION - 1;                 // Set counter Top using the PER register
			while ( tcc->SYNCBUSY.bit.PER == 1 ); // wait for sync

			// set all channels to 0%
			uint8_t chanCount = (tccConfig.tcc.tccn==1||tccConfig.tcc.tccn==2)?2:4;
			for (int i=0;i<chanCount;i++) {
				tcc->CC[i].reg = 0;					// start off at 0% duty cycle
				uint32_t chanbit = 0x1<<(TCC_SYNCBUSY_CC0_Pos+i);
				while ( (tcc->SYNCBUSY.reg & chanbit) > 0 );
			}

			if(enableOVFEO) {
				tcc->EVCTRL.bit.OVFEO = 1;
				while ( tcc->SYNCBUSY.bit.ENABLE == 1 ); // wait for sync
			}
			
			// Enable TC
			tcc->CTRLA.reg |= TCC_CTRLA_ENABLE | TCC_CTRLA_PRESCALER_DIV1; //48Mhz/1=48Mhz/2(up/down)=24MHz/1024=24KHz
			while ( tcc->SYNCBUSY.bit.ENABLE == 1 ); // wait for sync
			
#ifdef SIMPLEFOC_SAMD_DEBUG
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print("    Initialized TCC ");
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print(tccConfig.tcc.tccn);
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print("-");
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print(tccConfig.tcc.chan);
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print("[");
			SIMPLEFOC_SAMD_DEBUG_SERIAL.print(tccConfig.wo);
			SIMPLEFOC_SAMD_DEBUG_SERIAL.println("]");
#endif
		}
	}
	else if (tccConfig.tcc.tccn<TCC_INST_NUM) {
		// set invert bit for TCC channels in SW 6-PWM...
		Tcc* tcc = (Tcc*)GetTC(tccConfig.tcc.chaninfo);

		tcc->CTRLA.bit.ENABLE = 0;
		while ( tcc->SYNCBUSY.bit.ENABLE == 1 );

		uint8_t invenMask = ~(1<<tccConfig.tcc.chan);	// negate (invert) the signal if needed
		uint8_t invenVal = negate?(1<<tccConfig.tcc.chan):0;
		tcc->DRVCTRL.vec.INVEN = (tcc->DRVCTRL.vec.INVEN&invenMask)|invenVal;
		syncTCC(tcc); // wait for sync

		if (hw6pwm>0.0) {
			tcc->WEXCTRL.vec.DTIEN |= (1<<tccConfig.tcc.chan);
			tcc->WEXCTRL.bit.DTLS = hw6pwm*(SIMPLEFOC_SAMD_PWM_RESOLUTION-1);
			tcc->WEXCTRL.bit.DTHS = hw6pwm*(SIMPLEFOC_SAMD_PWM_RESOLUTION-1);
			syncTCC(tcc); // wait for sync
		}

		tcc->CTRLA.bit.ENABLE = 1;
		while ( tcc->SYNCBUSY.bit.ENABLE == 1 );

#ifdef SIMPLEFOC_SAMD_DEBUG
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print("(Re-)Initialized TCC ");
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print(tccConfig.tcc.tccn);
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print("-");
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print(tccConfig.tcc.chan);
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print("[");
		SIMPLEFOC_SAMD_DEBUG_SERIAL.print(tccConfig.wo);
		SIMPLEFOC_SAMD_DEBUG_SERIAL.println("]");
#endif
	}


}

void EVSYS_Init(int chaninfo)
{



}



void writeSAMDDutyCycle(int chaninfo, float dc) {
	uint8_t tccn = GetTCNumber(chaninfo);
	uint8_t chan = GetTCChannelNumber(chaninfo);
	if (tccn<TCC_INST_NUM) {
		Tcc* tcc = (Tcc*)GetTC(chaninfo);
		// set via CC
//		tcc->CC[chan].reg = (uint32_t)((SIMPLEFOC_SAMD_PWM_RESOLUTION-1) * dc);
//		uint32_t chanbit = 0x1<<(TCC_SYNCBUSY_CC0_Pos+chan);
//		while ( (tcc->SYNCBUSY.reg & chanbit) > 0 );
		// set via CCB
		while ( (tcc->SYNCBUSY.vec.CC & (0x1<<chan)) > 0 );
		tcc->CCB[chan].reg = (uint32_t)((SIMPLEFOC_SAMD_PWM_RESOLUTION-1) * dc);
//		while ( (tcc->SYNCBUSY.vec.CCB & (0x1<<chan)) > 0 );
//		tcc->STATUS.vec.CCBV |= (0x1<<chan);
//		while ( tcc->SYNCBUSY.bit.STATUS > 0 );
//		tcc->CTRLBSET.reg |= TCC_CTRLBSET_CMD(TCC_CTRLBSET_CMD_UPDATE_Val);
//		while ( tcc->SYNCBUSY.bit.CTRLB > 0 );
	}
	else {
		Tc* tc = (Tc*)GetTC(chaninfo);
		tc->COUNT8.CC[chan].reg = (uint8_t)((SIMPLEFOC_SAMD_PWM_TC_RESOLUTION-1) * dc);
		while ( tc->COUNT8.STATUS.bit.SYNCBUSY == 1 );
	}
}




#endif

/*
 * freqDma.h
 *
 *  Created on: Jul 2, 2014
 *      Author: Marco.HenryGin
 */

#ifndef FREQDMA_H_
#define FREQDMA_H_

/*************************************************************************
  *   $INCLUDES
*************************************************************************/
#include "PE_types.h"

/*************************************************************************
  *   $DEFINES
*************************************************************************/
#define NBR_FREQ_CHANNELS				4		/* It will be 4 */
/*!
 * The Size of the DMA table depends on the maximumm frequency and the 
 * sampling rate at which we used data (TCAP isr). In our application the 
 * TCAP isr is called every 28mS @1.125MHz and the maximum expected frequency 
 * is 1500Hz. We need at least 43.7 samples.
 *  
 */
#define NBR_DMA_CAPTURE_SAMPLES 50
#define CAPTURE_CLOCK_HZ 1125000	// 1121549   /* Ideally should be 1.125MHz */


#define FREQIN_DMA_MSK_NEW_FREQ 	0x01
#define FREQIN_DMA_MSK_HIGH_RANGE 0x02
#define FREQIN_DMA_MSK_OVF 				0x04

/*!
 * 	A structure for measuring frequency using DMA to capture pulses
 * 	
 * 	The DMA frequency measurement result is either a 32bit period ulPeriod (low 
 * 	to med frequency) or a 16bit quotient uPeriodNum/uPeriodDen when high
 * 	frequency range is detected fHighFreq.
 * 	
 * 	Every time we have a new reading either by pulse detection of input signal
 * 	or every half	the capture timer overflows, the flag bPulseCtr is inc
 * 	   
 */
typedef struct
{

	uint16 dmaTblIndex, dmaTblIndexPrev;
	// uint16 uMswCapTmr;
	uint32 uLastCapture;					// Last captured timer value
	
	union
	{
		uint32 ulTot;
		struct stPeriod
		{
			uint16 uNum, uDen;
		} sFraction;
		
	} sPeriod;
	bool	
	fNewFreq, fHighFreq, fAmbiguitySolved, fDisplay, fDmaChReady;
} stDmaFrequency;


/*************************************************************************
  *   $GLOBAL PROTOTYPES
*************************************************************************/

void freqDmaRun(void);		// The application
void timerCaptureIsr(void );
void initFreqDma(void);

/*************************************************************************
  *   $GLOBAL VARIABLES
*************************************************************************/
extern unsigned int mSSamplePre, timerTicks;
extern LDD_TDeviceData *TimerPtr;
extern  stDmaFrequency atFreqIn[];
extern uint8 bChannelEnblMsk;

/*************************************************************************
  *   $INLINE FUNCTIONS 
  *************************************************************************/


#endif /* FREQDMA_H_ */

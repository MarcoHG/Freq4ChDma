/*
 * FreqDma.c
 *
 *  Created on: Jul 2, 2014
 *      Author: Marco.HenryGin
 */
//==============================================================================
//  INCLUDES
//==============================================================================
// Typical PEX include
#include "Cpu.h"
#include "Events.h"
#include "CsIO1.h"
#include "freqDma.h"
#include "IO1.h"
#include "TCAP.h"
#include "TP3.h"
#include "TP4.h"

// #include "TINT1.h"
#include "DMA0.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include <PE_Types.h>
// Libraries
#include <stdio.h>
#include <string.h>
//==============================================================================
//  LOCAL DEFINES
//==============================================================================
//	DMA samples are move to a circular array with indexes 0,1,.. NBR_DMA_CAPTURE_SAMPLES-1
#define DMA_PREV_POS_INDEX(INDX,PPOS) ((int16)(INDX-PPOS) >= 0 ?  INDX - PPOS : NBR_DMA_CAPTURE_SAMPLES - (PPOS - INDX)  )
#define NO_FREQ_DETECT_OVF_COUNTER_PRE	80 /* Time = 43 * 65536/1,125,000  ~ 2.5 Sec */
#define TCAPT_MSW_MAX_VAL 0x3F		/* Define the maximum higher bits value of capture counter. If 0x3F will give 0 to 0x3FFFFF */
//==============================================================================
//  LOCAL PROTOTYPES.
//==============================================================================
/*!
 * These are a set of functions optionally called after the DMA is completed
 * @param UserData pointer to user data passed to interrupt
 */
void dmaTransferCompleteA(LDD_TUserData *UserData);
void dmaTransferCompleteB(LDD_TUserData *UserData);
void dmaTransferCompleteC(LDD_TUserData *UserData);
void dmaTransferCompleteD(LDD_TUserData *UserData);


//==============================================================================
//  GLOBAL DATA
//==============================================================================
unsigned int uCapture=0;
LDD_TDeviceData *TP3_ptr = NULL, *TP4_ptr=NULL;
LDD_TDeviceData *TimerPtr = NULL;
unsigned int timerTicks, ulPeriod;
unsigned short uNbrRestarts, uNbrOvfNoPulsePoll;
uint16 uMswDelta, uMswIndexSampled;
bool bDisplay = FALSE;
bool fNewLine = FALSE;
stDmaFrequency atFreqIn[NBR_FREQ_CHANNELS];
uint32 uPeriodLimit;
uint8 bChannelEnblMsk;
bool displayFloatMode;
//==============================================================================
//  LOCAL DATA
//==============================================================================
int uWastePreset = 500;
bool monitorEnable = FALSE;
static LDD_TDeviceData *DMAPtr = NULL;
static bool afCompleted[NBR_FREQ_CHANNELS];
static LDD_DMA_TTransferDescriptor TransferDesc;
static timerCapCtr=0;
unsigned long int uTimerCaptureClock = CAPTURE_CLOCK_HZ;
/*! 
 * 	DMA Tables where the Frequency Signals are captured
 */
static uint16	aDmaCaptureTbl[NBR_FREQ_CHANNELS][NBR_DMA_CAPTURE_SAMPLES];

//
void (*apfDmaTransferComplete[NBR_FREQ_CHANNELS])(LDD_TUserData *UserData) =
{
	dmaTransferCompleteA, dmaTransferCompleteB, dmaTransferCompleteC, dmaTransferCompleteD			
};
		

/*!
 * Structure to connect PEX TransferDescriptor to Frequency Channels:
 * - DMA Source
 * - DMA Channel number 
 * - DMA Peripheral Trigger source
 */
typedef struct tPexDmaTransferDesc
{
	LDD_DMA_TAddress 				SourceAddress;			//	The Location in memory to move from
	LDD_DMA_TTriggerSource 	TriggerSource;			//	DMA request source (see Kinetis Ref Manual Chapter 3, table 3-24)
	LDD_DMA_TChannelNumber 	ChannelNumber;			//	The DMA channel to use for xfer
	uint32									uCiter;							// 	Current Major ITERation count (15 bit) DMA_TCD0_CITER_ELINKNO
	
} tPexDmaTransferDesc;

const tPexDmaTransferDesc aTransferDesc[NBR_FREQ_CHANNELS] =
{
	//	Src Mem Address and Req# 				DMA Curr.ITERation major loop
	{(LDD_DMA_TAddress)&FTM0_C4V, 24, 	0, 	(uint32)&DMA_TCD0_CITER_ELINKNO	},		/// FTM0_CH4 (Req#24)	using DMA_Ch0
	{(LDD_DMA_TAddress)&FTM0_C5V, 25, 	1, 	(uint32)&DMA_TCD1_CITER_ELINKNO	},		/// FTM0_CH5 (Req#25)	using DMA_Ch1
	{(LDD_DMA_TAddress)&FTM0_C6V, 26, 	2, 	(uint32)&DMA_TCD2_CITER_ELINKNO	},		/// FTM0_CH6 (Req#26)	using DMA_Ch2
	{(LDD_DMA_TAddress)&FTM0_C7V, 27, 	3, 	(uint32)&DMA_TCD3_CITER_ELINKNO	},		/// FTM0_CH7 (Req#27)	using DMA_Ch3
};
	
/*!
 * Define a NULL Transfer descriptor to Initialize
 */
const LDD_DMA_TTransferDescriptor DMA_TRANSFER_DESC_NULL = 
{
  /* UserDataPtr = */                  NULL,
  /* SourceAddress = */                (LDD_DMA_TAddress)0,
  /* SourceAddressOffset = */          (LDD_DMA_TAddressOffset)0,
  /* SourceTransferSize = */           (LDD_DMA_TTransferSize)0,
  /* SourceModuloSize = */             (LDD_DMA_TModuloSize)0,
  /* DestinationAddress = */           (LDD_DMA_TAddress)0,
  /* DestinationAddressOffset = */     (LDD_DMA_TAddressOffset)0,
  /* DestinationTransferSize = */      0,
  /* DestinationModuloSize = */        (LDD_DMA_TModuloSize)0,
  /* TransferMode = */                 LDD_DMA_SINGLE_TRANSFER,
  /* ByteCount = */                    (LDD_DMA_TByteCount)0,
  /* OuterLoopCount = */               (LDD_DMA_TOuterLoopCount)0,
  /* InnerLoopChannelLink = */         FALSE,
  /* InnerLoopLinkedChannel = */       0,
  /* OuterLoopChannelLink = */         FALSE,
  /* OuterLoopLinkedChannel = */       0,
  /* AfterRequestComplete = */         (LDD_DMA_TAfterRequest)LDD_DMA_NO_ACTION,
  /* AddressOffset = */                (LDD_DMA_TAddressOffset)0,
  /* AfterTransferComplete = */        (LDD_DMA_TAfterTransfer)LDD_DMA_NO_ACTION,
  /* SourceAddressAdjustment = */      (LDD_DMA_TAddressOffset)0,
  /* DestinationAddressAdjustment = */ (LDD_DMA_TAddressOffset)0,
  /* ScatterGatherAddress = */         (LDD_DMA_TAddress)0,
  /* BandwidthControl = */             (LDD_DMA_TBandwidthControl)DMA_PDD_NO_STALL,
  /* ChannelAutoSelection = */         TRUE,
  /* ChannelNumber = */                (LDD_DMA_TChannelNumber)0,
  /* TriggerType = */                  LDD_DMA_SW_TRIGGER,
  /* TriggerSource = */                (LDD_DMA_TTriggerSource)0,
  /* PeriodicTrigger = */              FALSE,
  /* DisableAfterRequest = */          FALSE,
  /* Interrupts = */                   FALSE,
  /* OnComplete = */                   FALSE,
  /* OnHalfComplete = */               FALSE,
  /* OnError = */                      FALSE,
  /* OnCompleteEventPtr = */           NULL,
  /* OnErrorEventPtr = */              NULL,
  /* ChannelEnabled = */               FALSE 
};


//==============================================================================
// FUNCTIONS
//==============================================================================

// Call backs after DMA complete, we need each per DMA 
void dmaTransferCompleteA(LDD_TUserData *UserData)
{
  volatile bool *Completed = (volatile bool*)UserData;
  *Completed = TRUE;
  // test TP4_NegVal(TP4_ptr);
}

// Call backs after DMA complete, we need each per DMA 
void dmaTransferCompleteB(LDD_TUserData *UserData)
{
  volatile bool *Completed = (volatile bool*)UserData;
  *Completed = TRUE;
}

// Call backs after DMA complete, we need each per DMA 
void dmaTransferCompleteC(LDD_TUserData *UserData)
{
  volatile bool *Completed = (volatile bool*)UserData;
  *Completed = TRUE;
}

// Call backs after DMA complete, we need each per DMA 
void dmaTransferCompleteD(LDD_TUserData *UserData)
{
  volatile bool *Completed = (volatile bool*)UserData;
  *Completed = TRUE;
}


void clearTable()
{
	memset(&aDmaCaptureTbl[0], 0,sizeof(aDmaCaptureTbl[0][0]));	// Empty data buffer
}

void wasteSometime(uint16 ticks)
{
	uint16 i;
	for(i=0; i< ticks; ++i)
		__asm volatile ("nop");
}
void displayFreqBuffers(void)
{
	uint16 i;
	int16 j;
	const uint8 abPulseIndTbl[] = {'-', '\\' , '|' , '/' }; 
	static uint8 abPulseIndCtr[NBR_FREQ_CHANNELS];
	static uint8 aFreqStr[NBR_FREQ_CHANNELS][40] = {{0},{0},{0},{0}};
	uint8 abStrOut[10];
	float f1;
	
	if(fNewLine)
		printf("\r\n");
	else
		printf("\r");	// Home

	for(i=0; i< NBR_FREQ_CHANNELS; ++i)
	{
		if(atFreqIn[i].fNewFreq)
		{
			atFreqIn[i].fNewFreq	= FALSE;	// One shoot
			sprintf(abStrOut,"[%d%c]",i, abPulseIndTbl[ ++abPulseIndCtr[i]&0x03 ]);
			if(atFreqIn[i].fHighFreq)
				f1 = (float)uTimerCaptureClock*atFreqIn[i].sPeriod.sFraction.uDen/(float)atFreqIn[i].sPeriod.sFraction.uNum;
			else
				f1 = (float)uTimerCaptureClock/atFreqIn[i].sPeriod.ulTot;
			sprintf(aFreqStr[i],"%s %f ",abStrOut, f1);
		}
		printf("%s",aFreqStr[i]);
	}
}		

/*!
 * Timed routine is called at the middle and at the overflow of timer capture counter
 * 
 * The DMA capture table is used to provide the Period of the input signal.
 * 
 * The routine keeps the high part of the capture timer providing extra bits defined by ::TCAPT_MSW_MAX_VAL 
 * 
 * The current DMA index is compared with the one in the previous isr and any change represents the pulses that 
 * have been arrived since then, uNbrEdges. Calculations are made only when at least a pulse is detected, but
 * the high portion of the counter is incremented on the ovf (i.e. when the FTM0_CNT has rolled over 0xFFFF) 
 * 
 * When uNbrEdges is GE than 2, the period is within 16 bits and can be calculated with the last two samples from 
 * DMA table. But also, if more than two pulses are detected, we can go back into the DMA table to get the capture of
 * the pulse uNbrEdges before, with the two captures we can create precise period by the quotient between samples
 * divided by the number of edges.
 * 
 * When the uNbrEdges is 1, we keep a track of the last capture (in full extended bit format) to calculate the low 
 * frequency range.
 *   
 */
void timerCaptureIsr(void)
{
	static uint16 uMswCapTmr =0;	// This is the upper part of the TCAP counter
	uint16 uNbrEdges;
	uint16 i;
	// Increase chance to get a capture in the isr:	//wasteSometime(uWastePreset);	
	TP3_SetVal(TimerPtr);
	for(i=0; i< NBR_FREQ_CHANNELS; ++i)
	if(atFreqIn[i].fDmaChReady && bChannelEnblMsk & (1 << i) )
	{
		// Translate DMA major loop iteration counter (CITR) t2o index in the destination table 
		atFreqIn[i].dmaTblIndex = NBR_DMA_CAPTURE_SAMPLES - *((uint16 *)aTransferDesc[i].uCiter);	// position in table that DMA is going to write in
		// Get the Number of Pulse Edges between this and last OverFlow
		uNbrEdges = atFreqIn[i].dmaTblIndex >= atFreqIn[i].dmaTblIndexPrev ?  atFreqIn[i].dmaTblIndex - atFreqIn[i].dmaTblIndexPrev \
				: atFreqIn[i].dmaTblIndex + (NBR_DMA_CAPTURE_SAMPLES - atFreqIn[i].dmaTblIndexPrev);
		atFreqIn[i].dmaTblIndexPrev = atFreqIn[i].dmaTblIndex;	// for next detection
		// The uNbrEdges indicates the amount of captures since last time-isr
		if(uNbrEdges)
		{
			uint16 uTblIndex1;	
			uint32 uTmrCapture;		// 	Compute the last captured period using full 22 bits
			uTblIndex1 = DMA_PREV_POS_INDEX(atFreqIn[i].dmaTblIndex, 1);		// 	Index of Ts-1 Last (and safe) capture position

			// Keep building the 32bit capture of the last pulse 
			uTmrCapture =  (uint32)((uint32)uMswCapTmr << 16) +	aDmaCaptureTbl[i][uTblIndex1];	// 	Capture at Ts-1
		
			// Get the previous to last captured: 
			if(uNbrEdges >=2)		// If we had 2 or more captures in previous isr need to get it from DMA
			{
				uint16 uTblIndexN = DMA_PREV_POS_INDEX(atFreqIn[i].dmaTblIndex, uNbrEdges);		//	Index of Ts-2 at DMA
				// Unambiguous last 2 captures from DMA
				if(aDmaCaptureTbl[i][uTblIndex1] >= aDmaCaptureTbl[i][uTblIndexN])
					atFreqIn[i].sPeriod.sFraction.uNum = aDmaCaptureTbl[i][uTblIndex1] - aDmaCaptureTbl[i][uTblIndexN];
				else
					atFreqIn[i].sPeriod.sFraction.uNum = aDmaCaptureTbl[i][uTblIndex1] + (0xFFFF - aDmaCaptureTbl[i][uTblIndexN]) +1;
				atFreqIn[i].sPeriod.sFraction.uDen = uNbrEdges-1;
				atFreqIn[i].fHighFreq = TRUE;
			}
			else
			{
				// Need to get the unambiguous from 32 bit values
				if(uTmrCapture >= atFreqIn[i].uLastCapture)
				{
					atFreqIn[i].sPeriod.ulTot = uTmrCapture - atFreqIn[i].uLastCapture;
					atFreqIn[i].fAmbiguitySolved = FALSE;
				}
				else
				{		
					atFreqIn[i].fAmbiguitySolved = TRUE;
					// Ambiguity on lower 16bits
					if((uTmrCapture & 0xFFFF0000) == (atFreqIn[i].uLastCapture & 0xFFFF0000))			// 1) MSW are the same
						atFreqIn[i].sPeriod.ulTot = (uTmrCapture + 0x10000) - atFreqIn[i].uLastCapture;	// Carry over from lower 16 bits
					else	
						// Ambiguity on full 22 bits
						atFreqIn[i].sPeriod.ulTot = (uTmrCapture + (uint32)((uint32)(TCAPT_MSW_MAX_VAL + 1) << 16) ) - atFreqIn[i].uLastCapture;	// Rollover 0x3FFFFF
				
				}
				atFreqIn[i].fHighFreq = FALSE;
			}
			// ========  Just a Debug artifact ========
			atFreqIn[i].uLastCapture = uTmrCapture;
			atFreqIn[i].fNewFreq = TRUE;
			
		}	// if bNbrEdges
		// 	else		bNoPulseIsr = TRUE;	// We have a Timed isr with no pulse in previous - resync the next isr for low frequency if needed
	}	// if( ready && enabled)
	//
	// Interrupt Epilog: Prepare next Interrupt - we avoid getting captures close to 0xFFFF-0x0000 when in the interrupt
	if(FTM0_CNT > 0x7FFF )
		FTM0_C0V = 0xFFFF;		// 	Next isr when compare this value
	else
	{
		// 	We just Crossed 0xFFFF - Keep the higher portion of capture counter 
		if(++uMswCapTmr > TCAPT_MSW_MAX_VAL)	// We just need 6 extra bits
			uMswCapTmr =0;											// Roll-over 0x3FFFFF to 0
		FTM0_C0V = 0x7FFF;						//	Next isr will happen at mid range of counter
	}
	++timerCapCtr;
	TP3_ClrVal(TimerPtr);
}


void initFreqDma(void)
{
	uint8 i;
	
	// DMA channels initialized
	DMAPtr =  DMA0_Init(NULL);	
			
	for(i=0; i< NBR_FREQ_CHANNELS; ++i)
	{
		
			// Initialize transfer descriptor
		TransferDesc = DMA_TRANSFER_DESC_NULL;
			  
			// DMA Source
		TransferDesc.SourceAddress = aTransferDesc[i].SourceAddress;								//	(LDD_DMA_TAddress)&FTM0_C4V;
		TransferDesc.SourceAddressOffset = (LDD_DMA_TAddressOffset)0;								//	Single source		
		TransferDesc.SourceTransferSize = (LDD_DMA_TTransferSize)DMA_PDD_16_BIT;		//	16-bit value
		TransferDesc.SourceModuloSize = (LDD_DMA_TModuloSize)0;											// 	non relevant (single source)
			// DMA Destination
		TransferDesc.DestinationAddress = (LDD_DMA_TAddress)&aDmaCaptureTbl[i];								//	Move to Capture table	
		TransferDesc.DestinationAddressOffset = (LDD_DMA_TAddressOffset)sizeof(aDmaCaptureTbl[i][0]);	// Next write offset
		TransferDesc.DestinationTransferSize = (LDD_DMA_TTransferSize)DMA_PDD_16_BIT;		// moving 16-bit data
		TransferDesc.DestinationModuloSize = (LDD_DMA_TModuloSize)0;										// we will set manually a new address
			    
			// We have Major loop = Move 2 bytes per interrupt and repeat  NBR_DMA_CAPTURE_SAMPLES times 
		TransferDesc.TransferMode = LDD_DMA_NESTED_TRANSFERS;						
		TransferDesc.ByteCount = (LDD_DMA_TByteCount)2;	
		TransferDesc.OuterLoopCount = (LDD_DMA_TOuterLoopCount)NBR_DMA_CAPTURE_SAMPLES;
			    
			//	DMA channel
		TransferDesc.ChannelAutoSelection = FALSE;// TRUE;							// Fixed to ch 0	
		TransferDesc.ChannelNumber = aTransferDesc[i].ChannelNumber;		// Channel 0,1,2,3 from table above (must match PEX)
			    
			//	Trigger
		TransferDesc.TriggerType = LDD_DMA_HW_TRIGGER;							// Triggered by Peripheral request
			  																													//	
		TransferDesc.TriggerSource = aTransferDesc[i].TriggerSource;	// DMA source Table 3-24 == FTM0_CH4,
		TransferDesc.Interrupts = TRUE; 
		TransferDesc.OnComplete = TRUE;																// FALSE won't interrupt when DMA-done	
		TransferDesc.OnCompleteEventPtr = apfDmaTransferComplete[i];	// call this function
		TransferDesc.UserDataPtr = (LDD_TUserData*)&afCompleted[i];		// the UserDataPtr to pass data	
			    
			// After transfer
		TransferDesc.AfterTransferComplete = LDD_DMA_ADDRESS_ADJUSTMENT;
		TransferDesc.DestinationAddressAdjustment = -(2*NBR_DMA_CAPTURE_SAMPLES);	
			  
			//==	Allocate and Enable the DMA channels ==
		if(	DMA0_AllocateChannel(DMAPtr, &TransferDesc) != ERR_OK		||		// 		(FreqDma_AllocateChannel[i])(	DMAPtr[i], &TransferDesc);
				DMA0_EnableChannel(DMAPtr, &TransferDesc) != ERR_OK )
			atFreqIn[i].fDmaChReady = FALSE;
		else
			atFreqIn[i].fDmaChReady = TRUE;
	}
	//	Now setup the Capture Timer
	
	//============ Using PEX DMA, TCAP Components =========================
	// TCAP:TimerUnit_LDD  !!! DO not enable in init code and do not Autoninit in PEX properties !!  
	TimerPtr = TCAP_Init(NULL);		// 	Set the desired PEX Properties and get the pointer to static memory
	
	//	Individually disable Channels 
	FTM0_C0SC &= ~FTM_CnSC_CHIE_MASK;		// Disable CH0 interrupt and repeat for other channels
	FTM0_C0V = 0x7FFF;									//	Need to have DMA ready before OVrs
	FTM0_C4SC &= ~FTM_CnSC_CHIE_MASK;		// Disable CH1 interrupt
	FTM0_C5SC &= ~FTM_CnSC_CHIE_MASK;		// Disable CH2 interrupt
	FTM0_C6SC &= ~FTM_CnSC_CHIE_MASK;		// Disable CH3 interrupt
	FTM0_C7SC &= ~FTM_CnSC_CHIE_MASK;		// Disable CH4 interrupt
	
	    
	// 	Enable the base Timer Module by selecting the clk source in CLKS[FTM1_SC]
	TCAP_Enable(TimerPtr);
}


void freqDmaRun(void)
{
	uint8 i;
	
  bool fTCapCmpIE = 0, afDmaIE[NBR_FREQ_CHANNELS] = {0,0,0,0};
  const char abHelpStr[] = 
  		"\r\nFrequency Measurement Using DMA, press:\r\n" \
  		"'t' to toggle Tcap IE\r\n" \
  		"'1' to '4' to toggle DMA channel IE\r\n" \
  		"'+'/'-' to adjust TCAP Clock frequency calibration\n\r" \
  		"'n' to toggle new line\n\r" \
  		"'?' displays this message\r\n";
  
  
	// My local init
	clearTable();
	
	uPeriodLimit = 40000L;
	uint16 uPreset = 4;
	
	// Init Test points GPIOs
	TP3_ptr = TP3_Init(NULL);
	TP4_ptr = TP4_Init(NULL);
	
	// Init the DMA		
	initFreqDma();
  // DONE! to start capturing, enable the Channel capture IE in the App
  
	bChannelEnblMsk = 0x0F;
  
  // remove Debug UART buffering (stream i/o)
  setvbuf(stdout, NULL, _IONBF, 0); // no buffering on stdout - for printf()
  setvbuf(stdin, NULL, _IONBF, 0); // no buffering on stdin - for getchar() etc
  
  unsigned int uLocalCapture=0;
  
  // Init Periodic Time Interrupt
  // TINT1_Enable(TINT1Ptr);	// Enable interrupts after we get the pointer to PTA17
  int ch =0;
  uCapture = uLocalCapture =0;
  printf("%s", abHelpStr);
  while(1)
  {
  	if( (ch = getchar()) != EOF )
  	switch(ch)
  	{
  	// Diable all interrupts
  	case '0':	
  		
  		FTM0_C0SC &= 	~FTM_CnSC_CHIE_MASK;
  		fTCapCmpIE = FALSE;
  		afDmaIE[0] = afDmaIE[1] = afDmaIE[2] = afDmaIE[3] = FALSE;
  		FTM0_C4SC &=  ~(FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK);
  		FTM0_C5SC &=  ~(FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK);
  		FTM0_C6SC &=  ~(FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK);
  		FTM0_C7SC &=  ~(FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK);
  		break;
  		
  	// Enable/Dsbl DMA on Ch 1
  	case '1' :
  		if(!afDmaIE[0])
  		{
  			afDmaIE[0] = TRUE;
  			FTM0_C4SC |= (FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK);		// Enable TCAP_CH4 DMA request
  		}
  		else
  		{
  			afDmaIE[0] = FALSE;
  			FTM0_C4SC &=  ~(FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK);
  		}
  		
  		break;
  		
  	// Enable/Dsbl DMA on Ch 2  
  	case '2' :
  		if(!afDmaIE[1])
  		{
  			afDmaIE[1] = TRUE;
  			FTM0_C5SC |= (FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK);		// Enable TCAP_CH5 DMA request
  		}
  		else
  		{
  			afDmaIE[1] = FALSE;
  			FTM0_C5SC &=  ~(FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK);
  		}
  		break;
  		
      // Enable/Dsbl DMA on Ch 3  
    case '3' :
    	if(!afDmaIE[2])
    	{
    		afDmaIE[2] = TRUE;
    		FTM0_C6SC |= (FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK);		// Enable TCAP_CH5 DMA request
    	}
    	else
    	{
    		afDmaIE[2] = FALSE;
    		FTM0_C6SC &=  ~(FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK);
    	}
    	break;
      	
      // Enable/Dsb DMA on Ch 4  
    case '4' :
    	if(!afDmaIE[3])
    	{
    		afDmaIE[3] = TRUE;
    		FTM0_C7SC |= (FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK);		// Enable TCAP_CH4 DMA request
    	}
    	else
    	{
    		afDmaIE[3] = FALSE;
    		FTM0_C7SC &=  ~(FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK);
    	}
    	break;
    		
  	// Enable (OvfIsr)
  	case 't' : case 'T':
  		if(!fTCapCmpIE)
  		{
  			fTCapCmpIE = TRUE;
  			FTM0_C0SC |= FTM_CnSC_CHIE_MASK;													// TCAP_CH0 (OVF) ISR will IE will be enabled
  		}
  		else
  		{
  			fTCapCmpIE = FALSE;
  			FTM0_C0SC &= 	~FTM_CnSC_CHIE_MASK;			 
  		}

  				
  	  break;
  	
  	case 'n': case 'N':
  		fNewLine = fNewLine ? FALSE: TRUE;
  		break;
  		
 		case 'f':	case 'F':
 			//  			bDisplay = TRUE;
 			displayFloatMode = displayFloatMode? FALSE : TRUE;
 			break;
 			
 		case '?':
 			// Some help here
 			printf("%s", abHelpStr);
 			printf("\r\n Status: \r\nTCapAdjFreqCalib %lu Hz, TCAPIE: %d, DMA CH0:%d CH1:%d CH2:%d CH3:%d\n\n", uTimerCaptureClock, fTCapCmpIE, \ 
 					afDmaIE[0], afDmaIE[1], afDmaIE[2], afDmaIE[3]);
 			break;
 			
 		case ' ':
 			//fDisplayFreq = TRUE;
 			for(i=0; i< NBR_FREQ_CHANNELS; ++i)
 			if(atFreqIn[i].fDmaChReady && bChannelEnblMsk & (1 << i) )
 				atFreqIn[i].fDisplay = TRUE;		// Display a freq Value when Captured 
 			break;
 		case '+':
 			++uTimerCaptureClock;
 			break;
 		case '-':
 			--uTimerCaptureClock;
 			break;
 			
 		case 'c': case 'C':
 			clearTable();
 			break;
 			
  	}
  	//	Every 5*0.028 = 140mS
  	if(timerCapCtr >= uPreset)
  	{
  		timerCapCtr =0;
  		displayFreqBuffers();
  	}
  	
  		
  }
  //  DMA_Deinit(DMAPtr);
  // TIMER_Deinit(TimerPtr);
	
}



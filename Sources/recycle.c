{
	TransferDesc = DMA_TRANSFER_DESC_NULL;
  DMAPtr = DMA0_Init(NULL);
  /* Initialize transfer descriptor */
  	// Source
  TransferDesc.SourceAddress = (LDD_DMA_TAddress)&FTM0_C4V;											//	(FTM1_C0SC, FTM1_C0V)  FTM1_CH0 capture
  TransferDesc.SourceAddressOffset = (LDD_DMA_TAddressOffset)0;										//	Single source		
  TransferDesc.SourceTransferSize = (LDD_DMA_TTransferSize)DMA_PDD_16_BIT;				//	16-bit value
  TransferDesc.SourceModuloSize = (LDD_DMA_TModuloSize)0;													// 	non relevant (single source)
  	// Destination
  TransferDesc.DestinationAddress = (LDD_DMA_TAddress)&aDmaCaptureTbl[0];								//	Move to Capture table	
  TransferDesc.DestinationAddressOffset = (LDD_DMA_TAddressOffset)sizeof(aDmaCaptureTbl[0][0]);	// Next write offset
  TransferDesc.DestinationTransferSize = (LDD_DMA_TTransferSize)DMA_PDD_16_BIT;		// moving 16-bit data
  TransferDesc.DestinationModuloSize = (LDD_DMA_TModuloSize)0;										// we will set manually a new address
    
    // We have Major loop = Move 4 bytes per interrupt and repeat 8 times 
  TransferDesc.TransferMode = LDD_DMA_NESTED_TRANSFERS;						
  TransferDesc.ByteCount = (LDD_DMA_TByteCount)2;	
  TransferDesc.OuterLoopCount = (LDD_DMA_TOuterLoopCount)NBR_DMA_CAPTURE_SAMPLES;
    
    //	DMA channel
  TransferDesc.ChannelAutoSelection = FALSE;// TRUE;								// Fixed to ch 0	
  TransferDesc.ChannelNumber = (LDD_DMA_TChannelNumber)0;		// Channel 0 in this example
    
    //	Trigger
  TransferDesc.TriggerType = LDD_DMA_HW_TRIGGER;						// Triggered by Peripheral request
  																										//	
  TransferDesc.TriggerSource = (LDD_DMA_TTriggerSource)24;	// DMA source Table 3-24 == FTM0_CH4 
  TransferDesc.Interrupts = TRUE;	// we are here									We are not going to interrupt on DMA transfer complete 
  TransferDesc.OnComplete = TRUE;										// Signal an DMA-done	
  TransferDesc.OnCompleteEventPtr = &TransferComplete;			// call this function
  TransferDesc.UserDataPtr = (LDD_TUserData*)&Completed;		// the UserDataPtr to pass data	
    
    // AFter transfer
  TransferDesc.AfterTransferComplete = LDD_DMA_ADDRESS_ADJUSTMENT;
  TransferDesc.DestinationAddressAdjustment = -(2*NBR_DMA_CAPTURE_SAMPLES);	
  
  /* Start DMA transfer */
  DMA0_AllocateChannel(DMAPtr, &TransferDesc);					// Still need to call this method even  we have a fixed channel
  DMA0_EnableChannel(DMAPtr, &TransferDesc);						// This moves the TransferDesc to the TCD 32 bytes
  // DMA0_StartChannelTransfer(DMAPtr, &TransferDesc);
  //=============================================  
}  
/*!
 * Routine called at the middle and the overflow of timer capture 16 bit counter
 * 
 * The DMA capture table is used to provide the Period of the input signal.
 * 
 * The routine provides the high part of capture timer, uMswCapTmrthe 16bit capture counter with a 8 bit higher byte ,  with acreates the upper part function counts the number of Overflows which indeed reflects the upper 16-bit of the capture timer.
 * 
 * The current DMA index is compared with the one in the previous ovf, any change represents the pulses that 
 * have been arrived since then.
 * 
 * 
 *  
 * the amount of carry overs elpased on the capture timer counter  
 */
void timerCaptureIsr(void)
{
	bool fOvfHalf = FALSE, bCatch = FALSE;
	uint16 	dmaTblIndex;	// , lTable[NBR_DMA_CAPTURE_SAMPLES];
	// These should be part of the structure
	
	static uint16 dmaTblIndexPrev;
	static uint16 uMswCapTmr=0;
	static uint32 uLastCapture;					// Last captured timer value
	
	uint16 uNbrEdges;
	TP3_SetVal(TimerPtr); 
	
	// Increase chance to get a capture in the isr
	// wasteSometime(500);	
	
	// Prepare next Interrupt - we avoid getting captures close to 0xFFFF-0x0000 when in the interrupt
	if(FTM0_CNT > 0x7FFF )
		FTM0_C0V = 0xFFFF;// 0xFFE0;									//	We just crossed 0x7F00
	else
	{
		fOvfHalf = TRUE;										// 	We just Crossed 0xFFE0
		FTM0_C0V = 0x7FFF;									//	Need to have DMA ready before OVrs
	}
	
	// 	uCiter = DMA_TCD0_CITER_ELINKNO;			// Get the current DMA index, and use this value all over the routine
		
	// Translate DMA major loop iteration counter (CITR) to index in the destination table 
	dmaTblIndex = NBR_DMA_CAPTURE_SAMPLES - DMA_TCD0_CITER_ELINKNO;	// Points to position in table that DMA is going to write in
		
	// Get the Number of Pulse Edges between this and last OverFlow
	uNbrEdges = dmaTblIndex >= dmaTblIndexPrev ?  dmaTblIndex - dmaTblIndexPrev : dmaTblIndex + (NBR_DMA_CAPTURE_SAMPLES - dmaTblIndexPrev);
	dmaTblIndexPrev = dmaTblIndex;	// for next detection
	
	// The uNbrEdges indicates the amount of captures since last time-isr
	if(uNbrEdges)
	{
		uint16 uTblIndex1, uTblIndex2, uTblIndexN;	
		uint32 uTmrCapture, uTmrCapture1, uPeriod;							// 	Last captured period
		uTblIndex1 = DMA_PREV_POS_INDEX(dmaTblIndex,1);		// 	Index of Ts-1 Last (and safe) capture position
		uTblIndex2 = DMA_PREV_POS_INDEX(dmaTblIndex,2);		//	Index of Ts-2 at DMA
		
		
		uTblIndexN = DMA_PREV_POS_INDEX(dmaTblIndex,uNbrEdges);		// Extend precision at High frequency
		//
		// Examine if we receive a Capture inside this isr
		if( fOvfHalf && aDmaCaptureTbl[0][uTblIndex1] < aDmaCaptureTbl[0][uTblIndex2])
			bCatch = TRUE;
		// Build the 32bit capture of the last pulse 
		uTmrCapture = 	bCatch? uMswCapTmr +1 : uMswCapTmr;	
		uTmrCapture <<=	16;
		uTmrCapture += 	aDmaCaptureTbl[0][uTblIndex1];
		
		
		
		
		//	Build the 32bit capture of the previous to last pulse
		if(uNbrEdges == 1)						
			uTmrCapture1 = uLastCapture;		// Use the last known value
		else
		{
			// We have uNbrEdges >= 2, Calculate Period using the previous to last DMA sample instead
			uTmrCapture1  = uMswCapTmr;
			uTmrCapture1  <<= 16;
			uTmrCapture1  += aDmaCaptureTbl[0][uTblIndex2];	// 	Capture at Ts-2
		}
		// Solve for a 22 bit capture timer ambiguity 
		if(uTmrCapture >= uTmrCapture1)
			uPeriod = uTmrCapture - uTmrCapture1;
		else
			uPeriod = uTmrCapture + (0x3FFFFF - uTmrCapture1) +1;
		
		// ========  Just a Debug artifact ======== 
		if(bDisplay || bCatch) // && iCiter == DMA_TCD0_CITER_ELINKNO)	// Don't display if a new capture
		{
			printf("\r\n%6u %d %u %u %u", uPeriod, uNbrEdges, uTmrCapture, uTmrCapture1, uLastCapture);
			if(uNbrEdges >= 2) 
				printf(" Ex-> [%u, %u]", aDmaCaptureTbl[0][uTblIndex1], aDmaCaptureTbl[0][uTblIndexN] );		// this will extend the counter
			if(bCatch)
				printf("= Capture inside=");
			bDisplay = FALSE;
		}
		
		uLastCapture = uTmrCapture;
		
	}
	// 	else		bNoPulseIsr = TRUE;	// We have a Timed isr with no pulse in previous - resync the next isr for low frequency if needed
	
	// Epilog: keep the 6 bit msw 
	if(fOvfHalf)
		if(++uMswCapTmr > 0x3F)	// We just need 6 extra bits	
			uMswCapTmr =0;
	
	TP3_ClrVal(TimerPtr); 
	
}

#if 0
typedef struct
{
	union 
	{
		unsigned short int w[2];
		unsigned int l;
	} utCalcPeriod;

	unsigned short int uNbrEdges;			/* 	Number of Pulse Edges detected in the last overflow isr	*/
	unsigned char fNewFreq;
	
} stDmaFrequency;
#endif

// ===================
	// INIT DMA
	

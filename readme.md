Measuring Several Frequency channels using DMA 
=====================================================================================
Author: MHG, on 7/02/14
Measures frequency on Kinetis FTM timers using DMA and the prototype #1 board.

Four channels working, handed to Sander L. to have other user testings 7.10.14
------------------------------------------------------------------------------
- More debug and controls to enable/disable channels

		
First commit 7.08.14
-------------------------------------------------
- Single channel
	+ FTM0_CH0 is used asd compare for timed isr every half cycle and to obtain the captured channels
	+	FTM0_CH4 is the first S3L/Cfrequency input


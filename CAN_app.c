#include "MPC5606B.h"

void PIT1_ISR(void);

uint8_t LED_state		= 0;
uint32_t myData			= 0;
uint16_t j;
uint8_t i = 0;


void init_Clock(void);
void init_DSPI_1(void);
uint16_t ReadDataDSPI_1(void);
void init_SBC_Driver(void);
void config_PortE(void);
void init_PIT_1(void);
void initCAN_1(void);
void TransmitMsg(uint8_t TxData, uint16_t MsgID);
void ReceiveMsg(uint32_t *RxCode, uint32_t *RxID, uint32_t *RxLENGTH, uint32_t *RxTIMESTAMP, uint32_t *validMessages, uint8_t *RxDataArray);
void disableWatchdog(void);
void PIT1_ISR(void);

void init_Clock(void){

	//CGM.SC_SS.R 			|= 0x00000000;	// Select System Clock



	ME.MER.B.DRUN 			|= 1;			// Enable DRUN, RUN, SAFE, RESET modes from MODE ENTRY register (MER)
	ME.MER.B.RUN 			|= 1;
	ME.MER.B.SAFE			|= 1;
	ME.MER.B.RESET  		|= 1;

	//CGM.FXOSC_CTL.B.OSCBYP 	= 0;		  	// Adjust Fast External Oscillator Status
	//CGM.FXOSC_CTL.B.OSCDIV 	= 4;			// Adjust Fast External Oscillator Division Parameter
	CGM.FXOSC_CTL.R			= 0x00800000;	// Select FXOSC as root clock, No division, Interrupt disabled

	ME.RUN[0].B.PDO			|= 1;			// Adjust RUN[0]'s parameters (PLL=off, FXOSC=ON, D,CFLAON=Normal_Mode, MainVoltageRegulatorOn=ON, FastInternal=off)
	ME.RUN[0].B.MVRON		|= 1;
	ME.RUN[0].B.DFLAON		|= 3;
	ME.RUN[0].B.CFLAON		|= 3;
	ME.RUN[0].B.FMPLLON 	&= 0;
	ME.RUN[0].B.FIRCON		&= 0;
	ME.RUN[0].B.FXOSCON		|= 1;
	ME.RUN[0].B.SYSCLK 		&= 0;

	ME.RUNPC[1].B.RUN0		|= 1;			// Enable RUN0 for Peripheral Run Configuration


	ME.PCTL[68].B.RUN_CFG	= 1;			// Enable Clock gating to SIUL
	ME.PCTL[92].B.RUN_CFG	= 1;			// Enable Clock gating to PIT
	ME.PCTL[17].B.RUN_CFG	= 1;			// Enable Clock gating to CAN1 (16 + CAN_n)
	ME.PCTL[5].B.RUN_CFG	= 1;			// Enable Clock gating to DSPI1 (4 + DSPI_n)

	//ME.MCTL.B.KEY			= 0x5AF0;		// Set Key value proper for writing
	//ME.MCTL.B.TARGET_MODE	= 4;			// Enable RUN0 for mode control of actual device
	//ME.MCTL.B.KEY			= 0xA50F;		// Set Key value proper for writing
	//ME.MCTL.B.TARGET_MODE	= 4;			// Enable RUN0 for mode control of actual device


	ME.MCTL.R = 0x40005AF0;         /* Enter RUN0 Mode & Key */
	ME.MCTL.R = 0x4000A50F;         /* Enter RUN0 Mode & Inverted Key */

	while (ME.GS.B.S_MTRANS);    			// Wait for mode transition to complete
	while(ME.GS.B.S_CURRENTMODE != 4);		// Verify RUN0 is the current mode


}

void init_DSPI_1(void){

			/* Module Configuration Register */

	DSPI_1.MCR.B.MSTR		|= 1;			// Select master mode
	DSPI_1.MCR.B.CONT_SCKE	&= 0; 			// Disable continuous SCK
	DSPI_1.MCR.B.DCONF		&= 0;			// Configure as SPI =>> 00
	DSPI_1.MCR.B.FRZ		&= 0;			// Disable Freeze operation in HALT stage
	DSPI_1.MCR.B.MTFE		&= 0;			// Disable Modified Timing Format usage
	DSPI_1.MCR.B.PCSSE		&= 0;			// Disable Peripheral Chip Select Strobe (CS5_x is used as the Peripheral chip select 5 signal)
	DSPI_1.MCR.B.PCSIS5		&= 0;			// Inactive value = 0
	DSPI_1.MCR.B.PCSIS4		&= 0;			// Inactive value = 0
	DSPI_1.MCR.B.PCSIS3		&= 0;			// Inactive value = 0
	DSPI_1.MCR.B.PCSIS2		&= 0;			// Inactive value = 0
	DSPI_1.MCR.B.PCSIS1		&= 0;			// Inactive value = 0
	DSPI_1.MCR.B.PCSIS0		&= 1;			// Inactive value = 1

	DSPI_1.MCR.B.MDIS		&= 0;			// DSPI Clock is Enabled
	DSPI_1.MCR.B.DIS_TXF	&= 0;			// Enable Transmit FIFO
	DSPI_1.MCR.B.DIS_RXF	&= 0;			// Enable Receive FIFO
	DSPI_1.MCR.B.CLR_TXF	&= 0;			// Disable Counter clearing for Tx FIFO
	DSPI_1.MCR.B.CLR_RXF	&= 0; 			// Disable Counter clearing for Rx FIFO
	DSPI_1.MCR.B.SMPL_PT	&= 0;			// Set Sample Point to 0 
	DSPI_1.MCR.B.HALT		|= 1;			// Enable HALTing

			/* Clock and Transfer Attributes Register */

	DSPI_1.CTAR[0].B.DBR	&= 0;			// Disable double baud rate
	DSPI_1.CTAR[0].B.FMSZ	= 15;			// Adjut Frame Size to 16-Bit
	DSPI_1.CTAR[0].B.CPOL	&= 0;			// Clock Polarity Active-High
	DSPI_1.CTAR[0].B.CPHA	&= 0;			// Set clock phase settings to 0
	DSPI_1.CTAR[0].B.LSBFE	&= 0;			// Disable LSB first bit
	DSPI_1.CTAR[0].B.PCSSCK &= 0;			// Set CS to SCK delay Prescaler to 0
	DSPI_1.CTAR[0].B.PASC	&= 0;			// Set After SCK delay Prescaler to 0
	DSPI_1.CTAR[0].B.PDT	&= 0;			// Set Delay After Transmission Prescaler to 0
	DSPI_1.CTAR[0].B.PBR	&= 2;			// Set Baud Rate Prescaler to 2
	DSPI_1.CTAR[0].B.CSSCK  &= 4;			// Tcsc = 4us
	DSPI_1.CTAR[0].B.ASC	&= 4;			// Tasc = 4us
	DSPI_1.CTAR[0].B.DT		&= 2;			// Tdt 	= 1us (delay after transfer scaler)
	DSPI_1.CTAR[0].B.BR		&= 4;			// BaudRate 400000

			/* Module Configuration Register */
	DSPI_1.MCR.B.HALT		&= 0;			// Disable HALTing

			/* SIUL Operation*/

	SIU.PCR[112].B.SMC		&= 0;			// Disable output buffer in Safe Mode, SIN_1
	SIU.PCR[112].B.APC		&= 0;			// Disable Analog Pad gating for ADC
	SIU.PCR[112].B.PA		&= 0;			// 0=GPIO, 1=AF1, 2=AF2 ...
	SIU.PCR[112].B.OBE		&= 0;			// Disable Output Buffer
	SIU.PCR[112].B.IBE		|= 1;			// Enable Input Buffer
	SIU.PCR[112].B.ODE		&= 0;			// Disable Open Drain
	SIU.PCR[112].B.SRC		&= 0;			// Configure Slew Rate as slow
	SIU.PCR[112].B.WPE		|= 1;			// Enable Weak Pull Up/Down resistor
	SIU.PCR[112].B.WPS		|= 1;			// Select Pull Up resistor as weak resistor

	SIU.PSMI[8].B.PADSEL	= 2;			// Pad Selection for PCR_112 to SIN_1
	SIU.PSMI[9].B.PADSEL	= 3;			// Pad Selection for PCR_115 to CS0_1
	SIU.PSMI[7].B.PADSEL	= 2;			// Pad Selection for PCR_114 to SCK_1

	SIU.PCR[113].R 			= 0x0A04;		// AF2, WPE:off, OBE:ON, Slew Rate:FAST, SOUT_1
	SIU.PCR[114].R 			= 0x0A04;		// AF2, WPE:off, OBE:ON, Slew Rate:FAST, SCK_1
	SIU.PCR[115].R 			= 0x0A04;		// AF2, WPE:off, OBE:ON, Slew Rate:FAST, CS0_1
}

uint16_t ReadDataDSPI_1(void) {
	uint16_t spiData = 0;
	while (DSPI_1.SR.B.RFDF != 1){} /* Wait for Receive FIFO Drain Flag = 1 				*/
  	spiData = DSPI_1.POPR.R; 		/* Read data received by slave SPI 						*/
  	DSPI_1.SR.R = 0x80020000;       /* Clear TCF, RDRF flags by writing 1 to them 			*/
  	return spiData;
}

void init_SBC_Driver(void){//Ne iş yapıyor bu ?
	vuint32_t i;
	DSPI_1.PUSHR.R = 0x0001DF80;
	(void)ReadDataDSPI_1();            	/* A dummy read after each command						*/
	for (i=0; i<200; i++);				/* Wait a while for operations to be completed			*/

	DSPI_1.PUSHR.R = 0x00015A00;
	(void)ReadDataDSPI_1();
	for (i=0; i<200; i++);

	DSPI_1.PUSHR.R = 0x00015E90;
	(void)ReadDataDSPI_1();
	for (i=0; i<200; i++);

	DSPI_1.PUSHR.R = 0x000160C0;
	(void)ReadDataDSPI_1();
	for (i=0; i<200; i++);
}

void config_PortE(void){


	SIU.PCR[64].B.SMC		&= 0;			// On board push button ..PE[0]..
	SIU.PCR[64].B.APC		&= 0;
	SIU.PCR[64].B.PA		&= 0;
	SIU.PCR[64].B.OBE		&= 0;
	SIU.PCR[64].B.IBE		|= 1;
	SIU.PCR[64].B.ODE		&= 0;
	SIU.PCR[64].B.SRC		&= 0;
	SIU.PCR[64].B.WPE		&= 0;
	SIU.PCR[64].B.WPS		&= 0;

	SIU.PCR[65].R			= 0x0100;		// Same configuration with Pcr_64 (On board push button)

	SIU.PCR[68].R			= 0x0200;		// OBE enabled for on board LED
	SIU.PCR[69].R			= 0x0200;		// OBE enabled for on board LED
	SIU.PCR[70].R			= 0x0200;		// OBE enabled for on board LED
	SIU.PCR[71].R			= 0x0200;		// OBE enabled for on board LED
}

void init_PIT_1(void){//clok ayarları yapılıyor işte..

	PIT.PITMCR.B.MDIS		&= 0;
	PIT.PITMCR.B.FRZ		|= 1;

	PIT.CH[1].LDVAL.B.TSV	= 1600000;
	PIT.CH[1].TCTRL.B.TIE	|= 1;
	PIT.CH[1].TCTRL.B.TEN	|= 1;

}

void initCAN_1(void){


	CAN_1.MCR.B.MDIS		&= 0;			// Enable module
	CAN_1.MCR.B.FRZ			|= 1;			// Enable Freeze for debug mode
	CAN_1.MCR.B.FEN			&= 0;			// Disable FIFO buffer
	CAN_1.MCR.B.HALT		|= 1;			// Put module in HALTing
	CAN_1.MCR.B.SOFTRST		&= 0;			// Disable Soft Reset

	CAN_1.MCR.B.SUPV		&= 0;			// Disable Supervisor mode
	CAN_1.MCR.B.WRNEN		&= 0;			// Disable Warning Interrupt
	CAN_1.MCR.B.SRXDIS		&= 0;			// Enable Self Reception
	CAN_1.MCR.B.BCC			&= 0;			// Disable Backward Compatibility
	CAN_1.MCR.B.LPRIO_EN	&= 0;			// Local Priority disabled
	CAN_1.MCR.B.AEN			&= 0;			// Disable Abort
	CAN_1.MCR.B.IDAM		&= 0;			// ID mode A
	CAN_1.MCR.B.MAXMB		= 7;			// Adjust message buffer for 8 bit

							//CAN_1.CR.R = 0x04DB0006;

	CAN_1.CR.B.PRESDIV		= 4;
	CAN_1.CR.B.RJW			= 3;
	CAN_1.CR.B.PSEG1		= 3;
	CAN_1.CR.B.PSEG2		= 3;
	CAN_1.CR.B.BOFFMSK		&= 0;			// Bus off interrupt disabled
	CAN_1.CR.B.ERRMSK		&= 0;			// Error interrupt disabled
	CAN_1.CR.B.CLKSRC		&= 0;			// Adjust Clock source as oscillator NOT Bus
	CAN_1.CR.B.TWRNMSK		&= 0;			// Disable Tx Warning Interrupt
	CAN_1.CR.B.RWRNMSK		&= 0;			// Disable Rx Warning Interrupt
	CAN_1.CR.B.LPB			&= 0;			// Disable Loop Back Mode
	CAN_1.CR.B.SMP			&= 0;			// Use one sample to determine bit value (Extra sampling disabled)
	CAN_1.CR.B.BOFFREC		&= 0;			// Automatic Recovering from Bus-OFF State is enabled
	CAN_1.CR.B.TSYN			&= 0;			// Disable Timer Synchronization
	CAN_1.CR.B.LBUF			&= 0;			// Higher Priority Buffer is transmitted first (Lower Last)
	CAN_1.CR.B.LOM			&= 0;			// Deactivate Listen-Only Mode
	CAN_1.CR.B.PROPSEG		= 6;			// Adjust Propagation Segmentation to 6+1 = 7 Bit

	
	for (i=0; i<64; i++) {
		CAN_1.BUF[i].CS.B.CODE &= 0;   		// Clear Message buffers
	}

		/* CS: Control & Status, ID: Identification, DATA: Data Buffer, All these modules are part of CAN Message Buffer*/

	CAN_1.BUF[0].CS.B.CODE	= 8;			// Adjust Message Buffer (MB) for Tx purposes (Init as TX inactive)

	CAN_1.BUF[1].CS.B.IDE	&= 0;			// By default MB is in Rx Mode and Adjust ID format to Standard (Not extended)
	CAN_1.BUF[1].ID.B.STD_ID= 666U;			// Set Standard ID of the Module as 666
	CAN_1.BUF[1].CS.B.CODE	= 4;			// Set Buffer 1 to RX Empty mode

	CAN_1.RXGMASK.B.MI		= 0x07FF0000;	// Adjust RX Global Mask Register in order to check whether "don't care" or "check" for specific MB bits.

	SIU.PCR[42].B.SMC		&= 0;			// Adjust PC_10, PCR[42] for CAN_1_Tx (Open-Drain, AF1, Slew Rate:Fast, Output Buffer: ON
	SIU.PCR[42].B.APC		&= 0;
	SIU.PCR[42].B.PA		= 1;
	SIU.PCR[42].B.OBE		|= 1;
	SIU.PCR[42].B.IBE		&= 0;
	SIU.PCR[42].B.ODE		|= 1;
	SIU.PCR[42].B.SRC		|= 1;
	SIU.PCR[42].B.WPE		&= 0;
	SIU.PCR[42].B.WPS		&= 0;

	SIU.PCR[43].R			= 0x0100;		// Adjust Port_C_11 for CAN_1_Rx (Input Buffer Enabled)
	SIU.PSMI[0].B.PADSEL	= 0x01;			// In Pad Selection Multiplexed Inputs Register, Multiplex CAN_1_Rx to PCR[43]

	CAN_1.MCR.R				= 0x00000007;	// Enable CAN_1 Module, Disable Freeze, Disable HALTing
}

void TransmitMsg(uint8_t TxData, uint16_t MsgID) {
	uint8_t	i;
		/* Assumption:  Message buffer CODE is INACTIVE --> done in initCAN1 */
	CAN_1.BUF[0].CS.B.IDE 	= 0;           	/* Use standard ID length */
	CAN_1.BUF[0].ID.B.STD_ID= MsgID;      	/* Transmit ID */
	CAN_1.BUF[0].CS.B.RTR 	= 0;          	/* Data frame, not remote Tx request frame */
	CAN_1.BUF[0].CS.B.LENGTH= 1;
	CAN_1.BUF[0].DATA.B[0] 	= TxData;  		/* Data to be transmitted */

	for (i=1; i<8; i++) {
		CAN_1.BUF[0].DATA.B[i] = 0;			// Clear Data Buffer in Message Buffer
	}

	CAN_1.BUF[0].CS.B.SRR 	= 1;           	/* Tx frame (not required for standard frame)*/
	CAN_1.BUF[0].CS.B.CODE 	= 0xC;         	/* Activate msg. buf. to transmit data frame */
}

void ReceiveMsg(uint32_t *RxCode, uint32_t *RxID, uint32_t *RxLENGTH, uint32_t *RxTIMESTAMP, uint32_t *validMessages, uint8_t *RxDataArray){
	while( CAN_1.IFRL.B.BUF01I == 0);			// Wait until message has been received

	*RxCode 				= CAN_1.BUF[1].CS.B.CODE;		// Get Received message Rx code
	*RxID 					= CAN_1.BUF[1].ID.B.STD_ID;		// Get Standard ID of transmitter module
	*RxLENGTH				= CAN_1.BUF[1].CS.B.LENGTH;		// Get Length of message in byte

	
	for (j=0; j<*RxLENGTH; j++){
		*(RxDataArray + j)	= CAN_1.BUF[1].DATA.B[j];
	}

	*RxTIMESTAMP 			= CAN_1.BUF[1].CS.B.TIMESTAMP;	// Get Timer Value when Receive occur
	(void)CAN_1.TIMER.R;									// To unlock Message Buffer read Timer module
	CAN_1.IFRL.R 			= 0x00000000;

	SIU.PGPDO[2].B.PPD0		|= (*RxDataArray & 0x08) << 28;		// Set On board LED according to last 4 bit f Can message
	SIU.PGPDO[2].B.PPD0		|= (*RxDataArray & 0x04) << 27;
	SIU.PGPDO[2].B.PPD0		|= (*RxDataArray & 0x02) << 26;
	SIU.PGPDO[2].B.PPD0		|= (*RxDataArray & 0x01) << 25;

	*validMessages			+= 1;							// Increase number of valid message count

}

void disableWatchdog(void) {
  SWT.SR.R = 0x0000c520;     								// This value and following value are written to clear Soft Lock Bit in Control register
  SWT.SR.R = 0x0000d928;
  SWT.CR.R = 0x8000010A;     								// Disable WatchDog, Enable Freeze, Enable Reset onInvalid Acces (RIA) Select Module clock as Oscillator
}

void PIT1_ISR(void)
{
	//Clear PIT1 flag
	PIT.CH[1].TFLG.B.TIF 	= 1;

	TransmitMsg(myData,666U);

	myData++;
	if(LED_state == 0)
	{
		LED_state 			= 1;
		SIU.PGPDO[2].R 		= 0x0A000000;
	}
	else
	{
		LED_state 			= 0;
		SIU.PGPDO[2].R 		= 0x05000000;
	}
}

int main(void)
{
	//vuint32_t counter 		= 0;
	uint8_t Program_Status	= 1;
	uint32_t RxCode			= 0;
	uint32_t RxID			= 0;
	uint32_t RxLENGTH		= 0;
	uint32_t RxTIMESTAMP	= 0;
	uint32_t validMessages	= 0;
	uint8_t RxDataArray[8];

	disableWatchdog();
	init_Clock();
	init_DSPI_1();//SPI ayarlarını yaptık
	init_SBC_Driver();//SPI dan veri okuyor yanlış anlamadıysam
	config_PortE();// bu önemli değil port ayarlaması yapıyor..
	init_PIT_1();//clock (ınterrupt ) ayarlanıyor ama nedenn ???
	initCAN_1();//CAN in bütün ayarlamaları yapılıyor

	INTC.CPR.B.PRI = 0;			// Ensure that current priority register value is 0
	INTC.PSR[60].B.PRI = 2;		// Set PIT1 interrupt priority to any value bigger than 0


	/* Loop forever */
	for(;;) {

		if(SIU.GPDI[64].B.PDI) Program_Status = 0;//low ise 0
		if(SIU.GPDI[65].B.PDI) Program_Status = 1;//high ise 1

		switch(Program_Status){
		case 0:
			PIT.CH[1].TCTRL.B.TEN &= 0;			// Disable Timer
			break;

		case 1:
			PIT.CH[1].TCTRL.B.TEN |= 1;			// Enable Timer
			break;
		}

		if( CAN_1.IFRL.B.BUF01I != 0){
			ReceiveMsg(&RxCode, &RxID, &RxLENGTH, &RxTIMESTAMP, &validMessages, &RxDataArray[0]);
		}

		//for(counter=0; counter < 500000; counter++);
		//SIU.GPDO[70].B.PDO = 0;
		//for(counter=0; counter < 500000; counter++);
		//SIU.GPDO[70].B.PDO = 1;
	}
}

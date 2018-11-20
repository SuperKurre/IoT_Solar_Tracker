/*
 * SPI.cpp*
 *
 */

#include "SPI.h"


/*Constructor*/
SPI::SPI() {

	Init_PinMux();
	Setup_Master();
}

/*Initialize pin mux for SPI*/
void SPI::Init_PinMux(void){

	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	/*
	 * Initialize SPI0 pins connect
	 * SCK0: PINASSIGN3[15:8]: Select P0.0
	 * MOSI0: PINASSIGN3[23:16]: Select P0.16
	 * MISO0: PINASSIGN3[31:24] : Select P0.10
	 * SSEL0: PINASSIGN4[7:0]: Select P0.9
	 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 0, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 29, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 10, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 9, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));

	Chip_SWM_MovablePinAssign(SWM_SPI0_SCK_IO, 0);					/* P0.0 */
	Chip_SWM_MovablePinAssign(SWM_SPI0_MOSI_IO, 29);				/* P0.29 */
	Chip_SWM_MovablePinAssign(SWM_SPI0_MISO_IO, 10);				/* P0.10 */
	Chip_SWM_MovablePinAssign(SWM_SPI0_SSELSN_0_IO, 9);				/* P0.9 */

	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
}


/* Setup SPI handle and parameters */
void SPI::Setup_Master(void)
{
	SPI_CFG_T spi_cfg;
	SPI_DELAY_CONFIG_T spi_del_cfg;

	/* Initialize SPI Block */
	Chip_SPI_Init(LPC_SPI0);

	/* Set SPI Config register */
	spi_cfg.ClkDiv = 0x48;										/* Set SPI clock to 1Mhz */
	spi_cfg.Mode = SPI_MODE_MASTER;								/* Enable Master Mode */
	spi_cfg.ClockMode = SPI_CLOCK_MODE0;						/* Enable Mode 0 : CPHA = 0, CPOL = 0*/
	spi_cfg.DataOrder = SPI_DATA_MSB_FIRST;						/* Transmit MSB first */
	spi_cfg.SSELPol = (SPI_CFG_SPOL0_LO);						/* Slave select polarity is active low */
	Chip_SPI_SetConfig(LPC_SPI0, &spi_cfg);

	/* Set Delay register */
	spi_del_cfg.PreDelay = 			PRE_DELAY;
	spi_del_cfg.PostDelay = 		POST_DELAY;
	spi_del_cfg.FrameDelay = 		FRAME_DELAY;
	spi_del_cfg.TransferDelay = 	TRANSFER_DELAY;
	Chip_SPI_DelayConfig(LPC_SPI0, &spi_del_cfg);

	/*Enable SPI0*/
	Chip_SPI_Enable(LPC_SPI0);
}

/* Write data from tx_data_buff to SPI */
bool SPI::Write_SPI_Data(uint16_t* tx_data_buff){

	/* SPI Transfer Setup */
	SPI_DATA_SETUP_T transfer_setup;

	transfer_setup.pTx = tx_data_buff;						/* Transmit Buffer */
	transfer_setup.DataSize = 8;							/*size of frame in bits*/
	transfer_setup.Length = 3;								/* Total frames/transmit */

	/* Assert only SSEL0 */
	transfer_setup.ssel = SPI_TXCTL_ASSERT_SSEL0 | SPI_TXCTL_DEASSERT_SSEL1 | SPI_TXCTL_DEASSERT_SSEL2 |
			SPI_TXCTL_DEASSERT_SSEL3;
	transfer_setup.TxCnt = 0;
	transfer_setup.RxCnt = 0;

	if (Chip_SPI_WriteFrames_Blocking(LPC_SPI0, &transfer_setup) > 0) {
		/*transmit sucessful*/
		return true;
	}
	else {
		return false;
	}
}

/*Read data from SPI to rx_data_buff*/
bool SPI::Read_SPI_Data(uint16_t* rx_data_buff){

	/* SPI Transfer Setup */
	SPI_DATA_SETUP_T transfer_setup;

	transfer_setup.pRx = rx_data_buff;						/* Transmit Buffer */
	transfer_setup.DataSize = 8;							/*size of frame in bits*/
	transfer_setup.Length = 3;								/* Total frames/transmit */

	/* Assert only SSEL0 */
	transfer_setup.ssel = SPI_TXCTL_ASSERT_SSEL0 | SPI_TXCTL_DEASSERT_SSEL1 | SPI_TXCTL_DEASSERT_SSEL2 |
			SPI_TXCTL_DEASSERT_SSEL3;
	transfer_setup.TxCnt = 0;
	transfer_setup.RxCnt = 0;

	if (Chip_SPI_ReadFrames_Blocking(LPC_SPI0, &transfer_setup) > 0) {
		/*transmit sucessful*/
		return true;
	}
	else {
		return false;
	}
}

/* Write data from tx_data_buff to SPI, and read data from SPI to rx_data_buff */
bool SPI::RW_SPI_Data(uint16_t* tx_data_buff, uint16_t* rx_data_buff){

	/* SPI Transfer Setup */
	SPI_DATA_SETUP_T transfer_setup;

	transfer_setup.pTx = tx_data_buff;						/* Transmit Buffer */
	transfer_setup.pRx = rx_data_buff;						/* Receive Buffer */
	transfer_setup.DataSize = 8;							/*size of frame in bits*/
	transfer_setup.Length = 3;								/* Total frames/transmit */

	/* Assert only SSEL0 */
	transfer_setup.ssel = SPI_TXCTL_ASSERT_SSEL0 | SPI_TXCTL_DEASSERT_SSEL1 | SPI_TXCTL_DEASSERT_SSEL2 |
			SPI_TXCTL_DEASSERT_SSEL3;
	transfer_setup.TxCnt = 0;
	transfer_setup.RxCnt = 0;

	if (Chip_SPI_RWFrames_Blocking(LPC_SPI0, &transfer_setup) > 0) {
		/*transmit sucessful*/
		return true;
	}
	else {
		return false;
	}
}

SPI::~SPI() {

}


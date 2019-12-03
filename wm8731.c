
#include "MK64F12.h"
#include "wm8731.h"
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_sai.h"

static struct
{
	uint8_t slave_address;
	rtos_i2c_config_t config;
} wm8731_handle = {0};

/*CALLBACKS ptr*/
static void (*i2s_tx_callback)(void) = 0;
static void (*i2s_rx_callback)(void) = 0;

/* Sai config struct*/
static sai_bit_clock_t sai_tx_clock;
static sai_serial_data_t sai_tx_data;
static sai_fifo_t sai_tx_fifo;
static sai_frame_sync_t sai_tx_frame;
static sai_transceiver_t sai_tx_transceiver;

static sai_bit_clock_t sai_rx_clock;
static sai_serial_data_t sai_rx_data;
static sai_fifo_t sai_rx_fifo;
static sai_frame_sync_t sai_rx_frame;
static sai_transceiver_t sai_rx_transceiver;

/* ISR Handler*/
void I2S0_Tx_IRQHandler(void)
{
	if(i2s_tx_callback)
	{
		i2s_tx_callback();
	}
	NVIC_ClearPendingIRQ(I2S0_Tx_IRQn);
}
void I2S0_Rx_IRQHandler(void)
{
	if(i2s_rx_callback)
	{
		i2s_rx_callback();
	}
	NVIC_ClearPendingIRQ(I2S0_Rx_IRQn);
}

/* CALLBACK*/
void wm8731_tx_callback(void (*handler)(void))
{
	i2s_tx_callback = handler;

	NVIC_EnableIRQ(I2S0_Tx_IRQn);
}
void wm8731_rx_callback(void (*handler)(void))
{
	i2s_rx_callback = handler;

	NVIC_EnableIRQ(I2S0_Rx_IRQn);
}

void rtos_sai_i2s_config (void)
{
	/*
	 * Enable clock for I2S
	 */
	CLOCK_EnableClock(kCLOCK_Sai0);
	/**/
	CLOCK_EnableClock(kCLOCK_PortC);

	/*
	 * Config Mux options
	 */
	/* I2S_0 TXD0*/
	PORTC->PCR[bit_1] = PORT_PCR_MUX(bit_6);
	/* I2S_0 RXD0*/
	PORTC->PCR[bit_5] = PORT_PCR_MUX(bit_4);
	/* I2S_0 RX_BCLK*/
	PORTC->PCR[bit_9] = PORT_PCR_MUX(bit_4);
	/* I2S_0 RX_FS*/
	PORTC->PCR[bit_7] = PORT_PCR_MUX(bit_4);

	/*
	 * Sai config
	 */
	/*
	 * TX
	 */
	/*~~~~~~~~ i2s transmission configuration ~~~~~~~~~~*/

	sai_tx_clock.bclkSrcSwap = TRUE; /* el tx depende del rx */
	sai_tx_clock.bclkInputDelay = FALSE;
	sai_tx_clock.bclkPolarity = kSAI_PolarityActiveLow;
	sai_tx_clock.bclkSource = kSAI_BclkSourceBusclk; /* el bclk que se está recibiendo */

	sai_tx_data.dataOrder = kSAI_DataMSB;
	sai_tx_data.dataWord0Length = bit_32;
	sai_tx_data.dataWordNLength = bit_32;
	sai_tx_data.dataWordLength = bit_32;
	sai_tx_data.dataWordNum = bit_1;
	sai_tx_data.dataMaskedWord = bit_0;
	sai_tx_fifo.fifoWatermark = WATER_MARK_MASK; /* Se deja de recibir datos si se llena el buffer, le dice cada cuanto debe pasar datos al DMA */

	sai_tx_frame.frameSyncEarly = TRUE;
	sai_tx_frame.frameSyncPolarity = kSAI_PolarityActiveHigh;
	sai_tx_frame.frameSyncWidth = bit_1;

	sai_tx_transceiver.serialData = sai_tx_data;
	sai_tx_transceiver.frameSync = sai_tx_frame;
	sai_tx_transceiver.bitClock =  sai_tx_clock;
	sai_tx_transceiver.fifo = sai_tx_fifo;
	sai_tx_transceiver.masterSlave = kSAI_Slave;
	sai_tx_transceiver.syncMode = kSAI_ModeSync;
	sai_tx_transceiver.startChannel = bit_1;
	sai_tx_transceiver.channelMask = bit_1;
	sai_tx_transceiver.endChannel = bit_1;
	sai_tx_transceiver.channelNums = bit_1;
	/* ~~~~~~~~ SAI Tx Functions ~~~~~~~~ */
	SAI_TxSetConfig(I2S0, &sai_tx_transceiver);
	SAI_TxSetBitClockPolarity(I2S0, kSAI_PolarityActiveLow);
	/*
	 * RX
	 */
	/* ~~~~~~~~ i2s reception configuration ~~~~~~~~ */
	sai_rx_clock.bclkSrcSwap = FALSE; /* el rx depende del rx */
	sai_rx_clock.bclkInputDelay = FALSE;
	sai_rx_clock.bclkPolarity = kSAI_PolarityActiveLow;
	sai_rx_clock.bclkSource = kSAI_BclkSourceBusclk; /* el bclk que se está recibiendo */

	sai_rx_data.dataOrder = kSAI_DataMSB;
	sai_rx_data.dataWord0Length = bit_32;
	sai_rx_data.dataWordNLength = bit_32;
	sai_rx_data.dataWordLength = bit_32;
	sai_rx_data.dataWordNum = bit_1;
	sai_rx_data.dataMaskedWord = bit_0;
	sai_rx_fifo.fifoWatermark = WATER_MARK_MASK; /* Se deja de recibir datos si se llena el buffer, le dice cada cuanto debe pasar datos al DMA */

	sai_rx_frame.frameSyncEarly = TRUE;
	sai_rx_frame.frameSyncPolarity = kSAI_PolarityActiveHigh;
	sai_rx_frame.frameSyncWidth = bit_1;

	sai_rx_transceiver.serialData = sai_rx_data;
	sai_rx_transceiver.frameSync = sai_rx_frame;
	sai_rx_transceiver.bitClock =  sai_rx_clock;
	sai_rx_transceiver.fifo = sai_rx_fifo;
	sai_rx_transceiver.masterSlave = kSAI_Slave;
	sai_rx_transceiver.syncMode = kSAI_ModeAsync;
	sai_rx_transceiver.startChannel = bit_1;
	sai_rx_transceiver.channelMask = bit_1;
	sai_rx_transceiver.endChannel = bit_1;
	sai_rx_transceiver.channelNums = bit_1;
	/* ~~~~~~~~ SAI Rx Functions ~~~~~~~~ */
	SAI_RxSetConfig(I2S0, &sai_rx_transceiver);
	SAI_RxSetBitClockPolarity(I2S0, kSAI_PolarityActiveLow);
}

void wm8731_write_register (uint8_t reg, uint16_t data)
{
	uint8_t address;
	uint8_t buffer;

	address = reg << bit_1;
	address = address | (Hi(data) & bit_1);

	buffer = Lo(data);

	rtos_i2c_transfer(
		rtos_i2c_0,
		&buffer,
		bit_1,
		WM8731_DEVICE_ADDRESS,
		address,
		bit_1
	);
}

void wm8731_start(void)
{
	/*
	 * TCSR
	 * Transmit Control SAI Register
	 */
	/* FIFO Reset*/ /* Transmit Enable*/
	I2S0->TCSR |= (I2S_TCSR_FR_MASK);
	I2S0->TCSR |= (I2S_TCSR_TE_MASK);
	/*
	 * Transmit Channel Enable
	 */
	/*
	 * RCSR
	 * Receive Control SAI Register
	 */
	/* FIFO Reset*/ /* Receive Enable*/
	I2S0->RCSR |= (I2S_RCSR_FR_MASK);
	I2S0->RCSR |= (I2S_RCSR_RE_MASK);
	/*
	 * Receive Channel Enable
	 */
}

void wm8732_tx_irq_enable(void)
{
	/*
	 * TCSR
	 * Transmit Control SAI Register
	 */
	I2S0->TCSR |= I2S_TCSR_FRIE_MASK;
}
void wm8732_rx_irq_enable(void)
{
	/*
	 * RCSR
	 * Receive Control SAI Register
	 */
	I2S0->RCSR |= I2S_RCSR_FRIE_MASK;
}
/**********************************************************/
/**********************************************************/
/**********************************************************/
void wm8731_init(uint8_t slave_address, uint8_t mode, uint8_t audio_input, uint8_t sampling_rate, void (*handler_i2s)(void))
{
	wm8731_handle.slave_address = slave_address;

	/* Reset module */
	/*register 0xF = 1111b
	 * 0 to reset*/
	wm8731_write_register(WM8731_REG_RESET, WM8731_RESET);

	/* Left line in settings */
	/*register line in 0 to LEFT input
	 *line in left value default 10111 = 0117h */
	wm8731_write_register(WM8731_REG_LLINE_IN, WM8731_LINE_IN_LEFT);

	/* Rigth line in settings */
	/*register line in 1 to RIGHT input
	 *line in right value default 10111 = 0117h solo volumen*/
	wm8731_write_register(WM8731_REG_RLINE_IN, WM8731_LINE_IN_RIGHT);

	/* Left headphone out settings */
	/*register left headphone output 0010b = 0x02
	 * value for default 1111001b = 0x017C*/
	wm8731_write_register(WM8731_REG_LHPHONE_OUT, WM8731_HP_LEFT);

	/* Right headphone out settings */
	/*register left headphone output 0011b = 0x03
	 * value for default 1111001b = 0x017C*/
	wm8731_write_register(WM8731_REG_RHPHONE_OUT, WM8731_HP_RIGHT);

	/* Analog paths */
	/*register 0100b = 0x04
	 * Line Input Mute to ADC  = 8*/
	wm8731_write_register(WM8731_REG_ANALOG_PATH, WM8731_ANALOG_AUDIO_BYPASS);

	/* Digital paths */
	/*register 0101b = 0x05
	* ADC High Pass Filter Enable  = 0*/
	wm8731_write_register(WM8731_REG_DIGITAL_PATH, WM8731_DIGITAL_AUDIO);

	/* Power down control */
	/*register 0101b = 0x06
	* Line Input Power Down   = 0*/
	wm8731_write_register(WM8731_REG_PDOWN_CTRL, WM8731_POWER_MODE);

	/* Digital interface */
	/*register 0101b = 0x07
	* Line Input Power Down  = 0*/
	wm8731_write_register(WM8731_REG_DIGITAL_IF, WM8731_DB_INTERFACE);

	/* Sampling control */
	/*register 1000b = 0x08
	* CLKOUT divider select divided by 2   = 0*/
	wm8731_write_register(WM8731_REG_SAMPLING_CTRL, WM8731_SAMPLING);

	/* Active control */
	/*register 1001b = 0x09
	* CLKOUT divider select divided by 2   = 0*/
	wm8731_write_register(WM8731_REG_ACTIVE_CTRL, WM8731_ACTIVATE);
}

void wm8731_tx(uint32_t left_channel, uint32_t right_channel)
{
	/*
	 * TDR
	 * Transmit Data Register
	 */
	I2S0->TDR[0] = left_channel;
	//I2S0->TDR[1] = right_channel;
}
void wm8731_rx(uint32_t *left_channel, uint32_t *right_channel)
{
	/*
	 * RDR
	 * Receive Data Register
	 */
	*left_channel = I2S0->RDR[0];
	//*right_channel = I2S0->RDR[1];
}


#include "MK64F12.h"
#include "wm8731.h"
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_sai.h"


static struct
{
	uint8_t slave_address;
} wm8731_handle = {0};

void i2s_config (void)
{
	/*
	 * Enable clock for I2S
	 */
	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_Sai0);

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
	sai_config_t sai_tx_config =
	{
			.protocol = kSAI_BusPCMA,
			.syncMode = kSAI_ModeSync,
			.mclkOutputEnable = FALSE,
			.mclkSource = kSAI_MclkSourceSysclk,
			.masterSlave = kSAI_Slave,
	};
	sai_bit_clock_t sai_tx_clock =
	{
			.bclkSrcSwap = TRUE,
			.bclkInputDelay = FALSE,
			.bclkPolarity = kSAI_PolarityActiveLow,
			.bclkSource = kSAI_BclkSourceBusclk,
	};
	sai_serial_data_t sai_tx_data =
	{
			.dataOrder = kSAI_DataMSB,
			.dataWord0Length = bit_32,
			.dataWordNLength = bit_32,
			.dataWordLength = bit_32,
			.dataWordNum = bit_2,
			.dataMaskedWord = bit_0,
	};
	sai_fifo_t sai_tx_fifo_config =
	{
			.fifoWatermark = bit_1,
	};

	/* Init SAI I2S config*/
	SAI_TxInit(I2S0, &sai_tx_config);
	/* */
	SAI_TxSetBitClockPolarity(I2S0,sai_tx_clock.bclkPolarity);
	/* */
	SAI_TxSetSerialDataConfig(I2S0,&sai_tx_data);
	/* */
	SAI_TxSetFifoConfig(I2S0, &sai_tx_fifo_config);
	/* */
	SAI_TxSetChannelFIFOMask(I2S0, bit_3);

	/*
	 * RX
	 */
	sai_config_t sai_rx_config =
	{
			.protocol = kSAI_BusPCMA,
			.syncMode = kSAI_ModeSync,
			.mclkOutputEnable = FALSE,
			.mclkSource = kSAI_MclkSourceSysclk,
			.masterSlave = kSAI_Slave,
	};
	sai_bit_clock_t sai_rx_clock =
	{
			.bclkSrcSwap = FALSE,
			.bclkInputDelay = FALSE,
			.bclkPolarity = kSAI_PolarityActiveLow,
			.bclkSource = kSAI_BclkSourceBusclk,
	};
	sai_serial_data_t sai_rx_data =
	{
			.dataOrder = kSAI_DataMSB,
			.dataWord0Length = bit_32,
			.dataWordNLength = bit_32,
			.dataWordLength = bit_32,
			.dataWordNum = bit_2,
			.dataMaskedWord = bit_0,
	};
	sai_fifo_t sai_rx_fifo_config =
	{
			.fifoWatermark = bit_0,
	};

	/* Init SAI I2S config*/
	SAI_RxInit(I2S0, &sai_rx_config);
	/* */
	SAI_RxSetBitClockPolarity(I2S0,sai_rx_clock.bclkPolarity);
	/* */
	SAI_RxSetSerialDataConfig(I2S0,&sai_rx_data);
	/* */
	SAI_RxSetFifoConfig(I2S0, &sai_rx_fifo_config);
	/* */
	SAI_RxSetChannelFIFOMask(I2S0, bit_3);
}

void wm8731_write_register (uint8_t reg, uint16_t data)
{
	uint8_t address;
	uint8_t buffer;

	/* delay */
	vTaskDelay(pdMS_TO_TICKS(bit_10));

	address = reg << bit_1;
	address = address | (Hi(data) & bit_1);

	buffer = Lo(data);

	rtos_i2c_transfer(
		rtos_i2c_0,
		&buffer,
		bit_1,
		wm8731_handle.slave_address,
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
	/* FIFO Reset*/
	I2S0->TCSR |= I2S_TCSR_FR_MASK;
	/* Transmit Enable*/
	I2S0->TCSR |= I2S_TCSR_TE_MASK;
	/*
	 * RCSR
	 * Receive Control SAI Register
	 */
	/* FIFO Reset*/
	I2S0->RCSR |= I2S_TCSR_FR_MASK;
	/* Transmit Enable*/
	I2S0->RCSR |= I2S_TCSR_TE_MASK;
}

void wm8732_tx_irq_enable(void)
{
	/*
	 * TCSR
	 * Transmit Control SAI Register
	 */
	I2S0->TCSR |= I2S_TCSR_FRIE_MASK;
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
	wm8731_write_register(WM8731_REG_DIGITAL_IF, WM8731_DA_INTERFACE);

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
	I2S0->TDR[1] = right_channel;
}
void wm8731_rx(uint32_t *left_channel, uint32_t *right_channel)
{
	/*
	 * RDR
	 * Receive Data Register
	 */
	*left_channel = I2S0->RDR[0];
	*right_channel = I2S0->RDR[1];
}

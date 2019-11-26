
#include "MK64F12.h"
#include "wm8731.h"
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_sai.h"

/*CALLBACKS ptr*/
static void (*i2s_callback)(void) = 0;

static struct
{
	uint8_t slave_address;
	rtos_i2c_config_t config;
} wm8731_handle = {0};

static void wm8731_set_callback(void (*callback)(void))
{
	i2s_callback = callback;

	NVIC_EnableIRQ(I2S0_Tx_IRQn);
	NVIC_ClearPendingIRQ(I2S0_Tx_IRQn);
}

void I2S0_Tx_IRQHandler(void)
{
	if(i2s_callback)
	{
		i2s_callback();
	}
}

/*!
 *
 */
static void wm8731_write_register (uint8_t reg, uint16_t data);
/*!
 *
 */
static void i2s_config (void);
/*!
 *
 */
static void wm8731_start(void);
/*!
 *
 */
static void wm8732_tx_irq_enable(void);

static void i2s_config (void)
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

static void wm8731_write_register (uint8_t reg, uint16_t data)
{
	uint8_t address;
	uint8_t buffer;

	/* delay */
	vTaskDelay(pdMS_TO_TICKS(bit_10));

	address = reg << bit_1;
	address = address | (Hi(data) & bit_1);

	buffer = Lo(data);

	rtos_i2c_transfer(
		wm8731_handle.config.i2c_number,
		&buffer,
		bit_1,
		wm8731_handle.slave_address,
		address,
		bit_1
	);
}

static void wm8731_start(void)
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

static void wm8732_tx_irq_enable(void)
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
void wm8731_init(rtos_i2c_config_t config, uint8_t slave_address, uint8_t mode, uint8_t audio_input, uint8_t sampling_rate, void (*handler_i2s)(void))
{
	wm8731_handle.slave_address = slave_address;
	wm8731_handle.config = config;

	rtos_i2c_init(config);

	vTaskDelay(pdMS_TO_TICKS(100));

	/* Reset module */
	wm8731_write_register(WM8731_REG_RESET, WM8731_RESET);

	/* Left line in settings */
	wm8731_write_register(WM8731_REG_LLINE_IN, WM8731_LINE_IN_LEFT);

	/* Rigth line in settings */
	wm8731_write_register(WM8731_REG_RLINE_IN, WM8731_LINE_IN_RIGHT);

	/* Left headphone out settings */
	wm8731_write_register(WM8731_REG_LHPHONE_OUT, WM8731_HP_LEFT);

	/* Right headphone out settings */
	wm8731_write_register(WM8731_REG_RHPHONE_OUT, WM8731_HP_RIGHT);

	/* Analog paths */
	wm8731_write_register(WM8731_REG_ANALOG_PATH, WM8731_ANALOG_AUDIO_BYPASS);

	/* Digital paths */
	wm8731_write_register(WM8731_REG_DIGITAL_PATH, WM8731_DIGITAL_AUDIO);

	/* Power down control */
	wm8731_write_register(WM8731_REG_PDOWN_CTRL, WM8731_POWER_MODE);

	/* Digital interface */
	wm8731_write_register(WM8731_REG_DIGITAL_IF, WM8731_DA_INTERFACE);

	/* Sampling control */
	wm8731_write_register(WM8731_REG_SAMPLING_CTRL, WM8731_SAMPLING);

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

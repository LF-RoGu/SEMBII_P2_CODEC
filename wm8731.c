
#include "MK64F12.h"
#include "wm8731.h"
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_sai.h"

static void WM8731_soft_reset(void);
static void WM8731_config_digital_interface(wm8731_sampling_control_t *config_di);
static void WM8731_config_sampling_control(wm8731_sampling_control_t *config_sc);

static struct
{
	uint8_t slave_address;
	rtos_i2c_config_t config;
} wm8731_handle = {0};

/*CALLBACKS ptr*/
static void (*i2s_tx_callback)(void) = 0;
static void (*i2s_rx_callback)(void) = 0;

/* ISR*/
void I2S0_Tx_IRQHandler(void)
{
	if(i2s_tx_callback)
	{
		i2s_tx_callback();
	}
}
void I2S0_Rx_IRQHandler(void)
{
	if(i2s_rx_callback)
	{
		i2s_rx_callback();
	}
}

/* CALLBACK*/
void wm8731_tx_callback(void (*handler)(void * arg))
{
	i2s_tx_callback = handler;

	NVIC_EnableIRQ(I2S0_Tx_IRQn);
	NVIC_ClearPendingIRQ(I2S0_Tx_IRQn);
}
void wm8731_rx_callback(void (*handler)(void * arg))
{
	i2s_rx_callback = handler;

	NVIC_EnableIRQ(I2S0_Rx_IRQn);
	NVIC_ClearPendingIRQ(I2S0_Rx_IRQn);
}

void rtos_sai_i2s_config (void)
{
	/*  Enable clockgating */
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
	 * TX
	 */
	sai_config_t sai_tx_config =
	{
		.protocol = kSAI_BusPCMB,
		.syncMode = kSAI_ModeAsync,
		.mclkOutputEnable = FALSE,
		.mclkSource = kSAI_MclkSourceSysclk,
		.masterSlave = kSAI_Slave,
	};

	sai_bit_clock_t sai_tx_clock =
	{
		.bclkSrcSwap = TRUE,
		.bclkInputDelay = FALSE,
		.bclkPolarity = kSAI_PolarityActiveHigh,
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
		.protocol = kSAI_BusPCMB,   /*!rising edge syncronization*/
		.syncMode = kSAI_ModeSync,  /*!synchronous mode*/
		.mclkOutputEnable = FALSE,  /*!< Master clock output enable, true means master clock divider enabled */
		//.mclkSource = kSAI_MclkSourceSysclk,
		.masterSlave = kSAI_Slave,
		.bclkSource = kSAI_BclkSourceMclkDiv,

	};

	sai_bit_clock_t sai_rx_clock =
	{
		.bclkSrcSwap = T,
		.bclkInputDelay = FALSE,
		.bclkPolarity = kSAI_PolarityActiveHigh,
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
		.fifoWatermark = bit_1,
	};

	/* Init SAI I2S config*/
	SAI_RxInit(I2S0, &sai_rx_config);

	SAI_RxSetBitClockPolarity(I2S0, sai_rx_clock.bclkPolarity);
	/* */
	SAI_RxSetSerialDataConfig(I2S0, &sai_rx_data);
	/* */
	SAI_RxSetFifoConfig(I2S0, &sai_rx_fifo_config);
	/* */
	SAI_RxSetChannelFIFOMask(I2S0, bit_3);
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
	I2S0->TCSR |= (I2S_TCSR_FR_MASK | I2S_TCSR_TE_MASK);
	/*
	 * RCSR
	 * Receive Control SAI Register
	 */
	/* FIFO Reset*/ /* Transmit Enable*/
	I2S0->RCSR |= (I2S_RCSR_FR_MASK | I2S_RCSR_RE_MASK);
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
	wm8731_sampling_control_t config_sc =
	{
		NORMAL_MODE,      /* the MCLK is set up according to the desire sample rate */
		fs_256,			  /* rate which digital signal processing is carried out at  (256 frames per second)*/
		SC_ADC_DAC_48KHZ  /* set ADC's & DAC's sampling rate @48 kHZ */
	};


	wm8731_handle.slave_address = slave_address;

	/* generate a soft reset */
	W8731_soft_reset();

	/* configure left channel line input*/
	WM8731_config_left_channel_line_input();

	/* configure right channel line input*/
	WM8731_config_right_channel_line_input();

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
	//wm8731_write_register(WM8731_REG_ANALOG_PATH, WM8731_ANALOG_AUDIO_BYPASS);

	/* configure digital interface */
	wm8731_write_register(WM8731_REG_DIGITAL_PATH, WM8731_DIGITAL_INT_CONF);
	//WM8731_config_digital_interface(wm8731_sampling_control_t *config_di)

	/* Power down control */
	/*register 0101b = 0x06
	* Line Input Power Down   = 0*/
	wm8731_write_register(WM8731_REG_PDOWN_CTRL, WM8731_POWER_MODE);

	/* Digital interface */
	/*register 0101b = 0x07
	* Line Input Power Down  = 0*/
	wm8731_write_register(WM8731_REG_DIGITAL_IF, WM8731_DB_INTERFACE);

	/* Sampling control configuration */
	WM8731_config_sampling_control(config_sc);

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

static void WM8731_soft_reset(void)
{
	wm8731_write_register(WM8731_REG_RESET, WM8731_RESET);
}

static void WM8731_config_left_channel_line_input(void)
{
	wm8731_write_register(WM8731_REG_LLINE_IN, WM8731_LINE_IN_LEFT);
}

static void WM8731_config_right_channel_line_input(void)
{
	wm8731_write_register(WM8731_REG_LLINE_IN, WM8731_LINE_IN_RIGHT);
}

static void WM8731_config_sampling_control(wm8731_sampling_control_t *config_sc)
{
	uint16_t data;

//	data |= (config_sc->mode_select << 0);
//	data |= (config_sc->base_over_sampling_rate << 1);
//	data |= config_sc->adc_dac_sampling_rate;

	/* check */

	wm8731_write_register(WM8731_REG_SAMPLING_CTRL, data);
}


static void WM8731_config_digital_interface(wm8731_digital_interface_t *config_di)
{

	wm8731_write_register(WM8731_REG_DIGITAL_PATH, WM8731_DIGITAL_INT_CONF);
}

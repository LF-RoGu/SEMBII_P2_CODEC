/*!
 * \file      audio.h
 * \brief     Implements the audio configuration
 * \copyright ARM University Program &copy; ARM Ltd 2014.
 *
 */

#ifndef wm8731_H
#define wm8731_H

#include "MK64F12.h"
#include "rtos_i2c.h"
#include "stdint.h"
#include "bits.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "fsl_port.h"

/* The address of the codec is set by the state of the CSB pin */
#define WM8731_DEVICE_ADDRESS   0x1a    // 0011_010

/* WM8731 sound chip register addresses */
#define WM8731_ADDRESS				0x34 // WM8731 chip address on I2C bus
#define WM8731_REG_LLINE_IN			0x00 // Left Channel Line Input Volume Control
#define WM8731_REG_RLINE_IN			0x01 // Right Channel Line Input Volume Control
#define WM8731_REG_LHPHONE_OUT		0x02 // Left Channel Headphone Output Volume Control
#define WM8731_REG_RHPHONE_OUT		0x03 // Right Channel Headphone Output Volume Control
#define WM8731_REG_ANALOG_PATH		0x04 // Analog Audio Path Control
#define WM8731_REG_DIGITAL_PATH		0x05 // Digital Audio Path Control
#define WM8731_REG_PDOWN_CTRL		0x06 // Power Down Control Register
#define WM8731_REG_DIGITAL_IF		0x07 // Digital Audio Interface Format
#define WM8731_REG_SAMPLING_CTRL	0x08 // Sampling Control Register
#define WM8731_REG_ACTIVE_CTRL		0x09 // Active Control
#define WM8731_REG_RESET			0x0F // Reset register

/* WM8731 sound chip constants (for default set up) */
#define WM8731_RESET				0x0000 // Reset value
#define WM8731_LINE_IN_LEFT			0x0117 // LLI settings: Enable simultaneous load to left and right channels, Vol 0 dB
#define WM8731_LINE_IN_RIGHT		0x0117 // RLI settings: Enable simultaneous load to left and right channels, Vol 0 dB
#define WM8731_HP_LEFT				0x017C // Headphone settings : -9dB output, Enable simultaneous load to left and right channels
#define WM8731_HP_RIGHT				0x017C // Headphone settings : -9dB output, Enable simultaneous load to left and right channels
#define WM8731_ANALOG_AUDIO_BYPASS	0x0008 // Bypass Line In
#define WM8731_ANALOG_AUDIO_LINE	0x0011 // Line In -> ADC
#define WM8731_DIGITAL_AUDIO		0x0000
#define WM8731_POWER_MODE			0x0000 // Disable Power down
#define WM8731_DA_INTERFACE			0x0053 // Enable Master Mode and 32bit data
#define WM8731_DB_INTERFACE			0x0043 // Enable Master Mode and 32bit data
#define WM8731_SAMPLING				0x0000 // 48kHz, MCLK=12.288MHz
#define WM8731_ACTIVATE				0x0001 // Module is ON
#define WM8731_DEACTIVATE			0x0000 // Module is OFF


#define SC_ADC_DAC_48KHZ            0X00

#define WM8731_DIGITAL_INT_CONF     0x5A

#define Lo(param) ((char *)&param)[0]

#define Hi(param) ((char *)&param)[1]


typedef enum
{
	NORMAL_MODE,
	USB_MODE
} sc_mode_select_t;

typedef enum
{
	fs_256 = 0,
	fs_128 = 0,
	fs_384 = 1,
	fs_192 = 1
} sc_bosr_t;

typedef struct
{
  sc_mode_select_t mode_select;      /* the MCLK is set up according to the desire sample rate */
  sc_bosr_t base_over_sampling_rate; /* rate which digital signal processing is carried out at  (256 frames per second)*/
  uint8_t adc_dac_sampling_rate;     /* set ADC's & DAC's sampling rate @48 kHZ */
} wm8731_sampling_control_t;

typedef enum
{
	RIGHT_JUSTIFIED,
	LEFT_JUSTIFIED,
	I2S_FORMAT,
	DSP_MODE
} di_format_t;

typedef enum
{
	wl_16bit,
	wl_20bit,
	wl_24bit,
	wl_32bit
} di_word_length_t;

typedef enum
{
	LRC_HIGH_CH_RIGHT,
	LRC_LOW_CH_RIGHT
} di_phase_control_t;

typedef enum
{
	SLAVE_MODE,
	MASTER_MODE
} di_master_slave_mode_t;

typedef enum
{
	noclkinversion,
	clkinversion
} di_bit_clk_inv_t;

typedef struct
{
	di_format_t FORMAT;
	di_word_length_t IWL;
	di_phase_control_t LRP;
	di_master_slave_mode_t MS;
	di_bit_clk_inv_t BCLKINV;
} wm8731_digital_interface_t;


/*!
 *
 */
void wm8731_init(uint8_t slave_address, uint8_t mode, uint8_t audio_input, uint8_t sampling_rate, void (*handler_i2s)(void));
/*!
 *
 */
void wm8731_tx(uint32_t left_channel, uint32_t right_channel);
/*!
 *
 */
void wm8731_rx(uint32_t *left_channel, uint32_t *right_channel);
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*!
 *
 */
void wm8731_write_register (uint8_t reg, uint16_t data);
/*!
 *
 */
void rtos_sai_i2s_config (void);
/*!
 *
 */
void wm8731_start(void);
/*!
 *
 */
void wm8732_tx_irq_enable(void);
/*!
 *
 */
void wm8732_rx_irq_enable(void);
/*
 * CALLBACK
 */
/*!
 *
 */
void wm8731_rx_callback(void (*handler)(void * arg));
/*!
 *
 */
void wm8731_tx_callback(void (*handler)(void * arg));

#endif // wm8731_H


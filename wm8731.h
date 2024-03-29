/*!
	\file 	  wm8731.h
	\brief    This is the header file that contains the driver for the WM8731 Audio Codec, implemented
			  with Direct Memory Access.
	\authors: Luís F. Rodríguez @LProtox
			  César Villarreal  @4497cv
			  Fernanda Muñoz    @Fernmu
	\date	  06/12/2019
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
/**/
#define WATER_MARK_MASK (1U)

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
#define WM8731_ANALOG_AUDIO_BYPASS	0x0010 // Bypass Line In
#define WM8731_ANALOG_AUDIO_LINE	0x0011 // Line In -> ADC
#define WM8731_DIGITAL_AUDIO		0x0000
#define WM8731_POWER_MODE			0x0000 // Disable Power down
#define WM8731_DA_INTERFACE			0x0053 // Enable Master Mode and 32bit data
#define WM8731_DB_INTERFACE			0x0043 // Enable Master Mode and 16bit data
#define WM8731_SAMPLING				0x0000 // 48kHz, MCLK=12.288MHz
#define WM8731_ACTIVATE				0x0001 // Module is ON
#define WM8731_DEACTIVATE			0x0000 // Module is OFF

#define Lo(param) ((char *)&param)[0]

#define Hi(param) ((char *)&param)[1]

/*!
 	 \brief		 This function intialiazes the WM8731 device
 	 \param[in]  uint8_t,uint8_t,uint8_t,uint8_t,void
 	 \return     void
 */
void wm8731_init(uint8_t slave_address, uint8_t mode, uint8_t audio_input, uint8_t sampling_rate, void (*handler_i2s)(void));

/*!
 	 \brief		 This function configures the transmission of the WM8731.
 	 \param[in]  uint8_t,uint8_t,uint8_t,uint8_t,void
 	 \return     void
 */
void wm8731_tx(uint32_t left_channel, uint32_t right_channel);

/*!
 	 \brief		 This function configures the reception of the WM8731.
 	 \param[in]  uint32_t,uint32_t
 	 \return     void
 */
void wm8731_rx(uint32_t *left_channel, uint32_t *right_channel);

/*!
 	 \brief		 This function writes a command to configure the WM8731 
 	 \param[in]  uint8_t,uint8_t
 	 \return     void
 */
void wm8731_write_register (uint8_t reg, uint16_t data);

/*!
 	 \brief		 This function configures the i2s Tx/Rx
 	 \param[in]  void
 	 \return     void
 */
void rtos_sai_i2s_config (void);

/*!
 	 \brief		 This function starts the WM8731 device.
 	 \param[in]  void
 	 \return     void
 */
void wm8731_start(void);

/*!
 	 \brief		 This function configures the reception of the WM8731.
 	 \param[in]  void
 	 \return     void
 */
void wm8732_tx_irq_enable(void);

/*!
 	 \brief		 This function enables interrupts for reception
 	 \param[in]  void
 	 \return     void
 */
void wm8732_rx_irq_enable(void);

/*!
 	 \brief		 Rx handler function
 	 \param[in]  int8_t
 	 \return     void
 */
void wm8731_rx_callback(void (*handler)(void));

/*!
 	 \brief		 Tx handler function
 	 \param[in]  void (*handler)(void))
 	 \return     void
 */
void wm8731_tx_callback(void (*handler)(void));

/*!
 	 \brief		 This function initializes the DMA
 	 \param[in]  void
 	 \return     void
 */
void dma_init(void);

/*!
 	 \brief		 This function transmits and waits until information is transferred using DMA
 	 \param[in]  void
 	 \return     void
 */
void set_dma_transfer(void);
#endif // wm8731_H


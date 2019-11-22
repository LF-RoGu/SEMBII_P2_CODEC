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
#define WM8731_SAMPLING				0x0000 // 48kHz, MCLK=12.288MHz
#define WM8731_ACTIVATE				0x0001 // Module is ON
#define WM8731_DEACTIVATE			0x0000 // Module is OFF

#define Lo(param) ((char *)&param)[0]

#define Hi(param) ((char *)&param)[1]

void wm8731_init(rtos_i2c_config_t config, uint8_t slave_address, uint8_t mode, uint8_t audio_input, uint8_t sampling_rate, void (*handler_i2s)(void));

#endif // wm8731_H


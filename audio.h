
#ifndef audio_H
#define audio_H

#define ARM_MATH_CM4

#include "MK64F12.h"
#include "stdint.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "fsl_port.h"
#include "rtos_i2c.h"
#include "wm8731.h"

/** This enum describes the different input setup of the CODEC */
typedef enum {
	AUDIO_INPUT_LINE = 0x11,  // Select DAC input Line In and disable bypass
	AUDIO_INPUT_MIC = 0x15,   // Select DAC input Microphone In and disable bypass
} audio_input;

/** This enum describes the different sampling frequency setup of the CODEC */
typedef enum {
    FS_8000_HZ = 0x0C,   // 8kHz from 12.288MHz MCLK
    FS_32000_HZ = 0x18,  // 32kHz from 12.288MHz MCLK
    FS_48000_HZ = 0x00,  // 48kHz from 12.288MHz MCLK
    FS_96000_HZ = 0x1C,  // 96kHz from 12.288MHz MCLK
} sampling_rate;

/*! This enum describes the different input setup of the CODEC */
typedef enum {
    INTR = 0x00,  // User I2S interruptions to controll the flow of the program (generate interruption when  irq_depth=<FIFO level)
	DMA = 0x01,   // Use DMA requests to controll the flow of the program (generat a request whem dma_depth=<FIFO level)
} mode;
/*
 *  Semaphore
 */

static struct
{
	SemaphoreHandle_t init_end;
	SemaphoreHandle_t rx_end;
	SemaphoreHandle_t tx_end;
} audio_handle = {0};


/*!
 *
 */
void audio_config (void * arg);
/*!
 *
 */
void audio_bypass(void);
/*!
 *
 */
void audio_bypass_receive(void * arg);
/*!
 *
 */
void audio_bypass_send(void * arg);

#endif // audio_H



#include "MK64F12.h"
#include "audio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "fsl_port.h"
#include "rtos_i2c.h"
#include "wm8731.h"

rtos_i2c_config_t i2c_config;

static struct
{
	SemaphoreHandle_t init_end;
} audio_handle = {0};

/*!
 *
 */
static void audio_config (void * arg);
/*!
 *
 */
static void audio_get_data(void * arg);
/**********************************************************/
/**********************************************************/
/**********************************************************/

static void audio_config (void * arg)
{
	wm8731_init(i2c_config, WM8731_DEVICE_ADDRESS, INTR, AUDIO_INPUT_LINE, FS_48000_HZ, NULL);

	while(1)
	{
		xSemaphoreGive(audio_handle.init_end);

		vTaskDelay(portMAX_DELAY);
	}
}

static void audio_get_data (void * arg)
{
	xSemaphoreTake(audio_handle.init_end, portMAX_DELAY);

	while (1)
	{

	}
}

/**********************************************************/
/**********************************************************/
/**********************************************************/

void audio_init (void)
{
	i2c_config.SCL_pin = bit_24;
	i2c_config.SDA_pin = bit_25;
	i2c_config.baudrate = 100000;
	i2c_config.i2c_number = rtos_i2c_0;
	i2c_config.pin_mux = kPORT_MuxAlt5;
	i2c_config.port = rtos_i2c_portE;

    audio_handle.init_end = xSemaphoreCreateBinary();

    xTaskCreate(audio_config, "audio_config", 4*configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(audio_get_data, "audio_get_data", 4*configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-2, NULL);

    vTaskStartScheduler();

	while(1)
	{

	}
}



#include "MK64F12.h"
#include "audio.h"

uint32_t data_left;
uint32_t data_right;

/**********************************************************/
/**********************************************************/
/**********************************************************/

void audio_config (void * arg)
{
	while(1)
	{
		/* */
		wm8731_init(WM8731_DEVICE_ADDRESS, INTR, AUDIO_INPUT_LINE, FS_48000_HZ, NULL);
		/* */
		wm8732_tx_irq_enable();
		/* */
		wm8731_start();
		/* Semaphore*/
		xSemaphoreGive(audio_handle.init_end);
		/*Kill task*/
		vTaskSuspend(NULL);
	}
}

/**********************************************************/
/**********************************************************/
/**********************************************************/

void audio_bypass_receive(void)
{
	while(1)
	{
		xSemaphoreTake(audio_handle.init_end, portMAX_DELAY);
		vTaskDelay(pdMS_TO_TICKS(100));
		wm8731_rx(&data_left, &data_right);
		vTaskDelay(pdMS_TO_TICKS(100));
		xSemaphoreGive(audio_handle.init_end);
	}
}
void audio_bypass_send(void)
{
	while(1)
	{
		xSemaphoreTake(audio_handle.init_end, portMAX_DELAY);
		vTaskDelay(pdMS_TO_TICKS(100));
		wm8731_tx(data_left, data_right);
		vTaskDelay(pdMS_TO_TICKS(100));
		xSemaphoreGive(audio_handle.init_end);
	}
}

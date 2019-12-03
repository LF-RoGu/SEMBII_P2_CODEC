

#include "MK64F12.h"
#include "audio.h"

/**********************************************************/
/**********************************************************/
/**********************************************************/

void audio_config (void * arg)
{
    /* I2S*/
    rtos_sai_i2s_config();

    /* */
    wm8731_init(WM8731_DEVICE_ADDRESS, INTR, AUDIO_INPUT_LINE, FS_48000_HZ, NULL);

	wm8732_rx_irq_enable();
    wm8732_tx_irq_enable();

	/* */
	wm8731_rx_callback(audio_bypass);
	/* */
	wm8731_tx_callback(audio_bypass);

    wm8731_start();

	while(1)
	{
		/*Kill task*/
		vTaskSuspend(NULL);
	}
}

/**********************************************************/
/**********************************************************/
/**********************************************************/
void audio_bypass(void)
{
	volatile uint32_t data_left;
	volatile uint32_t data_right;


	wm8731_rx(&data_left, &data_right);

	wm8731_tx(data_left, data_right);
}

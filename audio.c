

#include "MK64F12.h"
#include "audio.h"

uint32_t data_left;
uint32_t data_right;

/**********************************************************/
/**********************************************************/
/**********************************************************/

void audio_config (void * arg)
{
    /* WM8731 initialization: */
    wm8731_init(WM8731_DEVICE_ADDRESS,
    		    INTR, AUDIO_INPUT_LINE,
				FS_48000_HZ,
				NULL);
	/* */
    wm8731_tx_callback(audio_bypass_send);

	/* */
	wm8731_rx_callback(audio_bypass_receive);

	wm8732_tx_irq_enable();
    NVIC_EnableIRQ(I2S0_Tx_IRQn);
    NVIC_SetPriority(I2S0_Tx_IRQn, bit_4);

	wm8732_rx_irq_enable();
	NVIC_EnableIRQ(I2S0_Rx_IRQn);
	NVIC_SetPriority(I2S0_Rx_IRQn, bit_4);

	/* */
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

void audio_bypass_receive(void * arg)
{
	wm8731_rx(&data_left, &data_right);
}

void audio_bypass_send(void * arg)
{
	wm8731_tx(data_left, data_right);
}

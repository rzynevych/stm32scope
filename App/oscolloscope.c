/*
 * oscolloscope.c
 *
 *  Created on: 9 мая 2026 г.
 *      Author: rusla
 */

#include "oscilloscope.h"

#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "tim.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

uint16_t _adc_buf[ADC_BUF_SIZE];
uint8_t  _msg_buf[MSG_SIZE];

uint32_t _duration = 1000;
uint16_t _data_length = 1000;

volatile uint8_t _half_lock = 0;
volatile uint8_t _full_lock = 0;

static osThreadId_t main_task_handle;

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (_half_lock) {
		return;
	}
	uint8_t *adc_buf_ptr = (uint8_t *) _adc_buf;
	for (int i = 0; i < ADC_BUF_SIZE / 2; ++i) {
		_msg_buf[i + HEADER_SIZE + METADATA_SIZE] = adc_buf_ptr[i * 2];
	}
	_half_lock = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (_full_lock || !_half_lock) {
		return;
	}
	uint8_t *adc_buf_ptr = (uint8_t *) _adc_buf;
	for (int i = ADC_BUF_SIZE / 2; i < ADC_BUF_SIZE; ++i) {
		_msg_buf[i + HEADER_SIZE + METADATA_SIZE] = adc_buf_ptr[i * 2];
	}
    _full_lock = 1;
}

void cdc_tratsmit_cplt_callback()
{
	_half_lock = 0;
	_full_lock = 0;
}

void main_task(void *argument)
{
	MX_USB_DEVICE_Init();

	// Callback for buff locks releasing
	g_cdc_tratsmit_cplt_callback = cdc_tratsmit_cplt_callback;

	// Start ADC DMA and timer
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)_adc_buf, ADC_BUF_SIZE);
	HAL_TIM_Base_Start(&htim2);

	_msg_buf[0] = HEADER_BYTE1;
	_msg_buf[1] = HEADER_BYTE2;

	memcpy(_msg_buf + HEADER_SIZE, &_duration, DURATION_SIZE);
	memcpy(_msg_buf + HEADER_SIZE + DURATION_SIZE, &_data_length, DATA_LENGTH_SIZE);

	osDelay(1000);

	uint32_t last_time = 0;
	for(;;)
	{
		if (g_usb_ready && _full_lock) {
			g_usb_ready = 0;
			CDC_Transmit_FS((uint8_t*)_msg_buf, MSG_SIZE);
		}

		uint32_t time = HAL_GetTick();
		if (time - last_time > 100) {
			last_time = time;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		}
		osDelay(0);
	}
}

void main_task_init()
{
	const osThreadAttr_t main_task_attributes = {
		.name = "main_task",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
	};

	main_task_handle = osThreadNew(main_task, NULL, &main_task_attributes);
}

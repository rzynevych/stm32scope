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

#define HEADER_SIZE 2
#define MSG_TYPE_SIZE 1
#define SAMPLING_PERIOD_SIZE 2
#define DATA_LENGTH_SIZE 2
#define METADATA_SIZE (SAMPLING_PERIOD_SIZE + DATA_LENGTH_SIZE)
#define MSG_SAMPLES_COUNT 1000
#define DATA_MSG_HEADER_FULL_SIZE (HEADER_SIZE + MSG_TYPE_SIZE + METADATA_SIZE)
#define DATA_MSG_SIZE (DATA_MSG_HEADER_FULL_SIZE + MSG_SAMPLES_COUNT)

#define ADC_BUF_SIZE (MSG_SAMPLES_COUNT * 10)

#define TRIGGER_MODE_SIZE 1
#define TRIGGER_VAL_SIZE 2
#define TRIGGER_MODE_MASK 0x0F
#define TRIGGER_DIRECTION_BIT_POS 7
#define SETTING_MSG_SIZE (HEADER_SIZE + MSG_TYPE_SIZE + TRIGGER_MODE_SIZE + TRIGGER_VAL_SIZE)

#define MSG_IN_BUFF_SIZE CDC_DATA_FS_MAX_PACKET_SIZE

#define HEADER_BYTE1 0xAA
#define HEADER_BYTE2 0xBB

#define ADC_MAX_VOLTAGE_MV 3300
#define AUTO_TRIGGER_PERIOD_MS 100

enum MsgType {
	MSG_TYPE_DATA,
	MSG_TYPE_SETTING
};

enum TriggerMode {
	TRIGGER_MODE_AUTO,
	TRIGGER_MODE_NORMAL,
	TRIGGER_MODE_SINGLE
};

enum TriggerDirection {
	TRIGGER_DIRECTION_RISING = 1,
	TRIGGER_DIRECTION_FALLING = -1
};

static uint16_t _adc_buf[ADC_BUF_SIZE];
static uint8_t _samples_buf[ADC_BUF_SIZE];
static uint8_t _data_msg_buf[DATA_MSG_SIZE];
static uint8_t _setting_msg_buf[SETTING_MSG_SIZE];
static uint8_t _msg_in_buf[MSG_IN_BUFF_SIZE];

static uint32_t _sampling_period = 1000; // in nanoseconds
static uint16_t _data_length = MSG_SAMPLES_COUNT;

static volatile uint8_t _half_lock = 0;
static volatile uint8_t _full_lock = 0;

static uint8_t  _trigger_mode = TRIGGER_MODE_AUTO;
static int8_t   _trigger_direction = TRIGGER_DIRECTION_RISING;
static int16_t  _trigger_val_mv = 0;
static uint32_t _trigger_position = 0;
static uint8_t 	_trigger_pause = 0;
static uint32_t _last_trigger_time = 0;

static osThreadId_t _main_task_handle;
static osThreadId_t _usb_in_task_handle;

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (_half_lock || _trigger_pause) {
		return;
	}
	uint8_t  trigger_ready = 0;
	uint8_t trigger_val = 255 * (_trigger_val_mv + ADC_MAX_VOLTAGE_MV/2) / ADC_MAX_VOLTAGE_MV;
	int min_trigger = MSG_SAMPLES_COUNT/2;
	for (int i = 0; i < ADC_BUF_SIZE / 2; ++i) {
		uint8_t val = _adc_buf[i];
		_samples_buf[i] = val;
		if (i >= min_trigger &&_trigger_direction * val < _trigger_direction * trigger_val) {
			trigger_ready = 1;
		}
		if (!_trigger_position && trigger_ready && i >= min_trigger
				&& _trigger_direction * val >= _trigger_direction * trigger_val) {
			_trigger_position = i;
		}
	}
	_half_lock = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (_full_lock || !_half_lock) {
		return;
	}
	uint8_t  trigger_ready = 0;
	uint8_t trigger_val = 255 * (_trigger_val_mv + ADC_MAX_VOLTAGE_MV/2) / ADC_MAX_VOLTAGE_MV;
	int max_trigger = ADC_BUF_SIZE - MSG_SAMPLES_COUNT/2;
	for (int i = ADC_BUF_SIZE / 2; i < ADC_BUF_SIZE; ++i) {
		uint8_t val = _adc_buf[i];
		_samples_buf[i] = val;
		if (i < max_trigger &&_trigger_direction * val < _trigger_direction * trigger_val) {
			trigger_ready = 1;
		}
		if (!_trigger_position && trigger_ready && i < max_trigger
				&& _trigger_direction * val >= _trigger_direction * trigger_val) {
			_trigger_position = i;
		}
	}
	if (_trigger_position) {
		_full_lock = 1;
	} else {
		_half_lock = 0;
	}
}

void cdc_transmit_cplt_callback()
{
	_half_lock = 0;
	_full_lock = 0;
}

uint8_t send_usb_msg(uint8_t* buf, uint16_t len)
{
	g_usb_ready = 0;
	return CDC_Transmit_FS(buf, len);
}

void main_task(void *argument)
{
	MX_USB_DEVICE_Init();

	// Callback for buff locks releasing
	g_cdc_tratsmit_cplt_callback = cdc_transmit_cplt_callback;

	// Start ADC DMA and timer
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)_adc_buf, ADC_BUF_SIZE);
	HAL_TIM_Base_Start(&htim2);

	_data_msg_buf[0] = HEADER_BYTE1;
	_data_msg_buf[1] = HEADER_BYTE2;
	_data_msg_buf[2] = MSG_TYPE_DATA;

	uint32_t offset = HEADER_SIZE + MSG_TYPE_SIZE;
	memcpy(_data_msg_buf + offset, &_sampling_period, SAMPLING_PERIOD_SIZE);
	offset += SAMPLING_PERIOD_SIZE;
	memcpy(_data_msg_buf + offset, &_data_length, DATA_LENGTH_SIZE);

	osDelay(1000); // wait for USB connection

	uint32_t last_time = 0;
	for(;;)
	{
		uint32_t time = HAL_GetTick();
		if (g_usb_ready && _full_lock) {
			if (_trigger_mode == TRIGGER_MODE_SINGLE) {
				_trigger_pause = 1;
			}
			memcpy(_data_msg_buf + DATA_MSG_HEADER_FULL_SIZE, _samples_buf + _trigger_position - MSG_SAMPLES_COUNT/2, MSG_SAMPLES_COUNT);
			_trigger_position = 0;
			_last_trigger_time = time;
			send_usb_msg((uint8_t*)_data_msg_buf, DATA_MSG_SIZE);
		}
		if (_trigger_mode == TRIGGER_MODE_AUTO && time - _last_trigger_time > AUTO_TRIGGER_PERIOD_MS) {
			_trigger_position = ADC_BUF_SIZE / 2;
		}
		if (time - last_time > 100) {
			last_time = time;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		}
		osDelay(0);
	}
}

void handle_in_message()
{
	if (_msg_in_buf[0] != HEADER_BYTE1 || _msg_in_buf[1] != HEADER_BYTE2) {
		return;
	}
	size_t offset = HEADER_SIZE;
	_trigger_mode = _msg_in_buf[offset] & TRIGGER_MODE_MASK;
	_trigger_direction = ((_msg_in_buf[offset] & 0x80) >> TRIGGER_DIRECTION_BIT_POS) ? TRIGGER_DIRECTION_FALLING : TRIGGER_DIRECTION_RISING;
	offset += TRIGGER_MODE_SIZE;
	memcpy(&_trigger_val_mv, _msg_in_buf + offset, TRIGGER_VAL_SIZE);

	_trigger_pause = 0;

	offset = HEADER_SIZE + MSG_TYPE_SIZE;
	_setting_msg_buf[offset] = _trigger_mode;
	offset += TRIGGER_MODE_SIZE;
	memcpy(_setting_msg_buf + offset, &_trigger_val_mv, TRIGGER_VAL_SIZE);

	while (!g_usb_ready) {
		osDelay(1);
	}
	send_usb_msg((uint8_t*)_setting_msg_buf, SETTING_MSG_SIZE);
}

void usb_in_task(void *argument)
{
	g_usb_in_buff = _msg_in_buf;
	_setting_msg_buf[0] = HEADER_BYTE1;
	_setting_msg_buf[1] = HEADER_BYTE2;
	_setting_msg_buf[2] = MSG_TYPE_SETTING;
	for(;;)
	{
		if (g_usb_data_received) {
			handle_in_message();
			g_usb_data_received = 0;
		}
		osDelay(1);
	}
}

void main_task_init()
{
	const osThreadAttr_t main_task_attributes = {
		.name = "main_task",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
	};

	_main_task_handle = osThreadNew(main_task, NULL, &main_task_attributes);
}

void usb_in_task_init()
{
	const osThreadAttr_t usb_in_task_attributes = {
		.name = "usb_in_task",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
	};

	_usb_in_task_handle = osThreadNew(usb_in_task, NULL, &usb_in_task_attributes);
}

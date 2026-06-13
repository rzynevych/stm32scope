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
#define SAMPLE_SIZE 2
#define DATA_MSG_HEADER_FULL_SIZE (HEADER_SIZE + MSG_TYPE_SIZE + METADATA_SIZE)
#define DATA_MSG_SIZE (DATA_MSG_HEADER_FULL_SIZE + MSG_SAMPLES_COUNT * SAMPLE_SIZE)

#define ADC_BUF_SIZE MSG_SAMPLES_COUNT
#define SUBBUFFER_SIZE (MSG_SAMPLES_COUNT/2)
#define SUBBUFFER_COUNT 4
#define SAMPLES_BUFF_SIZE (SUBBUFFER_SIZE * SUBBUFFER_COUNT)


#define TRIGGER_MODE_SIZE 1
#define TRIGGER_VAL_SIZE 2
#define TRIGGER_MODE_MASK 0x0F
#define TRIGGER_DIRECTION_BIT_POS 7
#define SETTING_MSG_SIZE (HEADER_SIZE + MSG_TYPE_SIZE + TRIGGER_MODE_SIZE + TRIGGER_VAL_SIZE)

#define MSG_IN_BUFF_SIZE CDC_DATA_FS_MAX_PACKET_SIZE

#define HEADER_BYTE1 0xAA
#define HEADER_BYTE2 0xBB

#define ADC_MAX_VOLTAGE_MV 3300
#define ADC_MID_VOLTAGE_MV (ADC_MAX_VOLTAGE_MV/2)
#define ATTENUATION_FACTOR 0.4
#define ADC_MAX_INT_VAL 4095
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
static uint16_t _samples_buf[SAMPLES_BUFF_SIZE];
static uint8_t _data_msg_buf[DATA_MSG_SIZE + 4];
static uint8_t _setting_msg_buf[SETTING_MSG_SIZE];
static uint8_t _msg_in_buf[MSG_IN_BUFF_SIZE];

static uint32_t _sampling_period_ns = 1000; // in nanoseconds
static uint16_t _data_length = MSG_SAMPLES_COUNT * SAMPLE_SIZE;

static volatile uint32_t _last_written_subbuff = 0;

static uint8_t  _trigger_mode = TRIGGER_MODE_AUTO;
static int8_t   _trigger_direction = TRIGGER_DIRECTION_RISING;
static int16_t  _trigger_val_mv = 0;
static uint32_t _trigger_position = 0;
static uint8_t 	_trigger_pause = 0;
static uint32_t _last_trigger_time = 0;
static uint32_t _current_time = 0;
static uint32_t _trigger_search_start = 0;
static uint32_t _trigger_search_time = 0;


static osThreadId_t _main_task_handle;
static osThreadId_t _usb_in_task_handle;

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint32_t new_subbuff = (_last_written_subbuff + 1) % SUBBUFFER_COUNT;
	memcpy(_samples_buf + SUBBUFFER_SIZE * new_subbuff, _adc_buf, SUBBUFFER_SIZE * sizeof(uint16_t));
	_last_written_subbuff = new_subbuff;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint32_t new_subbuff = (_last_written_subbuff + 1) % SUBBUFFER_COUNT;
	memcpy(_samples_buf + SUBBUFFER_SIZE * new_subbuff, _adc_buf + SUBBUFFER_SIZE, SUBBUFFER_SIZE * sizeof(uint16_t));
	_last_written_subbuff = new_subbuff;
}

void cdc_transmit_cplt_callback()
{

}

uint8_t send_usb_msg(uint8_t* buf, uint16_t len)
{
	g_usb_ready = 0;
	uint8_t ret = CDC_Transmit_FS(buf, len);
	return ret;
}

static inline uint32_t subbuff_distance(uint32_t front, uint32_t back)
{
	int32_t dist = front - back;
	if (dist < 0) {
		dist += SUBBUFFER_COUNT;
	}
	return dist;
}

static void search_trigger()
{
	if (_trigger_pause || !g_usb_ready) {
		return;
	}

	static uint32_t current_subbuff = 2;
	uint32_t dist = subbuff_distance(_last_written_subbuff, current_subbuff);
	while (dist == 0) {
		osDelay(0);
		dist = subbuff_distance(_last_written_subbuff, current_subbuff);
	}
	if (dist > 1) {
		current_subbuff = (current_subbuff + dist - 1) % SUBBUFFER_COUNT;
	}

	_trigger_search_start = TIM5->CNT;

	uint8_t  trigger_ready = 0;
	int16_t trigger_int_val = (int32_t)_trigger_direction * ADC_MAX_INT_VAL * (ADC_MID_VOLTAGE_MV + _trigger_val_mv)/ADC_MAX_VOLTAGE_MV;
	for (int i = current_subbuff * SUBBUFFER_SIZE; i < (current_subbuff + 1) * SUBBUFFER_SIZE; ++i) {
		int16_t val = _trigger_direction * _samples_buf[i];
		if (val < trigger_int_val) {
			trigger_ready = 1;
		}
		if (trigger_ready && _trigger_position == UINT32_MAX && val >= trigger_int_val) {
			_trigger_position = i;
			break;
		}
	}
	current_subbuff = (current_subbuff + 1) % SUBBUFFER_COUNT;

	if (_trigger_position == UINT32_MAX && _trigger_mode == TRIGGER_MODE_AUTO && _current_time - _last_trigger_time > AUTO_TRIGGER_PERIOD_MS) {
		_trigger_position = _last_written_subbuff * SUBBUFFER_SIZE;
	}
	_trigger_search_time = TIM5->CNT - _trigger_search_start;
}

static void send_data()
{
	uint16_t *first_src;
	uint32_t first_len;
	uint16_t *second_src = NULL;
	uint32_t second_len = 0;
	if (_trigger_position < SUBBUFFER_SIZE) {
		first_len = SUBBUFFER_SIZE - _trigger_position;
		first_src = _samples_buf + SAMPLES_BUFF_SIZE - first_len;
		second_len = MSG_SAMPLES_COUNT - first_len;
		second_src = _samples_buf;
	} else if (_trigger_position > (SUBBUFFER_COUNT - 1) * SUBBUFFER_SIZE) {
		first_len = SAMPLES_BUFF_SIZE - (_trigger_position - SUBBUFFER_SIZE);
		first_src = _samples_buf + _trigger_position - SUBBUFFER_SIZE;
		second_len = MSG_SAMPLES_COUNT - first_len;
		second_src = _samples_buf;
	} else {
		first_src = _samples_buf + _trigger_position - SUBBUFFER_SIZE;
		first_len = MSG_SAMPLES_COUNT;
	}
	memcpy(_data_msg_buf + DATA_MSG_HEADER_FULL_SIZE, first_src, SAMPLE_SIZE * first_len);
	if (second_src) {
		memcpy(_data_msg_buf + DATA_MSG_HEADER_FULL_SIZE + first_len * SAMPLE_SIZE, second_src, SAMPLE_SIZE * second_len);
	}
//	_trigger_search_time = TIM5->CNT - _trigger_search_start;
	memcpy(_data_msg_buf + DATA_MSG_SIZE, &_trigger_search_time, 4);
	_trigger_position = UINT32_MAX;
	_last_trigger_time = _current_time;
	send_usb_msg((uint8_t*)_data_msg_buf, DATA_MSG_SIZE + 4);
}

void main_task(void *argument)
{
	MX_USB_DEVICE_Init();

	// Callback for buff locks releasing
	g_cdc_tratsmit_cplt_callback = cdc_transmit_cplt_callback;

	// Start ADC DMA and timer
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)_adc_buf, ADC_BUF_SIZE);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim5);

	_data_msg_buf[0] = HEADER_BYTE1;
	_data_msg_buf[1] = HEADER_BYTE2;
	_data_msg_buf[2] = MSG_TYPE_DATA;

	uint32_t offset = HEADER_SIZE + MSG_TYPE_SIZE;
	memcpy(_data_msg_buf + offset, &_sampling_period_ns, SAMPLING_PERIOD_SIZE);
	offset += SAMPLING_PERIOD_SIZE;
	memcpy(_data_msg_buf + offset, &_data_length, DATA_LENGTH_SIZE);

	osDelay(1000); // wait for USB connection

	uint32_t last_blink_time = 0;
	for(;;)
	{
		_current_time = HAL_GetTick();

		search_trigger();
		if (_trigger_position != UINT32_MAX) {
			if (_trigger_mode == TRIGGER_MODE_SINGLE) {
				_trigger_pause = 1;
			}
			send_data();
		}

		if (_current_time - last_blink_time > 100) {
			last_blink_time = _current_time;
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
	int16_t trigger_val;
	_trigger_mode = _msg_in_buf[offset] & TRIGGER_MODE_MASK;
	_trigger_direction = ((_msg_in_buf[offset] & 0x80) >> TRIGGER_DIRECTION_BIT_POS) ? TRIGGER_DIRECTION_FALLING : TRIGGER_DIRECTION_RISING;
	offset += TRIGGER_MODE_SIZE;
	memcpy(&trigger_val, _msg_in_buf + offset, TRIGGER_VAL_SIZE);
	_trigger_val_mv = trigger_val * ATTENUATION_FACTOR;

	_trigger_pause = 0;

	offset = HEADER_SIZE + MSG_TYPE_SIZE;
	_setting_msg_buf[offset] = _trigger_mode;
	offset += TRIGGER_MODE_SIZE;
	memcpy(_setting_msg_buf + offset, &trigger_val, TRIGGER_VAL_SIZE);

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

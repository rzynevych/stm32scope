/*
 * oscolloscope.c
 *
 *  Created on: 9 May 2026
 *      Author: Ruslan Zynevych
 */

#include "oscilloscope_protocol.h"

#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "tim.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

static uint16_t _adc_buf[ADC_BUF_SIZE];
static uint16_t _ring_buf[RING_BUFF_SIZE];
static uint8_t _data_msg_buf[DATA_MSG_SIZE];
static uint8_t _setting_msg_buf[SETTING_MSG_BUFF_SIZE];
static uint8_t _msg_in_buf[SETTING_MSG_BUFF_SIZE];

static OscilloscopeState _osc_state = {.data_length = MSG_SAMPLES_COUNT * SAMPLE_SIZE, .sampling_period_ns = 500};
static TriggerSettings _trg_settings = {.mode = TRIGGER_MODE_AUTO, .direction = TRIGGER_DIRECTION_RISING};
static TriggerSearchState _trg_srch_state = {0};

static volatile uint32_t _ring_buf_pos = 0;
static volatile uint32_t _last_written_subbuff = 0;

static uint32_t _current_time = 0;
static uint8_t  _usb_connected = 0;

static osThreadId_t _main_task_handle;
static osThreadId_t _usb_in_task_handle;

static void copy_samples(uint16_t *src)
{
	if (_trg_srch_state.position != TRG_NPOS) {
		return;
	}

	BaseType_t higherPriorityTaskWoken = pdFALSE;
	if (_osc_state.oversamp_factor == 0) {
		_ring_buf_pos = (_ring_buf_pos + SUBBUFFER_SIZE) % RING_BUFF_SIZE;
		memcpy(_ring_buf + _ring_buf_pos, src, SUBBUFFER_SIZE * sizeof(uint16_t));
		_last_written_subbuff = _ring_buf_pos / SUBBUFFER_SIZE;
		vTaskNotifyGiveFromISR(_main_task_handle, &higherPriorityTaskWoken);
	} else {
		uint32_t sum = 0;
		int i = 0;
		for (; i < SUBBUFFER_SIZE; ++i) {
			if (i != 0 && i % _osc_state.oversamp_factor == 0) {
				_ring_buf[_ring_buf_pos + i / _osc_state.oversamp_factor - 1] = sum / _osc_state.oversamp_factor;
				sum = 0;
			}
			sum += src[i];
		}
		_ring_buf[_ring_buf_pos + i / _osc_state.oversamp_factor - 1] = sum / _osc_state.oversamp_factor;
		_ring_buf_pos = (_ring_buf_pos + SUBBUFFER_SIZE / _osc_state.oversamp_factor) % RING_BUFF_SIZE;
		if (_ring_buf_pos % SUBBUFFER_SIZE == 0) {
			_last_written_subbuff = (SUBBUFFER_COUNT + _ring_buf_pos / SUBBUFFER_SIZE - 1) % SUBBUFFER_COUNT;
			vTaskNotifyGiveFromISR(_main_task_handle, &higherPriorityTaskWoken);
		}
	}
	portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	copy_samples(_adc_buf);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	copy_samples(_adc_buf + SUBBUFFER_SIZE);
}

uint8_t send_usb_msg(uint8_t* buf, uint16_t len)
{
	while (!g_usb_ready) {
		osDelay(1);
	}

	g_usb_ready = 0;
	uint8_t ret = CDC_Transmit_FS(buf, len);
	return ret;
}

// Calculate distance between subbuffers
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
	if (_trg_srch_state.pause) {
		return;
	}

	// skip trigger to keep trigger period over MIN_TRIGGER_PERIOD_MS
	if (_current_time - _trg_srch_state.last_trg_time < MIN_TRIGGER_PERIOD_MS) {
		return;
	}

	// Calculate distance from current_subbuff to last_written_subbuff
	uint32_t dist = subbuff_distance(_last_written_subbuff, _trg_srch_state.current_subbuff);
	if (dist == 0) {
		return;
	}
	if (dist > 1) {
		_trg_srch_state.current_subbuff = (_trg_srch_state.current_subbuff + dist - 1) % SUBBUFFER_COUNT;
	}

	_trg_srch_state.search_start_ms = TIM5->CNT;

	TriggerSettings trigger;
	taskENTER_CRITICAL(); // guarantees that settings aren't partially updated
	trigger = _trg_settings;
	taskEXIT_CRITICAL();

	uint8_t  trigger_ready = 0;
	int16_t trigger_int_val = (int32_t)trigger.direction * ADC_MAX_INT_VAL * (ADC_MID_VOLTAGE_MV + trigger.val_mv)/ADC_MAX_VOLTAGE_MV;
	for (int i = _trg_srch_state.current_subbuff * SUBBUFFER_SIZE; i < (_trg_srch_state.current_subbuff + 1) * SUBBUFFER_SIZE; ++i) {
		int16_t val = trigger.direction * _ring_buf[i];
		if (val < trigger_int_val - TRG_HYST) {
			trigger_ready = 1;
		}
		if (trigger_ready && _trg_srch_state.position == TRG_NPOS && val > trigger_int_val + TRG_HYST) {
			_trg_srch_state.position = i;
			break;
		}
	}
	_trg_srch_state.current_subbuff = (_trg_srch_state.current_subbuff + 1) % SUBBUFFER_COUNT;
	_trg_srch_state.search_time_ms += TIM5->CNT - _trg_srch_state.search_start_ms;

	if (trigger.mode == TRIGGER_MODE_AUTO && _trg_srch_state.position == TRG_NPOS && _current_time - _trg_srch_state.last_trg_time > AUTO_TRIGGER_PERIOD_MS) {
		_trg_srch_state.position = _last_written_subbuff * SUBBUFFER_SIZE;
	}
}

static void send_data()
{
	uint16_t *first_src;
	uint32_t first_len;
	uint16_t *second_src = NULL;
	uint32_t second_len = 0;
	if (_trg_srch_state.position < SUBBUFFER_SIZE) {
		first_len = SUBBUFFER_SIZE - _trg_srch_state.position;
		first_src = _ring_buf + RING_BUFF_SIZE - first_len;
		second_len = MSG_SAMPLES_COUNT - first_len;
		second_src = _ring_buf;
	} else if (_trg_srch_state.position > (SUBBUFFER_COUNT - 1) * SUBBUFFER_SIZE) {
		first_len = RING_BUFF_SIZE - (_trg_srch_state.position - SUBBUFFER_SIZE);
		first_src = _ring_buf + _trg_srch_state.position - SUBBUFFER_SIZE;
		second_len = MSG_SAMPLES_COUNT - first_len;
		second_src = _ring_buf;
	} else {
		first_src = _ring_buf + _trg_srch_state.position - SUBBUFFER_SIZE;
		first_len = MSG_SAMPLES_COUNT;
	}
	memcpy(_data_msg_buf + DATA_MSG_HEADER_FULL_SIZE, first_src, first_len * SAMPLE_SIZE);
	if (second_src) {
		memcpy(_data_msg_buf + DATA_MSG_HEADER_FULL_SIZE + first_len * SAMPLE_SIZE, second_src, second_len * SAMPLE_SIZE);
	}

	_data_msg_buf[HEADER_SIZE + MSG_TYPE_SIZE] = _osc_state.data_msg_seq;
	_osc_state.data_msg_seq++;

//	_trg_srch_state.search_time_ms = TIM5->CNT - _trg_srch_state.search_start_ms;
	memcpy(_data_msg_buf + HEADER_SIZE + MSG_TYPE_SIZE + SEQ_SIZE + DATA_LENGTH_SIZE, &_trg_srch_state.search_time_ms, TRIGGER_SEARCH_TIME_SIZE);
	_trg_srch_state.position = TRG_NPOS;
	_trg_srch_state.last_trg_time = _current_time;
	send_usb_msg((uint8_t*)_data_msg_buf, DATA_MSG_SIZE);
}

void main_task(void *argument)
{
	// Start ADC DMA and timer
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)_adc_buf, ADC_BUF_SIZE);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim5);

	_data_msg_buf[0] = HEADER_BYTE1;
	_data_msg_buf[1] = HEADER_BYTE2;
	_data_msg_buf[2] = MSG_TYPE_DATA;
	_data_msg_buf[DATA_MSG_SIZE - 2] = FOOTER_BYTE1;
	_data_msg_buf[DATA_MSG_SIZE - 1] = FOOTER_BYTE2;

	_trg_srch_state.position = TRG_NPOS;

	uint32_t offset = HEADER_SIZE + MSG_TYPE_SIZE + SEQ_SIZE;
	memcpy(_data_msg_buf + offset, &_osc_state.data_length, DATA_LENGTH_SIZE);

	osDelay(USB_CONNECT_DELAY_MS); // wait for USB connection

	uint32_t last_blink_time = 0;
	for(;;)
	{
		_current_time = HAL_GetTick();

		ulTaskNotifyTake(pdTRUE, SUBBUFER_WAIT_MAX_DELAY_MS); // wait for next subbuffer
		search_trigger();
		if (_trg_srch_state.position != TRG_NPOS && _usb_connected) {
			if (_trg_settings.mode == TRIGGER_MODE_SINGLE) {
				_trg_srch_state.pause = 1;
			}
			send_data();
		}

		if (_current_time - last_blink_time > LED_BLINK_PERIOD_MS) {
			last_blink_time = _current_time;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		}
	}
}


static void handle_set_trigger()
{
	size_t offset = CMD_START_POS;
	int16_t trigger_val;
	uint8_t trigger_mode;
	int8_t	trigger_direction;
	trigger_mode = _msg_in_buf[offset] & TRIGGER_MODE_MASK;
	trigger_direction = ((_msg_in_buf[offset] & 0x80) >> TRIGGER_DIRECTION_BIT_POS) ? TRIGGER_DIRECTION_FALLING : TRIGGER_DIRECTION_RISING;
	offset += TRIGGER_MODE_SIZE;
	memcpy(&trigger_val, _msg_in_buf + offset, TRIGGER_VAL_SIZE);

	taskENTER_CRITICAL(); // for thread safe settings update
	_trg_settings.mode = trigger_mode;
	_trg_settings.direction = trigger_direction;
	_trg_settings.val_mv = trigger_val * ATTENUATION_FACTOR;
	_trg_srch_state.pause = 0;
	taskEXIT_CRITICAL();

	offset = HEADER_SIZE;
	_setting_msg_buf[offset] = MSG_TYPE_TRIGGER;
	offset += MSG_TYPE_SIZE;
	_setting_msg_buf[offset] = _trg_settings.mode;
	offset += TRIGGER_MODE_SIZE;
	memcpy(_setting_msg_buf + offset, &trigger_val, TRIGGER_VAL_SIZE);
	send_usb_msg((uint8_t*)_setting_msg_buf, TRIGGER_MSG_SIZE);
}

static void handle_set_samp_rate()
{
	size_t offset = CMD_START_POS;
	uint8_t samp_rate = _msg_in_buf[offset];
	uint32_t adc_timer_period;
	uint32_t sampling_period_ns;
	uint32_t oversamp_fact;
	switch (samp_rate) {
	case SR_2MSPS:
		adc_timer_period = SR_PERIOD_2MSPS;
		oversamp_fact = OVS_FACT_2MSPS;
		sampling_period_ns = 500;
		break;
	case SR_1MSPS:
		adc_timer_period = SR_PERIOD_1MSPS;
		oversamp_fact = OVS_FACT_1MSPS;
		sampling_period_ns = 1000;
		break;
	case SR_100KSPS:
		adc_timer_period = SR_PERIOD_100KSPS;
		oversamp_fact = OVS_FACT_100KSPS;
		sampling_period_ns = 10000;
		break;
	case SR_10KSPS:
		adc_timer_period = SR_PERIOD_10KSPS;
		oversamp_fact = OVS_FACT_10KSPS;
		sampling_period_ns = 100000;
		break;
	default:
		return;
	}

	HAL_TIM_Base_Stop(&htim2);

	taskENTER_CRITICAL();
	_ring_buf_pos = 0;
	_osc_state.oversamp_factor = oversamp_fact;
	_osc_state.sampling_period_ns = sampling_period_ns;
	taskEXIT_CRITICAL();

	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_AUTORELOAD(&htim2, adc_timer_period - 1);

	HAL_TIM_Base_Start(&htim2);

	offset = HEADER_SIZE;
	_setting_msg_buf[offset] = MSG_TYPE_SAMP_RATE;
	offset += MSG_TYPE_SIZE;
	memcpy(_setting_msg_buf + offset, &_osc_state.sampling_period_ns, SAMPLING_PERIOD_SIZE);
	offset += SAMPLING_PERIOD_SIZE;
	send_usb_msg((uint8_t*)_setting_msg_buf, SAMP_PERIOD_MSG_SIZE);
}

static uint8_t crc8(const uint8_t* data, size_t length)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

static void handle_in_message()
{
	if (_msg_in_buf[0] != HEADER_BYTE1 || _msg_in_buf[1] != HEADER_BYTE2) {
		return;
	}
	size_t offset = HEADER_SIZE;

	uint8_t msg_len = _msg_in_buf[offset];
	offset += CMD_LEN_SIZE;

	uint8_t msg_crc = _msg_in_buf[offset + msg_len];
	uint8_t check_crc = crc8(_msg_in_buf + offset, msg_len);
	if (msg_crc != check_crc) {
		return;
	}

	uint8_t cmd_type = _msg_in_buf[offset];
	offset += CMD_TYPE_SIZE;
	switch (cmd_type) {
	case CMD_TYPE_TRIGGER:
		handle_set_trigger();
		break;
	case CMD_TYPE_SAMP_RATE:
		handle_set_samp_rate();
		_usb_connected = 1;
		break;
	case CMD_TYPE_DISCONNECT:
		_usb_connected = 0;
		break;
	}
}

void usb_in_task(void *argument)
{
	g_usb_in_buff = _msg_in_buf;
	_setting_msg_buf[0] = HEADER_BYTE1;
	_setting_msg_buf[1] = HEADER_BYTE2;
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
		.priority = (osPriority_t) osPriorityAboveNormal,
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

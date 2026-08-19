/*
 * oscilloscope_protocol.h
 *
 *  Created on: 19 Jul 2026
 *      Author: Ruslan Zynevych
 */

#ifndef OSCILLOSCOPE_PROTOCOL_H_
#define OSCILLOSCOPE_PROTOCOL_H_

#include <stdint.h>

#define HEADER_SIZE 2
#define FOOTER_SIZE 2
#define MSG_TYPE_SIZE 1
#define SEQ_SIZE 1
#define DATA_LENGTH_SIZE 2
#define TRIGGER_SEARCH_TIME_SIZE 4 // for debug
#define METADATA_SIZE (SEQ_SIZE + DATA_LENGTH_SIZE + TRIGGER_SEARCH_TIME_SIZE)
#define MSG_SAMPLES_COUNT 1000
#define SAMPLE_SIZE 2
#define DATA_MSG_HEADER_FULL_SIZE (HEADER_SIZE + MSG_TYPE_SIZE + METADATA_SIZE)
#define DATA_MSG_SIZE (DATA_MSG_HEADER_FULL_SIZE + MSG_SAMPLES_COUNT * SAMPLE_SIZE + FOOTER_SIZE)

#define ADC_BUF_SIZE MSG_SAMPLES_COUNT
#define SUBBUFFER_SIZE (MSG_SAMPLES_COUNT/2)
#define SUBBUFFER_COUNT 4
#define SAMPLES_BUFF_SIZE (SUBBUFFER_SIZE * SUBBUFFER_COUNT)

#define CMD_LEN_SIZE 1
#define CMD_TYPE_SIZE 1
#define CMD_START_POS (HEADER_SIZE + CMD_LEN_SIZE + CMD_TYPE_SIZE)
#define SAMPLING_PERIOD_SIZE 4
#define TRIGGER_MODE_SIZE 1
#define TRIGGER_VAL_SIZE 2
#define TRIGGER_MODE_MASK 0x0F
#define TRIGGER_DIRECTION_BIT_POS 7
#define SAMP_RATE_SIZE 1
#define TRIGGER_MSG_SIZE (HEADER_SIZE + MSG_TYPE_SIZE + TRIGGER_MODE_SIZE + TRIGGER_VAL_SIZE)
#define SAMP_PERIOD_MSG_SIZE (HEADER_SIZE + MSG_TYPE_SIZE + SAMPLING_PERIOD_SIZE)

#define SETTING_MSG_BUFF_SIZE CDC_DATA_FS_MAX_PACKET_SIZE

#define HEADER_BYTE1 0xAA
#define HEADER_BYTE2 0xBB
#define FOOTER_BYTE1 0xCC
#define FOOTER_BYTE2 0xDD

#define ADC_MAX_VOLTAGE_MV 3300
#define ADC_MID_VOLTAGE_MV (ADC_MAX_VOLTAGE_MV/2)
#define ATTENUATION_FACTOR 0.4
#define ADC_MAX_INT_VAL 4095
#define TRG_HYST 10
#define TRG_NPOS UINT32_MAX
#define MIN_TRIGGER_PERIOD_MS 20
#define AUTO_TRIGGER_PERIOD_MS 100

#define USB_CONNECT_DELAY_MS 1000
#define LED_BLINK_PERIOD_MS 100

enum MsgType {
	MSG_TYPE_DATA,
	MSG_TYPE_TRIGGER,
	MSG_TYPE_SAMP_RATE
};

enum CmdType {
	CMD_TYPE_TRIGGER,
	CMD_TYPE_SAMP_RATE,
	CMD_TYPE_DISCONNECT
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

enum SampleRateVal {
	SR_2MSPS,
	SR_1MSPS,
	SR_100KSPS,
	SR_10KSPS
};

enum SampleRateTimerPeriod {
	SR_PERIOD_2MSPS = 32,
	SR_PERIOD_1MSPS = 64,
	SR_PERIOD_100KSPS = 64,
	SR_PERIOD_10KSPS = 64
};

enum OversamplingFactor {
	OVS_FACT_2MSPS = 0,
	OVS_FACT_1MSPS = 0,
	OVS_FACT_100KSPS = 10,
	OVS_FACT_10KSPS = 100
};

typedef struct {
	uint32_t sampling_period_ns;
	uint32_t oversamp_factor;
	uint16_t data_length;
	uint8_t  data_msg_seq;
} OscilloscopeState;

typedef struct {
	uint8_t  mode;
	int8_t   direction;
	int16_t  val_mv;
} TriggerSettings;

typedef struct {
	uint32_t last_trg_time;
	uint32_t position;
	uint32_t search_start_ms;
	uint32_t search_time_ms;
	uint8_t  pause;
} TriggerSearchState;

#endif /* OSCILLOSCOPE_PROTOCOL_H_ */

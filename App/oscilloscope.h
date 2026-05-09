/*
 * oscilloscope.h
 *
 *  Created on: 9 мая 2026 г.
 *      Author: rusla
 */

#ifndef OSCILLOSCOPE_H_
#define OSCILLOSCOPE_H_

#define ADC_BUF_SIZE 1000
#define HEADER_SIZE 2
#define DURATION_SIZE 4
#define DATA_LENGTH_SIZE 2
#define METADATA_SIZE (DURATION_SIZE + DATA_LENGTH_SIZE)
#define MSG_SIZE (HEADER_SIZE + METADATA_SIZE + ADC_BUF_SIZE)

#define HEADER_BYTE1 0xAA
#define HEADER_BYTE2 0xBB

void main_task_init();

#endif /* OSCILLOSCOPE_H_ */

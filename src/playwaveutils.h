/*---------------------------------------------------------------------------/
/ playwaveutils - .wav file handling module
/ 
/ utility module developped as part of the DuckyBeats project
/----------------------------------------------------------------------------/
/ Copyright (C) 2019, Hugo Schaaf, all right reserved.
/----------------------------------------------------------------------------*/
#ifndef PLAYWAVEUTILS_H
#define PLAYWAVEUTILS_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include "pff.h"

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Wavefile structuration
 *      data identifier       offset (in byte)
 */
#define FILE_BLOCK_ID       	0x00
#define FILE_BLOCK_SIZE       	0x04
#define FILE_FORMAT           	0x08
#define FORMAT_BLOCK_ID       	0x0C
#define FORMAT_BLOCK_SIZE      	0x10
#define SAMPLE_FORMAT         	0x14
#define NUM_CHANNELS          	0x16
#define SAMPLE_FREQUENCY      	0x18
#define BYTE_PER_SEC          	0x1C
#define BYTE_PER_BLOCK        	0x20
#define BITS_PER_SAMPLE       	0x22
#define DATA_BLOCK_ID       	0x24
#define DATA_BLOCK_SIZE        	0x26
#define DATA_START_OFFSET		0x28

#define WAVEFILE_HEADER_SIZE  	0x28 /* 40 bytes long */
#define ID_SIZE               	0x04 /* size of each block Id */
#define WAVEFILE_FORMAT_ID    	"WAVE"

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* four character compare */
#define FCC(c1,c2,c3,c4)	(((uint32_t)c4<<24)+((uint32_t)c3<<16)+((uint32_t)c2<<8)+(uint32_t)c1)
/* load little endian word/dword */
#define LD_WORD(p)			(((uint16_t)*(p+1)<<8)+(uint16_t)*p)
#define LD_DWORD(p)			(((uint32_t)*(p+3)<<24)+((uint32_t)*(p+2)<<16)+((uint32_t)*(p+1)<<8)+(uint32_t)*p)


/* fifo read buffer setup */
#define BUFFER_SIZE 128     /* sd card read performs only 512 bytes reads. So 2 buffers of 256 bytes to store data 
                               and maximise read efficiency */    
#define pos(buf, offset)    (buf+offset)

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Audio management
 */

#define SAMPLE_FREQ_MAX             10000UL
#define SAMPLE_FREQ_MIN             8000UL
#define SAMPLE_TIMER_SET_FREQ(f)    OCR0A = (uint8_t)(F_CPU/8UL/(uint32_t)(f)-1UL)

/* data sampling timer :
 * The lower sample frequency limit is fixed by the maximal 
 * sample timer's period :
 * - (OCR0A_max+1)/F_CPU/8 = 128µs ~> 8kHz
 * The upper limit sample frequency is fixed by the maximal
 * PWM frequency which must by at least 8 times the maximal
 * sample frequency :
 * - F_CPU/256/8 = 8kHz
 */

static inline
void sample_timer_init(void)
{
    TCCR0A = (uint8_t)_BV(WGM01);   /* timer CTC mode, mode 2 */
    TCCR0B = 0;                     /*                        */
    TIMSK0 = 0;
    TCNT0 = 0;
}

static inline
void sample_timer_start(void)
{
    TIMSK0 |= (uint8_t) _BV(OCIE0A);  
    TCCR0B |= (uint8_t) _BV(CS01);    /* select clock source, 8 prescaling
                                         sample freq range : ~8kHz - ~16kkHz   */
}

static inline
void sample_timer_stop(void)
{
    TCCR0B &= (uint8_t) ~_BV(CS01);
    TIMSK0 &= (uint8_t) ~_BV(OCIE0A);
}

/* PWM waveform generation timer
 * I/O clock selected with no prescaling
 * so the timer is counting @F_CPU, faster as possible
 */
#define SET_PWM_VALUE(d)            OCR2B = (uint8_t)d
#define SET_PWM_PIN_OUTPUT()        DDRD |= (uint8_t) _BV(PD3)

static inline
void PWM_init(void)
{
	SET_PWM_PIN_OUTPUT();
    TCCR2A = (uint8_t)(_BV(COM2B1) | _BV(WGM21) | _BV(WGM20));   /* fast PWM mode, OC2B output pin */
    TCCR2B = 0x00;
    TIMSK2 = 0;
    TCNT2 = 0;
}

static inline
void PWM_start(void)
{
	SET_PWM_VALUE(128);	/* start to the center value */
    TCCR2B |= (uint8_t)_BV(CS20);	/* start PWM at F_CPU frequency/256 */
}

static inline
void PWM_stop(void)
{
    TCCR2B &= (uint8_t) ~_BV(CS20);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * playback routine and .wav file 
 * management
 */
extern FATFS fs;
extern DIR dir;
extern FILINFO fno;

uint32_t load_header(void);
uint8_t playback(void);

#endif
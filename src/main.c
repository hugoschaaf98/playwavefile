/* main.c
 *
 * Play .wav files stored on a SDc/MMc
 * external card. Only 8bit resolution
 * mono encoding accepted otherwise
 * the file will be skipped.
 *
 * Author : Hugo Schaaf
 * Date : 12/05/2019
 */
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "pff.h"
#include "playwaveutils.h"
#include "usart328p.h"

FATFS fs;
DIR dir;
FILINFO fno;

void IO_init(void);

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/* main thread
 */
int main(void)
{
	FRESULT res;
	char path[23];

	IO_init();

	res = pf_mount(&fs);
	if(res != FR_OK) usart_puts("Cannot mount memory card.\n");
	else
	{
		res = pf_opendir(&dir, "WAV");
		if(res != FR_OK) usart_puts("Unable to open WAV directory.\n");
		else
		{
			/* play the directory */
			for(;;)
			{
				res = pf_readdir(&dir, &fno);
				if( res != FR_OK || fno.fname[0] == '\0') break;

				usart_puts("\nOpen : ");
				usart_puts(fno.fname);
				
				sprintf(path, "%s/%s", "WAV", fno.fname);
				res = pf_open(path);
				if( load_header() < 1024)
				{
					usart_puts("\ncan't play file.\n");
				}
				else
				{
					usart_puts("\nstart playing...\n");
					if(playback()) usart_puts("error whilhe playing.\n");
					else usart_puts("file successfully played.\n");
				}
			}
			usart_puts("Directory entirely played.\n");
		}
	}

	for(;;)
		{;;}

	return 0;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * utility stuff
 */

void IO_init(void)
{
	usart_init(9600);	/* init the usart interface */
	_delay_ms(200);		/* let enough time for SD card to power On */
}
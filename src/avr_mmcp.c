/*-----------------------------------------------------------------------
 * Low level disk I/O module for Petit FatFs (C)ChaN, 2014 
 * 
 * Designed for ATmega328P using SPI interface
 * 
 * Author   : Hugo Schaaf
 * date     : 05/2019 
 * 
 * v2.2.3
 *-----------------------------------------------------------------------*/

#include "diskio.h"
#include <avr/io.h>
#include <util/delay.h>

/* SPI pin definition 						*/
/* here are specifications for atmega328p ! */
#ifndef SPI_DDR
	#define SPI_DDR		DDRB
#endif
#ifndef SPI_PORT
	#define SPI_PORT 	PORTB
#endif
#ifndef SPI_CS
	#define SPI_CS		PB2
#endif
#ifndef SPI_MOSI
	#define SPI_MOSI	PB3
#endif
#ifndef SPI_MISO
	#define SPI_MISO	PB4
#endif
#ifndef SPI_SCK
	#define SPI_SCK		PB5
#endif

/* Chip Select (CS) control */
#ifndef SELECT()
	#define SELECT()	SPI_PORT &= (uint8_t)(~_BV(SPI_CS))
#endif
#ifndef DESELECT()
	#define DESELECT()	SPI_PORT |= (uint8_t)_BV(SPI_CS)
#endif
#ifndef IS_SELECTED()
	#define IS_SELECTED() (SPI_PORT & _BV(SPI_CS))
#endif

/* SD/MMC SPI command set definiton */
/* MSByte of a command frame -> 0 1 index[5:0]
 * so b7:b6 01 stands for 0x40
 */
#define GO_IDLE 			(0x40 + 0)  /* CMD0 */
#define INIT 				(0x40 + 1)  /* CMD1 */
#define APP_INIT 			(0xC0 + 41) /* ACMD41, 1st bit set artificially to reconize acmd further */
#define CHECK_V 			(0x40 + 8)  /* CMD8 */
#define STOP_READ 			(0x40 + 12) /* CMD12 */
#define SET_BLOCKLEN 		(0x40 + 16) /* CMD16 */
#define READ_SINGLE_BLOCK 	(0x40 + 17) /* CMD17 */
#define READ_MULTI_BLOCK 	(0x40 + 18) /* CMD18 */
#define WRITE_SINGLE_BLOCK 	(0x40 + 24) /* CMD24 */
#define WRITE_MULTI_BLOCK 	(0x40 + 25) /* CMD25 */
#define ACMD_LEADING 		(0x40 + 55) /* CMD55 */
#define READ_OCR 			(0x40 + 58) /* CMD58 */
#define IS_ACMD(cmd)		(cmd & 0x80)/* test is this is an ACMD command */
#define ACMD_MASK			0x7F		/* retrieve the following command of ACMD suite */	

/* R1 response flags */
#define IN_IDLE_STATE		0x01
#define ERASE_RESET			0x02
#define ILLEGAL_CMD			0x04
#define CMD_CRC_ERR			0x08
#define ERASE_SEQ_ERR		0x10
#define ADDRESS_ERR			0x20
#define PARAM_ERR			0x40
#define IS_R1_RESP(i)		((i & 0x80)==0)

/* Data Token */
#define D_TOK1 				0xFE /* token for CMD17/18/24 */
#define D_TOK2				0xFC /* token for CMD25 */
#define STP_TRAN_TOK		0xFD /* stop transmission token for CDM25 */

/* Flags definitions in command responses */
#define HCS_SET				0x40000000UL
#define CCS_SET				0x40000000UL

/* Data responses flags */
#define DATA_RESP_MASK		0x0F
#define DATA_ACCEPTED		0x05
#define DATA_CRC_ERR		0x0B
#define DATA_WRITE_ERR		0x0D

/* extreme values */
#define DATA_MAX_SIZE		512	/* max number of bytes to read or write */

/* valid CRC fields + 1 terminating bit
 * if CMD0 or CMD8, CRC must be correct so add CRC
 * CRC calculator : 
 * http://www.ghsi.de/pages/subpages/Online%20CRC%20Calculation/
 * with polynom : 10001001
 * CRC 7bits + 1 (LSB) to from a byte so 
 * i = (CRC<<1)+1
 */
#define GO_IDLE_CRC			0x95
#define CHECK_V_CRC			0X87

/* Card types identification */
#define CT_UNKNOWN			0x00 /* byte adress for these cases */
#define CT_SDC1				0x01 /*-                            */
#define CT_SDC2				0x02 /*-                            */
#define CT_MMC3				0x04 /*-                            */
#define CT_BLOCK			0x08 /* block adress read/write in this case */

/* just to avoid trouble - forward data to the outgoing stream */
#define FORWARD(d)

static uint8_t cardType;

/*------------------------------------*/
/* Prototypes for spi control, mode 0 */


/* void init_spi(void)
 *
 * SPI initialization
 * Enable SPI as master, interrupts disabled, MSB transmitted first
 * mode 0 and 64 clock prescaling (supposing clk -> 16 MHz so spi clk running at 250kHz)
 */
static inline
void init_spi(void)
{
	/* data direction settings first, to avoid automatic slave switching */
	SPI_DDR |= (uint8_t)(_BV(SPI_MOSI) | _BV(SPI_CS) | _BV(SPI_SCK));	/* select data direction, 
															   SPI_MISO is overriden as an input */
	SPI_PORT |= (uint8_t)(_BV(SPI_MISO) | _BV(SPI_SCK) );	/* enable pullup resistor on MISO */

	PRR &= (uint8_t)(~_BV(PRSPI));	/* exit from power reduction mode to be able to enable SPI */
	SPSR &= (uint8_t)(~_BV(SPI2X));	/* disable spi double speed */
	SPCR = (uint8_t)(_BV(SPE) | _BV(MSTR) | _BV(SPR1) | _BV(SPR0));
}

/* void spi_set_rw_speed(void)
 *
 * Change the SPI clock frequency
 * assuming F_CPU is 16MHz, should not exceed
 * 20-25MHz.
 * SCK -> 16MHz/2 = 8MHz
 */
static inline
void spi_set_rw_speed(void)
{
	SPCR &= (uint8_t)( ~(_BV(SPR1) | _BV(SPR0)) );	/* clear previous speed config */
	SPSR |= (uint8_t)_BV(SPI2X);	/* set SCK freq = F_CPU/2 */
}

static inline
uint8_t spi(uint8_t data)
{	
	SPDR = data;
	while( !(SPSR & (uint8_t)(_BV(SPIF))) )
	{;;}
	return (uint8_t)SPDR;
}

static inline
uint8_t rx_spi(void)
{
	return spi(0xFF);
}

static inline
void tx_spi(uint8_t data)
{
	spi(data);
}

static inline
uint8_t waitNotBusy(uint16_t timeout)
{
	for(;timeout>0 && rx_spi()==0xFF; timeout--)
	{ 	_delay_ms(1); }
	return timeout>0;
}


/*-----------------------------------------*/
/* Prototypes for SDC/MMC SPI mode control */

/*-------------------------------------------*/
/* uint8_t send_cmd(uint8_t cmd, uint32_t arg)
 *
 * send a command to the drive. 
 * After sending a command, the drive is 
 * still selected (eg CS low). No need to
 * re-select the drive after having send a
 * command !
 */

static uint8_t send_cmd(uint8_t cmd, uint32_t arg)
{
	uint8_t i = 0xFF, result = 0x00;	/* i : dummy CRC and Stop */

	if(IS_ACMD(cmd))	/* if ACMD<n> command */
	{
		result = send_cmd(ACMD_LEADING, 0);
		cmd &= ACMD_MASK;	/* clear the MSB artificially set to retrieve 
					   	   the ACMD command index */
		if(result>1)
		{
			return result;
		}
	}

	/* give some extra clocks to the card and select it */
	DESELECT();
	rx_spi();
	SELECT();
	rx_spi();

	tx_spi(cmd);
	tx_spi((uint8_t)(arg >> 24));
	tx_spi((uint8_t)(arg >> 16));
	tx_spi((uint8_t)(arg >> 8));
	tx_spi((uint8_t)arg);

	if(cmd == GO_IDLE)
	{
		i = GO_IDLE_CRC;
	}
	if(cmd == CHECK_V)
	{
		i = CHECK_V_CRC;
	}

	tx_spi(i);

	if (cmd == STOP_READ)
	{
			result = rx_spi();	/* Receive stuff byte */
	}

	/* get the response to the command, 10 attempts. Maximum waiting time can be up to 8 dummy bytes */
	i=10;
	do
	{
		result = rx_spi();
	}while( !IS_R1_RESP(result) && i--);

	return result;
}


/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (void)
{
	uint8_t ocr[4]={0};
	uint16_t notimeout=0;	/*	notimeout == 0	-> timeout reached
						 	 *	notimeout > 0	-> timeout not reached
						 	 */
#if PF_USE_WRITE
	if (cardType != CT_UNKNOWN && !IS_SELECTED() ) disk_writep(0, 0);	/* Finalize write process if it is in progress */
#endif

	cardType = CT_UNKNOWN;

	init_spi();

	DESELECT();
	for(uint8_t i =10; i>0; i--)	/* 80 dummy clock */
	{ rx_spi(); }

	for(notimeout=1000; notimeout && (send_cmd(GO_IDLE, 0x00) != IN_IDLE_STATE) ; notimeout--)
	{
		_delay_us(100);
	}

	if(notimeout)
	{
		if(send_cmd(CHECK_V, 0x01AA) == 0x01)	/* if CMD8 is valid */
		{
			/* card type is SDC2 or SDHC/SDXC */
			for(uint8_t i=0; i<4; i++)	/* receive the 32 data bits of the response */
			{	ocr[i] = rx_spi(); }

			if(ocr[2] == 0x01 && ocr[3] == 0xAA)
			{
				/* wait for in_idle_state bit cleared */
				for(notimeout=10000; notimeout && send_cmd(APP_INIT, HCS_SET ); notimeout--)
				{
				}

				if(notimeout && send_cmd(READ_OCR, 0x00)== 0x00)	/* check if timeout reached */
				{
					/* check if CS flag is set to deduce correct card type */
					for(uint8_t i=0; i<4 ; i++)
						ocr[i] = rx_spi();

					cardType = (ocr[0] & 0x40) ? CT_SDC2 | CT_BLOCK : CT_SDC2;
				}
			}
		}
		else	/* if CMD8 isnt valid or no response, Card imay be SDC v1 or MMC v3 */
		{
			/* wait exiting IDLE_STATE */
			for(notimeout=10000; notimeout && send_cmd(APP_INIT, 0x00); notimeout--)
				_delay_us(100);

			if(notimeout)
			{	
				cardType = CT_SDC1;
			}

			else
			{
				for(notimeout=10000; notimeout && send_cmd(INIT, 0x00); notimeout--)
					_delay_us(100);

				cardType = (notimeout ? CT_MMC3 : CT_UNKNOWN);
			}
		}
	}

	/* set the block size if necessary */
	if(!(cardType & CT_BLOCK) && cardType != CT_UNKNOWN)
	{	
		send_cmd(SET_BLOCKLEN, 0x0200);
	}

	DESELECT();

	if(cardType != CT_UNKNOWN)
		spi_set_rw_speed();/* increase spi clock frequency */

	/* if card type has not been deduces */
	return (cardType==CT_UNKNOWN) ? STA_NOINIT : 0;
}



/*-----------------------------------------------------------------------*/
/* Read Partial Sector                                                   */
/*-----------------------------------------------------------------------*/
#if PF_USE_READ

inline 
static void skip_data(uint16_t bytes)
{
	while(bytes--)rx_spi();
}

DRESULT disk_readp (
	BYTE* buff,		/* Pointer to the destination object */
	DWORD sector,	/* Sector number (LBA) */
	UINT offset,	/* Offset in the sector */
	UINT count		/* Byte count (bit15:destination) */
)
{
	DRESULT res = RES_ERROR;
	uint16_t notimeout = 0, bc = 0;

	if(cardType == CT_UNKNOWN)	/* check if card has been initialized */
		return RES_NOTRDY;

	if(!(cardType & CT_BLOCK)) sector<<=9;
	
	if(offset+count > 512)	/* check if the parameters are valid */
		return RES_PARERR;

	if(send_cmd(READ_SINGLE_BLOCK, sector) == 0x00)	/* initiate read */
	{
		/* wait for the data token to be received */
		for(notimeout = 10000; notimeout && (rx_spi() != D_TOK1); notimeout--)
		{;;}

		if(notimeout)
		{
			bc = 512 + 2 - offset - count;	/* number of bytes to read -1 sector + CRC */

			/* skip leading data */
			skip_data(offset);

			if(buff)
			{
				/* fill in the buffer */
				do
				{
					*buff++=rx_spi();
				}while(--count);
			}
			else
			{
				/* forward to the outgoing stream */
				do
				{
					rx_spi();
					FORWARD(0);
				}while(--count);
			}

			skip_data(bc);/* skip trailing data and CRC */
			res = RES_OK;
		}
	}

	DESELECT();
	rx_spi();

	return res;
}
#endif

/*-----------------------------------------------------------------------*/
/* Write Partial Sector                                                  */
/*-----------------------------------------------------------------------*/

#if PF_USE_WRITE
/* if only up to 512 bytes (one block) will be written
 * at a time
 */
DRESULT disk_writep (
	const BYTE *buff,	/* Pointer to the bytes to be written (NULL:Initiate/Finalize sector write) */
	DWORD sc			/* Number of bytes to send, Sector number (LBA) or zero */
)
{
	DRESULT res;
	uint32_t bcnt;
	static uint32_t wcnt;	/* Sector write counter */

	//dbg("%s","entering disk_writep()\n");
	//dbg("sc = %ld\n", sc);

	res = RES_ERROR;

	if (buff)	/* Send data bytes */
	{
		//dbg("%s","writing data packet\n");
		bcnt = sc;
		while (bcnt && wcnt)	/* Send data bytes to the card */
		{		
			tx_spi(*buff++);
			wcnt--; bcnt--;
		}
		res = RES_OK;
	}
	else
	{
		if (sc)	/* Initiate sector write process */
		{
			//dbg("%s","initiating disk write\n");
			if (!(cardType & CT_BLOCK)) sc <<=9;	/* Convert to byte address if needed */
			if (send_cmd(WRITE_SINGLE_BLOCK, sc) == 0)	/* WRITE_SINGLE_BLOCK */
			{
				//dbg("%s","write command succeeded. Sending data token.\n");			
				tx_spi(0xFF);
				tx_spi(D_TOK1);	/* Data block header */
				wcnt = 512;			/* Set byte counter */
				res = RES_OK;
			}
		}
		else	/* Finalize sector write process */
		{
			//dbg("%s","finalizing disk write\n");
			bcnt = wcnt + 2;
			while (bcnt--) tx_spi(0);	/* Fill left bytes and CRC with zeros */
			if ((rx_spi() & DATA_RESP_MASK) == DATA_ACCEPTED)	/* Receive data resp and wait for end of write process in timeout of 500ms */
			{	
				//dbg("%s","data accepted. Waiting while card busy.\n");
				for (bcnt = 10000; rx_spi() != 0xFF && bcnt; bcnt--)	/* Wait for ready */
				{
					//_delay_us(4);
				}
				if (bcnt) res = RES_OK;
			}
			DESELECT();
			rx_spi();
		}
	}
	//dbg("%s","exiting disk_writep()\n");
	return res;
}

#endif

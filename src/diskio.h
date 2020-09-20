/*-----------------------------------------------------------------------
/  PFF - Low level disk interface modlue include file    (C)ChaN, 2014
/-----------------------------------------------------------------------*/

#ifndef _DISKIO_DEFINED
#define _DISKIO_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#include "pff.h"

/* I don't know if it changes really something. I have written 2 versions
 * of DRESULT disk_writep(BYTE*, DWORD).
 * By default let USE_MULTI_BLOCK_WRITE disabled.
 * 0 -> disable
 * 1 -> enable
 */
#define USE_MULTI_BLOCK_WRITE	0


/* Status of Disk Functions */
typedef BYTE	DSTATUS;


/* Results of Disk Functions */
typedef enum {
	RES_OK = 0,		/* 0: Function succeeded */
	RES_ERROR,		/* 1: Disk error */
	RES_NOTRDY,		/* 2: Not ready */
	RES_PARERR		/* 3: Invalid parameter */
} DRESULT;


/*---------------------------------------*/
/* Prototypes for disk control functions */

DSTATUS disk_initialize (void);
DRESULT disk_readp (BYTE* buff, DWORD sector, UINT offser, UINT count);
DRESULT disk_writep (const BYTE* buff, DWORD sc);

#define STA_NOINIT		0x01	/* Drive not initialized */
#define STA_NODISK		0x02	/* No medium in the drive */

#ifdef __cplusplus
}
#endif

#endif	/* _DISKIO_DEFINED */

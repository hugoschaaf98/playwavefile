#include "playwaveutils.h"

#ifndef dbg(s)
#ifdef DEBUG
    
    #define DBGFLAG     1
    #include <stdlib.h>
    #include "usart328p.h"
    #define dbg(s) usart_puts(s)
#else
    #define dbg(s)
#endif
#endif

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Wave file management
 * and play function
 * valid .wav files are LPCM - 8 bits resolution - mono
 */

static uint8_t buffer0[BUFFER_SIZE];
static uint8_t buffer1[BUFFER_SIZE];
static uint8_t* buffers[]={buffer0, buffer1};
volatile uint8_t    active_buffer = 0, alt_buffer = 1;
volatile uint16_t   buffer_index = 0, buffer_end = 0, bcnt=0;


uint32_t load_header (void)    /* 0:Invalid format, 1:I/O error, >=1024:Number of samples */
{
    uint32_t sz, f;
    uint16_t br;
    char dbgstr[50]="";


    if (pf_read(buffer0, WAVEFILE_HEADER_SIZE, &br)) return 1;   /* Load file header (40 bytes) */
    if (br != WAVEFILE_HEADER_SIZE || LD_DWORD( pos(buffer0, FILE_FORMAT) ) != FCC('W','A','V','E')) return 0;
    /* Get Chunk ID and size */
    if(LD_DWORD( pos(buffer0, FORMAT_BLOCK_ID) ) != FCC('f','m','t',' ') ) return 0;    /* fmt chunk */
    sz = LD_DWORD( pos(buffer0, FORMAT_BLOCK_SIZE) );        /* Chunk size */
    if (sz < 16) return 0;      /* Check chunk size. 16 is for LPCM, more is for uuh..?? */
    if ( LD_WORD( pos(buffer0,SAMPLE_FORMAT) ) != 1) return 0;   /* Check coding type (LPCM) */
    if ( LD_WORD( pos(buffer0,NUM_CHANNELS) ) != 1) return 0;   /* Check channels (1/2) */
    if ( LD_WORD( pos(buffer0,BITS_PER_SAMPLE) ) != 8) return 0;  /* Check resolution (8 bit) */

    f = LD_DWORD( pos(buffer0,SAMPLE_FREQUENCY) );    /* Check sampling frequency (8kHz-48kHz) */

    ltoa(f, dbgstr, 10);
    dbg("f : "); dbg(dbgstr); dbg("\n");

    if (f < SAMPLE_FREQ_MIN || f > SAMPLE_FREQ_MAX) return 0;
    
    SAMPLE_TIMER_SET_FREQ(f);   /* Set sampling interval */
    itoa(OCR0A, dbgstr, 10);
    dbg("OCR0A : "); dbg(dbgstr); dbg("\n");
    if( LD_DWORD( pos(buffer0,DATA_BLOCK_ID) ) == FCC('d','a','t','a'))     /* 'data' chunk */
    {
        sz = LD_DWORD( pos(buffer0,DATA_BLOCK_SIZE) );
        if (sz < 1024) return 0; /* Check size - 21ms minimum sound duration */
        PWM_init();
        return sz;  /* Start to play */
    }

    return 0;
}

/* audio sample timer interrupt */
ISR(TIMER0_COMPA_vect)
{
    uint8_t sreg = SREG;
    cli();

    if(buffer_index == BUFFER_SIZE)   /* if end of current buffer */
    {
        /* switch buffers */
        buffer_end = 1;
        buffer_index = 0;
        active_buffer ^= 1;
        alt_buffer ^=1;
    }

    SET_PWM_VALUE(buffers[active_buffer][buffer_index++]);
    bcnt--;

    SREG = sreg;
}

uint8_t playback(void)
{
    FRESULT res;
    uint16_t br;    /* bytes which have been read */

    dbg("Entering playback()\n");

    /* initialize fifo read buffer */
    active_buffer = 0;
    alt_buffer = 1;
    buffer_index = 0;
    buffer_end = 0;

    /* go to data and skip sector unaligned part to maximise further reads efficiency */
    if(pf_read(0, 512 - (fs.fptr + DATA_START_OFFSET)%512, &br) != FR_OK) return 1;

    dbg("first FIFO fill in.\n");

    pf_read(buffers[active_buffer], BUFFER_SIZE, &br);
    bcnt+= br;
    pf_read(buffers[alt_buffer], BUFFER_SIZE, &br);
    bcnt+= br;
    
    dbg("\nstarting play loop.\n");

    sei();
    PWM_start();
    sample_timer_start();

    do
    {
        if(buffer_end)
        {
            buffer_end = 0;
            res = pf_read(buffers[alt_buffer], BUFFER_SIZE, &br);
            bcnt = br;
        }

    }while(br == BUFFER_SIZE);

    /* wait while FIFO not empty */
    while(bcnt)
        {;;}

    sample_timer_stop();
    PWM_stop();

    dbg("exiting playback()\n");

    return 0;
}
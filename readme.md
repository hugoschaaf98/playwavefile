### Petit Fatfs low level I/O implementations for SDC/MMC cards

<avr_mmcp.c> has been writen to work with SDC/MMC devices. Please check
the register definitions of your chip for adaptation. Here, it's written to work on
AVR Atmega328p !
You should write the "write directly to output stream" which is not implemented here,
(implement/replace the FORWARD() macro) as it's project specific (as mentionned in the Petit Fatfs documentation).



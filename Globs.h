
#ifndef _GLOBS_H_
#define _GLOBS_H_

#define F_CPU 11059200UL
// #define F_CPU 7372800UL

#define _FLASHER_


#define TRUE 1
#define FALSE 0


#define CORES 32		// don't touch this
#define MAXASICS 32		// don't touch this ... max supported ASICS on board
#define ASICS 32			// can be 4, 24 or 32 ... real count of the ASISC on a board
#define JCELLS	8
#define FIFO	33

#define S_CHECK 0
#define S_READ 1


#define _VER_	129
#define C_FIX	17	// 180 MHz


#ifdef _FLASHER_
#define SECTION_FLASH  __attribute__ ((section (".flash")))
#endif


typedef unsigned int u_int;
typedef unsigned char u_char;
typedef unsigned long u_long;

typedef unsigned char BYTE;
typedef unsigned int WORD;
typedef unsigned long DWORD;


#define FLASH  __attribute__ ((progmem))


#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#ifndef sbi
#define sbi(port,bit)		(port |=  (1<<bit))
#define cbi(port,bit)		(port &= ~(1<<bit))
#define bis(port,bit)		(port & (1<<bit))
#define bic(port,bit)		(!(port & (1<<bit)))
#endif

#define SBI(port,bit)		(port |=  (1<<bit))
#define CBI(port,bit)		(port &= ~(1<<bit))
#define BIS(port,bit)		(port & (1<<bit))
#define BIC(port,bit)		(!(port & (1<<bit)))


// typedef union tag_ncn {
// 	DWORD	dwnonce;
// 	BYTE	nonce[4];
// } U_NONCE;

typedef struct _tag_pair {
	BYTE	wload[44];
	BYTE	gnonc[4];
} S_PAIR;
#define SZ_PAIR sizeof(S_PAIR)

typedef struct _tag_abuf {
	WORD incer;
	BYTE cix;
} S_ABUF;
#define SZ_ABUF sizeof(S_ABUF)

typedef struct _tag_subm {
	BYTE nonce[4];
	WORD incer;
	BYTE cix;
	BYTE chip;
} S_SUBM;
#define SZ_SUBM sizeof(S_SUBM)


typedef struct _tag_Job {
	BYTE midstate[32];
	BYTE mshit[4];
	BYTE ntime[4];
	BYTE ndiff[4];
	BYTE exnc2[4];
	BYTE mj_ID;
//	BYTE BrdIdx;
} S_MJOB;
#define SZ_MJOB sizeof(S_MJOB)

#endif

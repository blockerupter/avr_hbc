// MCU = atmega88p / pa
// fuses set: bootsz1, bootsz0, dwen, sut1, sut0, cksel3, cksel0



// modes of C_CLK
#define M_OFF	0xff
#define M_ALCK	(0 << 5)	// sets clock for all chips
#define M_LCLK	(1 << 5)	// sets clock for local selected chip
#define M_LRST	(2 << 5)	// reset only the selectd by C_CLK chip

// commands global for all boards
#define C_RES	(0 << 5)	// resets all the mega88s on all boards, returns silence
#define C_LPO	(1 << 5)	// LongPoll - stop the jobs, clear the FIFO pojnters, returns silence, the BoardID contains future/10 - 1 value, eg v=2 -> fu=10*(2+1) = 30 seconds
#define C_GCK	(2 << 5)	// global clock for all boards, on the BoardID place & 0x0f
#define C_DIF	(3 << 5)	// the BoardID replacedby last LSB 2 bits the difficulty

// commands board specified ones

#define C_JOB	(4 << 5)	// followed by WL[44] + exnc2[4] + MJOB_IDx[1] in 8N1, returns 0x58= confirmation that the Job has been gotten, good for sync also
#define C_ASK	(5 << 5)	// see below
#define C_TRS	(6 << 5)	// returns 32 bytes status of the core test + 32 bytes clocks + 1 byte = g_dif + (InFuture/10)-1)[1]  ... total 66 bytes
#define C_CLK	(7 << 5)	// resets mega88 on the selected board, returns silence

// answers on C_ASK:b
#define A_WAL	0x56	// ready to take a new master job :)
#define A_NO	0xa6	// nothing, means the chips are working/busy
#define A_YES	0x5A	// there is a nonce in the FIFO
#define A_STR	0x6c	// send a string or data followed by termination zero to the host, the host will print it.
// A_YES is followed by ... see below in the function AnswerIT();
// ------------------------



#define	E_IDLE	0xff
#define	E_ASK	0
#define	E_CLK	1
#define	E_RES	2
#define	E_LPO	3
#define E_TRS	4
#define	E_GCK	5
#define	E_JOB	6
#define	E_DIF	7
#define E_PDN	8
#define E_RSC	9


#define D_JOB	1
#define D_NNJ	2


#include <avr/boot.h>
#include "globs.h"

static BYTE b_bclk[MAXASICS];
static BYTE Astat[MAXASICS];

static WORD InFuture = 10;
static WORD cntime = 0;
static BYTE b9 = 0;
static BYTE locix = 0;
static BYTE g_dif = 0;
static BYTE d_sys = 0;
static BYTE head = 0;
static BYTE tail = 0;
static volatile BYTE BoardID;

static S_ABUF s_bufa[MAXASICS];
static S_SUBM s_fifo[FIFO];
static S_MJOB s_job[JCELLS];
static S_MJOB sb_job;
static BYTE   hang[MAXASICS];



#include "pairs_64.c"
#include "SPI.c"
#include "uart.c"

#ifdef _FLASHER_
static void flasher(void) SECTION_FLASH;

// static void SpmBufferFill(WORD adr, WORD data) SECTION_FLASH;
// static void SpmCommand(WORD addr, BYTE function) SECTION_FLASH;
// static void FlashPage(WORD page, void *data) SECTION_FLASH;
#endif

static BYTE Init(void) {
// Port IO section --------------------------------------------
///	DDRC |= 0x1f;				// outputs 5 bits = 32 channels/AISCs
///	PORTC = 0xdf;				// PORTC.5 is HiImpedance input  for the ADC
///								// PC.0-4=Addres/Assic, PB.5=ADC_Input. PB.6=Reset = not in use

	PORTD = 0x7f;				// PD.0=Rx, PD.2-6  pulluped inputs
	DDRD = 0xfe;				// PD.7=BS PD.1=Tx Outputs 
								// PD.0=Rx, PD.1=Tx, PD.2-6=MicroSwitch=BOX:Board_ID, PD.7=BS


	PORTC = 0x00;				// PC0-4=outputs 5 bits(32 AISCs), PC5=HW_Reset
	DDRC = 0x3f;				// all outputs

	PORTB = 0x3f;				// keep all in one ... B4=MISO is pulluped input. 
	DDRB = 0x2f;				// all outputs  except B6 & B7, which are for the ext. crystal and B4=MISO


	PORTD = 0x7f;				// PD.0=Rx, PD.2-6  pulluped inputs, PD7=0;
	DDRD = 0x82;				// PD.7=BS PD.1=Tx Outputs 
// -------------------------------------------------------------

// SPI section -------------------------------------------------
#if(F_CPU == 7372800UL)
	SPSR = 1 << SPI2X;						// enable fast mode
	SPCR  = (1<<SPE) | (1<<MSTR); 			// Enable SPI in Master mode, mode 0, LSB first, CLK = Fosc/2 = 11/2 = 5.5 MHz, 3.6Mhz at 7.3Mhz
#endif
#if(F_CPU == 11059200UL)
	SPSR = 0; 								// fast mode disabled
	SPCR  = (1<<SPE) | (1<<MSTR); 			// Enable SPI in Master mode, mode 0, LSB first, CLK = Fosc/4 = 11/4 = 2.7 MHz
#endif
//	SPSR = 1 << SPI2X;
//	SPCR  = (1<<SPE) | (1<<MSTR) | (1<<SPR0); 	// Enable SPI in Master mode, mode 0, LSB first, CLK = Fosc/8 = 11/8 = 1.38Mhz

//	SPSR = 0; //1 << SPI2X;
//	SPCR  = (1<<SPE) | (1<<MSTR) | (1<<SPR0); 	// Enable SPI in Master mode, mode 0, LSB first, CLK = Fosc/16 = 11/8 = 0.7MMhz
// -------------------------------------------------------------

// UART section	------------------------------------------------
	UBRR0H	= 0x00;				//(BYTE) (UART_BAUD_SELECT>>8);
#if(F_CPU == 11059200UL)
	UBRR0L	= 2; 				// 460800 with U2X(BYTE) Xtall =  11059200, val = 11059200/8/460800 - 1 = 3-1 = 2
#endif
#if(F_CPU == 7372800UL)
	UBRR0L	= 1; 				// 460800 with U2X(BYTE) Xtall =  7372800, val = 7372800/8/460800 - 1 = 2-1 = 1
#endif

 	UCSR0A	= (1<<U2X0);		// Double speed (divider is 8, not 16)
 	UCSR0C	= (3<<UCSZ00);		// 8N1
// 	UCSR0B	= (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0) | (1<<UCSZ02);	// Enable  Rx and Tx and Rx interrupt, 9bit mode, TXB8N=0
 	UCSR0B	= (1<<RXEN0) | (1<<TXEN0) | (1<<UCSZ02);				// Enable  Rx and Tx and no interrupt, 9bit mode, TXB8N=0
// -------------------------------------------------------------

// ADC part ----------------------------------------------------
///	ADCSRA	= 0x83;	//	ADC enabled, Clock = 8/8 = 1MHz, = 77ksps
// -------------------------------------------------------------
	return 0;
}



static void Reboot(void)		{ asm volatile("cli\n\tldi r31,0\n\tldi r30,0\n\tijmp"); }	// restart the MCU
static void TestResponse(void) 	{ PutnUART(&Astat[0],MAXASICS); PutnUART(&b_bclk[0],CORES); PutcUART(g_dif); PutcUART(InFuture/10); PutcUART(_VER_); }
static void JobReset(void)		{ tail=0; head=0; ASIC_Reset_All(); memset(hang,0,MAXASICS); d_sys = 0; cntime=0; }
static void Jobler(void)		{ if(++locix >= JCELLS) locix=0; memcpy((void *)&s_job[locix] ,(void*)&sb_job ,SZ_MJOB); d_sys |= D_JOB; cntime=0; }
static void TestLoad(BYTE cor)	{ for(BYTE i=0;i<44;i++) SPI_IO(i|0x80), SPI_IO(pgm_read_byte(&sPair[cor].wload[i])); Start_Job(); }
static void DwSwapB(BYTE *sr)	{ BYTE bt; bt=sr[0]; sr[0]=sr[3]; sr[3]=bt; bt=sr[1]; sr[1]=sr[2]; sr[2]=bt; } 
static void JobLoad(void)		{ 
	DWORD d = *(DWORD *)&s_job[locix].ntime;
	BYTE i, c, *p = (BYTE *)&d;
	
	DwSwapB((BYTE *)&d); d+=cntime; DwSwapB((BYTE *)&d);
	for(i=0;i<36;i++)		{ SPI_IO(i|0x80); SPI_IO(s_job[locix].midstate[i]); }
	for(c=0;c<4;c++)		{ SPI_IO(i|0x80); SPI_IO(p[c]); i++; }
	for(c=0;c<4;c++)		{ SPI_IO(i|0x80); SPI_IO(s_job[locix].ndiff[c]); i++; }
	Start_Job();
}

// returns midstate[32+4], ntime[4], ndiff[4], exnonc2[4], nonce[4], mj_ID[1], chipID[1] 
static void AnswerIT(void)		{
	static BYTE	cix;
	static DWORD	d;
	
	cix = s_fifo[tail].cix;
	d = *(DWORD *)&s_job[cix].ntime;
	DwSwapB((BYTE *)&d); d+=s_fifo[tail].incer; DwSwapB((BYTE *)&d);
	PutnUART(&s_job[cix].midstate[0],36);	// for(i=0;i<36;i++)		PutcUART(s_job.midstate[i]);
	PutnUART((BYTE *)&d,4); 				// for(i=0;i< 4;i++)		PutcUART(p[i]);
	PutnUART(s_job[cix].ndiff,8);	//,4);	// for(i=0;i< 4;i++)		PutcUART(s_job.ndiff[i]);
//	PutnUART(s_job[cix].exnc2,4);			// for(i=0;i< 4;i++)		PutcUART(s_job.exnc2[i]);
	PutnUART((BYTE *)&s_fifo[tail].nonce,4);
	PutcUART(s_job[cix].mj_ID);
	PutcUART(s_fifo[tail].chip);
	if(++tail >= FIFO)	tail = 0;
}

static void CoreTest(void)	{
	BYTE lop, cnt, cor, ch, res, nonc[4]; // nonc[16];

	memset(b_bclk,13,CORES);
	memset(Astat,0x00,MAXASICS);
	HW_Reset(); ASIC_Reset_All(); SetBitDiff(1);
	for(lop=0;lop<=2;lop++) {
		for(cor=0;cor<CORES;cor++) {
			for(ch=0;ch<ASICS;ch++) {
				if((Astat[ch] & 0x80))			continue;
				Open_nAsic(ch); ASIC_ResetDelay(ch); Clr_Mask();
				res = GetAsic();
				if(res==1)		TestLoad(cor);			// assuming the chip is here
				else			Astat[ch] |= 0x80;		// chip  is  not here
			}
			for(ch=0;ch<ASICS;ch++) {
				if((Astat[ch] & 0x80))			continue;
				if(!Astat[ch])					continue;	
				Open_nAsic(ch);
				cnt = 20; while(--cnt) { res = GetAsic(); if(res & 0x3c) break; }
				if(cnt) {
					GetNonce(nonc,res);
					if(++(*(DWORD *)&nonc) == pgm_read_dword(&sPair[cor].gnonc[0]))		Astat[ch] = 0x00;
					else Astat[ch] |= 0x20;
				} else Astat[ch] |= 0x40;
				_delay_us(300); // ASIC_Reset(ch); keep it running so the DCDC being loaded
			}
		}
	}
//	status 0x00 means the chip is OK, it is here and returns a golden nonce
//	status 0x20 means the chip is here, but returns a bad nonce 
//	status 0x40 means the chip is here, but doesn't return a nonce 
//	status 0x80 means the chip missing or not powered or not clocked or bad soldered or damaged
	ASIC_Reset_All();
	SetBitDiff(g_dif);
	for(ch=ASICS;ch<MAXASICS;ch++)	Astat[ch] = 0x80;
	for(ch=0;ch<MAXASICS;ch++)		if(Astat[ch] & 0x80) b_bclk[ch]=0;
}

static BYTE GetUART(void)		{ while( !(UCSR0A & (1<<RXC0)) ) ; b9 = UCSR0B & 2; return UDR0; }

static void DoUART(void) {
	BYTE *buf = (BYTE *)&sb_job;
	BYTE cmd, ch, idx, mode, chip;

//	if(!IsUART())				return;	
	b9 = UCSR0B & 2; ch = UDR0;
	if(b9)						return;
	cmd = ch & 0xe0; ch &= 0x1f;
	if(ch != BoardID)	if(cmd > C_DIF )	return; // skip the board specified commands, but the global ones will be executed even the boardID is differnet

	switch(cmd) {
		case C_ASK:	if(tail != head) { PutcUART(A_YES); AnswerIT(); }
					else { PutcUART( (d_sys ^ D_JOB) ? A_WAL:A_NO ); d_sys &= ~D_NNJ; }  return;
		case C_LPO:	InFuture = 10+ch*10; JobReset();				return;
#ifdef _FLASHER_
		case C_DIF:	if(ch==0x15) flasher();	else SetBitDiff(g_dif=ch&3);	return;
#else
		case C_DIF:	SetBitDiff(g_dif=ch&3);							return;
#endif
		case C_TRS: TestResponse();									return;
		case C_GCK: memset(b_bclk, ch, MAXASICS);  JobReset(); return;
		case C_RES:	Reboot();										return; // :)
		
		case C_JOB:	for(idx=0;idx<SZ_MJOB;idx++) { buf[idx] = GetUART();	if(!b9)	return; }
					PutcUART(0x58); Jobler();	return;

		case C_CLK:	ch = GetUART();	if(!b9)						return;
					mode = ch & 0xe0; chip = ch & 0x1f;
					switch(mode) {
						case M_ALCK:	for(idx=0;idx<MAXASICS;idx++) { 
											ch = GetUART() & 0x1f;
											if(!b9)				return;
											b_bclk[idx]=ch; 
										}
										JobReset(); return;
						case M_LCLK:	ch = GetUART() & 0x1f; 
										if(!b9)	return;	
										b_bclk[chip] = ch; // ASIC_Reset(chip);	return;
						case M_LRST: 	ASIC_Reset(chip);	return;
					}
	}
}


int main(void) {
	BYTE	ch, chead, nonce[4]; // nonce[16];
	BYTE	volatile res;
	
	Init();
	
	BoardID = PIND; BoardID ^= 0xff; BoardID >>= 2; BoardID &= 0x1f;
#if (ASICS < 5)
	BoardID = 1;
#endif
	
	memset(b_bclk,11,MAXASICS);
	memset(Astat,0,MAXASICS);
	HW_Reset();	 ASIC_Reset_All(); SetBitDiff(0);

	CoreTest(); ch = 0;

	memset(b_bclk,C_FIX+2,MAXASICS);
	memset(hang,0,MAXASICS);
	HW_Reset(); ASIC_Reset_All(); SetBitDiff(0);


	while(1) {
		if(!(Astat[ch] & 0x88) && (d_sys & D_JOB) ) {
			Open_nAsic(ch);  res = GetAsic();
			if( (res & 0x3c)) {
				if( GetNonce(nonce,res) ) {
					ASIC_Reset(ch); res &= 1; // attempt to shuffle the moment of job needed and avoid one more GetAsic()
					chead = head;
					if(++chead >= FIFO)	chead=0;
					if(chead != tail) {
						s_fifo[head].cix = s_bufa[ch].cix;	s_fifo[head].incer = s_bufa[ch].incer; s_fifo[head].chip = ch;
						*(DWORD *)&s_fifo[head].nonce[0] = *(DWORD *)&nonce;
						head = chead;
					}
				} 
			}
			if(res == 1) {
				hang[ch]++;
				if(d_sys & D_JOB) {
					JobLoad();
					s_bufa[ch].cix = locix;	s_bufa[ch].incer = cntime;
					if(cntime == InFuture)	d_sys |= D_NNJ;
					if(++cntime>=(InFuture+32))	d_sys &= ~D_JOB;
				}
			} else hang[ch]=0;
		}
		if(hang[ch] > 3) { hang[ch]=0; PwrDown(ch); }	// attempt to block the chip which is always in w_allow=1 state
		if(IsUART())			DoUART();
		if(++ch >= ASICS)	ch = 0;
#ifdef _FLASHER_
		if(ch>100)	{ flasher(); flasher(); }	// never happenes, but forces the compiler to allocate the flasher part at its section 0x1f00
#endif

	}
	return 0;
}



#ifdef _FLASHER_
static void  flasher(void) {
	WORD	adr, cnt;
	BYTE	page, pgend;
	
	while( !(UCSR0A & (1<<RXC0)) ); if(!(UCSR0B & 2))		return;
	pgend = UDR0;
	
	if(!pgend)				return;
	if(pgend>124)			return;			// 124 pages of 64 bytes = max 7936 bytes
	for(page = 0; page < pgend; page++) {
		for(cnt=0;cnt<32;cnt++) {
			while( !(UCSR0A & (1<<RXC0)) ); if(!(UCSR0B & 2))	return; adr = UDR0;
			while( !(UCSR0A & (1<<RXC0)) ); if(!(UCSR0B & 2))	return; adr |= UDR0<<8;
			while(BIS(SPMCSR,SPMEN)); __boot_page_fill_normal(cnt<<1, adr);
		}
		adr = page<<6;
		while(BIS(SPMCSR,SPMEN));	__boot_page_erase_normal(adr); while(BIS(SPMCSR,SPMEN));	
		while(BIS(SPMCSR,SPMEN));	__boot_page_write_normal(adr);
		while(BIS(SPMCSR,SPMEN));	SPMCSR = (1 << RWWSRE) | (1 << SPMEN);	asm volatile("spm");
		while(BIS(SPMCSR,SPMEN));
	}
	asm volatile("cli\n\tldi r31,0\n\tldi r30,0\n\tijmp"); // local reboot new app
}
#endif



/*
PORTC:
PC.0 to PC.4 - address for 32 ASICs for the decoders 138 ... chipselect
PC.5 - ADC input for the test device, Can be used to expand the box ID for the real device
PC.6 - reset, must be pulluped by 4.7k

PORTB:
PB.6 & PB.7 - crystal  7.378xxx MHz, actually 11.059,200 MHz
PB.0 & PB.1 - outputs to determine the ASIC difficulty, quazi Open drain, internally pulluped
PB.2 - SPI chip select for the test device, global SPI chip select for the real device, active 0, must be ext. pulluped by 4.7k
PB.3 - MOSI = SDO, tied to ASICS_SDI via level shifter
PB.4 - MISO = SDI, tied to ASICS_SDO via level shifter
PB.5 - SPI Clock  tied to ASICS_SPI_CLK_input via level shifter

PORTD:
PD.0 - UART Rx, pulluped input by 270 Ohm, because of baud=460800 bps
PD.1 - UART Tx
PD.7 - Band Selector, must be tied to the ASICs bs_signal, quazi Open drain, internally pulluped bla bla bla
PD.6 - PD.2 internaly pulluped inputs, BOX:Board_ID, can be expanded a bit ... PD.6-PD.2, total 5 bits = 32 boards
*/

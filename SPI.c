

#define SPI_CK 5
#define SPI_CS 2
#define SPI_DO 3

#define CHIP_REG_START              44
#define CHIP_PLL_REG				45
#define CHIP_REG_CLEAR              62
#define CHIP_REG_STATUS             63

#define CHIP_CMD_CHECK              0x00
#define CHIP_CMD_RESET              0xc0
#define CHIP_CMD_WRITE_REG(n)       (0x80 | (n))
#define CHIP_CMD_READ_REG(n)        (0x40 | (n))

#define CHIP_STAT_W_ALLOW           0x01
#define CHIP_STAT_R_READY           0x02
#define CHIP_STAT_TICKET_MASK       0x3c
extern BYTE bClk;

static void SetPllSPI(BYTE chip);
#define Set_BS()			sbi(PORTD,7)
#define Clr_BS()			cbi(PORTD,7)
#define GetAsic()	 		SPI_R()
#define Close_Asic()		sbi(PORTB,SPI_CS)

static inline void SPI_IO (BYTE out)	{ SPDR = out; while(!(SPSR & 0x80)); } // return SPDR; }
static inline BYTE SPI_R (void) 		{ SPDR = 0; while(!(SPSR & 0x80)); return SPDR; }
static inline void Open_nAsic(BYTE n)	{ /*sbi(PORTB,SPI_CS);*/ PORTC=n; cbi(PORTB,SPI_CS); /*_delay_us(5);*/  }

static inline void ASIC_Reset(BYTE ch)		{ Open_nAsic(ch); SPI_IO(CHIP_CMD_RESET); SPI_R(); _delay_us(30); SetPllSPI(ch); }
static inline void ASIC_ResetDelay(BYTE ch)	{ ASIC_Reset(ch);  _delay_us(200); }

static inline void HW_Reset(void)			{ sbi(PORTC,5); _delay_ms(200); cbi(PORTC,5); _delay_ms(100); }
static void ASIC_Reset_All(void)		{ 
	BYTE i;
	for(i=0;i<ASICS;i++)	{ Open_nAsic(i); SPI_IO(CHIP_CMD_RESET); SPI_R();  }
	_delay_us(100);
	for(i=0;i<ASICS;i++)	SetPllSPI(i); 
	_delay_us(300);
	}

static void PwrDown(BYTE ch) { Open_nAsic(ch); Astat[ch] |= 0x08; SPI_IO(CHIP_CMD_WRITE_REG(CHIP_PLL_REG)); SPI_IO(0x80); }

static void SetPllSPI(BYTE chip)  {
	BYTE ckk;

	Clr_BS();
	ckk = b_bclk[chip];
	Open_nAsic(chip);
	if(!ckk)	{ Astat[chip] |= 0x08; SPI_IO(CHIP_CMD_WRITE_REG(CHIP_PLL_REG)); SPI_IO(0x80); return; }
	else 		{ Astat[chip] &= 0xf7; SPI_IO(CHIP_CMD_WRITE_REG(CHIP_PLL_REG)); SPI_IO((ckk) | 0x80); }
	_delay_us(50);
	SPI_IO(CHIP_CMD_WRITE_REG(CHIP_PLL_REG)); SPI_IO(ckk);
}


static inline void Clr_Mask(void) { SPI_IO(CHIP_CMD_READ_REG(CHIP_REG_CLEAR));	SPI_R(); }

static BYTE GetNonce(BYTE *nonce, BYTE mask) {
	BYTE i, addr, res, *p;

	if(!(mask & 0xfe))		return 0;
	mask >>= 2;	mask &= 0x0f;
	if (!mask)				return 0;
	res = 0;
	
	p = nonce;
//	if(!g_dif) {	// diff=1
//		if (mask & 0x01)	{ addr = 46+0x40; for(i=0;i<4;i++) { SPI_IO(addr++); *p++ = SPI_R(); } res++; }
//	    if (mask & 0x02)	{ addr = 50+0x40; for(i=0;i<4;i++) { SPI_IO(addr++); *p++ = SPI_R(); } res++; }
//	    if (mask & 0x04)	{ addr = 54+0x40; for(i=0;i<4;i++) { SPI_IO(addr++); *p++ = SPI_R(); } res++; }
//	    if (mask & 0x08)	{ addr = 58+0x40; for(i=0;i<4;i++) { SPI_IO(addr++); *p++ = SPI_R(); } res++; }
//	}
//	else {
		if	(mask & 0x01)		addr = 46+0x40;
	    else if (mask & 0x02)	addr = 50+0x40;
	    else if (mask & 0x04)	addr = 54+0x40;
	    else					addr = 58+0x40;
	    for(i=0;i<4;i++) { SPI_IO(addr++); *p++ = SPI_R(); } res++;
//	}
   
	Clr_Mask();
	return res;
} 

static inline void Start_Job(void)	{ SPI_IO(0x80|CHIP_REG_START);	SPI_R(); }

static void SetBitDiff(BYTE df) {
	if		(df == 3)		{ sbi(PORTB,1); sbi(PORTB,0); }  // w = 256k;
    else if	(df == 2)		{ sbi(PORTB,1); cbi(PORTB,0); }  // w = 4096;
    else if	(df == 1)		{ cbi(PORTB,1); sbi(PORTB,0); }  // w =   64;
    else					{ cbi(PORTB,1); cbi(PORTB,0); }  // w =   1;
}


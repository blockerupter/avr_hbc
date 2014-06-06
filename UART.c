
static void PutcUART(BYTE bt)			{ while(!(UCSR0A & (1<<UDRE0)));  UDR0 = bt; }
static void PutnUART(BYTE *s, BYTE n)	{ while(n--)	PutcUART(*s++); }
#define IsUART()			 	(UCSR0A & (1<<RXC0))

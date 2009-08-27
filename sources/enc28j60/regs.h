/*
 * ENC28J60 Control Registers
 * Control register definitions are a combination of address,
 * bank number, and Ethernet/MAC/PHY indicator bits.
 * - Register address	(bits 0-4)
 * - Bank number	(bits 5-6)
 * - MAC/PHY indicator	(bit 7)
 */
#define ADDR_MASK		0x1F
#define BANK_MASK		0x60
#define SPRD_MASK		0x80

/* All-bank registers */
#define EIE			0x1B
#define EIR			0x1C
#define ESTAT			0x1D
#define ECON2			0x1E
#define ECON1			0x1F

/* Bank 0 registers */
#define ERDPTL			(0x00 | 0x00)
#define ERDPTH			(0x01 | 0x00)
#define EWRPTL			(0x02 | 0x00)
#define EWRPTH			(0x03 | 0x00)
#define ETXSTL			(0x04 | 0x00)
#define ETXSTH			(0x05 | 0x00)
#define ETXNDL			(0x06 | 0x00)
#define ETXNDH			(0x07 | 0x00)
#define ERXSTL			(0x08 | 0x00)
#define ERXSTH			(0x09 | 0x00)
#define ERXNDL			(0x0A | 0x00)
#define ERXNDH			(0x0B | 0x00)
#define ERXRDPTL		(0x0C | 0x00)
#define ERXRDPTH		(0x0D | 0x00)
#define ERXWRPTL		(0x0E | 0x00)
#define ERXWRPTH		(0x0F | 0x00)
#define EDMASTL			(0x10 | 0x00)
#define EDMASTH			(0x11 | 0x00)
#define EDMANDL			(0x12 | 0x00)
#define EDMANDH			(0x13 | 0x00)
#define EDMADSTL		(0x14 | 0x00)
#define EDMADSTH		(0x15 | 0x00)
#define EDMACSL			(0x16 | 0x00)
#define EDMACSH			(0x17 | 0x00)

/* Bank 1 registers */
#define EHT0			(0x00 | 0x20)
#define EHT1			(0x01 | 0x20)
#define EHT2			(0x02 | 0x20)
#define EHT3			(0x03 | 0x20)
#define EHT4			(0x04 | 0x20)
#define EHT5			(0x05 | 0x20)
#define EHT6			(0x06 | 0x20)
#define EHT7			(0x07 | 0x20)
#define EPMM0			(0x08 | 0x20)
#define EPMM1			(0x09 | 0x20)
#define EPMM2			(0x0A | 0x20)
#define EPMM3			(0x0B | 0x20)
#define EPMM4			(0x0C | 0x20)
#define EPMM5			(0x0D | 0x20)
#define EPMM6			(0x0E | 0x20)
#define EPMM7			(0x0F | 0x20)
#define EPMCSL			(0x10 | 0x20)
#define EPMCSH			(0x11 | 0x20)
#define EPMOL			(0x14 | 0x20)
#define EPMOH			(0x15 | 0x20)
#define ERXFCON			(0x18 | 0x20)
#define EPKTCNT			(0x19 | 0x20)

/* Bank 2 registers */
#define MACON1			(0x00 | 0x40 | 0x80)
#define MACON3			(0x02 | 0x40 | 0x80)
#define MACON4			(0x03 | 0x40 | 0x80)
#define MABBIPG			(0x04 | 0x40 | 0x80)
#define MAIPGL			(0x06 | 0x40 | 0x80)
#define MAIPGH			(0x07 | 0x40 | 0x80)
#define MACLCON1		(0x08 | 0x40 | 0x80)
#define MACLCON2		(0x09 | 0x40 | 0x80)
#define MAMXFLL			(0x0A | 0x40 | 0x80)
#define MAMXFLH			(0x0B | 0x40 | 0x80)
#define MICMD			(0x12 | 0x40 | 0x80)
#define MIREGADR		(0x14 | 0x40 | 0x80)
#define MIWRL			(0x16 | 0x40 | 0x80)
#define MIWRH			(0x17 | 0x40 | 0x80)
#define MIRDL			(0x18 | 0x40 | 0x80)
#define MIRDH			(0x19 | 0x40 | 0x80)

/* Bank 3 registers */
#define MAADR5			(0x00 | 0x60 | 0x80)
#define MAADR6			(0x01 | 0x60 | 0x80)
#define MAADR3			(0x02 | 0x60 | 0x80)
#define MAADR4			(0x03 | 0x60 | 0x80)
#define MAADR1			(0x04 | 0x60 | 0x80)
#define MAADR2			(0x05 | 0x60 | 0x80)
#define EBSTSD			(0x06 | 0x60)
#define EBSTCON			(0x07 | 0x60)
#define EBSTCSL			(0x08 | 0x60)
#define EBSTCSH			(0x09 | 0x60)
#define MISTAT			(0x0A | 0x60 | 0x80)
#define EREVID			(0x12 | 0x60)
#define ECOCON			(0x15 | 0x60)
#define EFLOCON			(0x17 | 0x60)
#define EPAUSL			(0x18 | 0x60)
#define EPAUSH			(0x19 | 0x60)

/* PHY registers */
#define PHCON1			0x00
#define PHSTAT1			0x01
#define PHID1			0x02
#define PHID2			0x03
#define PHCON2			0x10
#define PHSTAT2			0x11
#define PHIE			0x12
#define PHIR			0x13
#define PHLCON			0x14

/* ENC28J60 EIE Register Bit Definitions */
#define EIE_INTIE		0x80
#define EIE_PKTIE		0x40
#define EIE_DMAIE		0x20
#define EIE_LINKIE		0x10
#define EIE_TXIE		0x08
#define EIE_TXERIE		0x02
#define EIE_RXERIE		0x01

/* ENC28J60 EIR Register Bit Definitions */
#define EIR_PKTIF		0x40
#define EIR_DMAIF		0x20
#define EIR_LINKIF		0x10
#define EIR_TXIF		0x08
#define EIR_TXERIF		0x02
#define EIR_RXERIF		0x01
#define EIR_BITS		\
"\20\1rxerif\2txerif\4txif\5linkif\6dmaif\7pktif"

/* ENC28J60 ESTAT Register Bit Definitions */
#define ESTAT_INT		0x80
#define ESTAT_BUFER		0x40
#define ESTAT_LATECOL		0x10
#define ESTAT_RXBUSY		0x04
#define ESTAT_TXABRT		0x02
#define ESTAT_CLKRDY		0x01
#define ESTAT_BITS		\
"\20\1clkrdy\2txabrt\3rxbusy\5latecol\7bufer\10int"

/* ENC28J60 ECON2 Register Bit Definitions */
#define ECON2_AUTOINC		0x80
#define ECON2_PKTDEC		0x40
#define ECON2_PWRSV		0x20
#define ECON2_VRPS		0x08

/* ENC28J60 ECON1 Register Bit Definitions */
#define ECON1_TXRST		0x80
#define	ECON1_RXRST		0x40
#define ECON1_DMAST		0x20
#define ECON1_CSUMEN		0x10
#define ECON1_TXRTS		0x08
#define	ECON1_RXEN		0x04
#define ECON1_BSEL1		0x02
#define ECON1_BSEL0		0x01

/* ENC28J60 MACON1 Register Bit Definitions */
#define MACON1_TXPAUS		0x08
#define MACON1_RXPAUS		0x04
#define MACON1_PASSALL		0x02
#define MACON1_MARXEN		0x01

/* ENC28J60 MACON3 Register Bit Definitions */
#define MACON3_PADCFG2		0x80
#define MACON3_PADCFG1		0x40
#define MACON3_PADCFG0		0x20
#define MACON3_TXCRCEN		0x10
#define MACON3_PHDREN		0x08
#define MACON3_HFRMEN		0x04
#define MACON3_FRMLNEN		0x02
#define MACON3_FULDPX		0x01

/* ENC28J60 MACON4 Register Bit Definitions */
#define MACON4_DEFER		0x40
#define MACON4_BPEN		0x20
#define MACON4_NOBKOFF		0x10

/* ENC28J60 MICMD Register Bit Definitions */
#define MICMD_MIISCAN		0x02
#define MICMD_MIIRD		0x01

/* ENC28J60 MISTAT Register Bit Definitions */
#define MISTAT_NVALID		0x04
#define MISTAT_SCAN		0x02
#define MISTAT_BUSY		0x01

/* ENC28J60 PHY PHCON1 Register Bit Definitions */
#define	PHCON1_PRST		0x8000
#define	PHCON1_PLOOPBK		0x4000
#define	PHCON1_PPWRSV		0x0800
#define	PHCON1_PDPXMD		0x0100

/* ENC28J60 PHY PHSTAT1 Register Bit Definitions */
#define	PHSTAT1_PFDPX		0x1000
#define	PHSTAT1_PHDPX		0x0800
#define	PHSTAT1_LLSTAT		0x0004
#define	PHSTAT1_JBSTAT		0x0002
#define PHSTAT1_BITS		\
"\20\02jbstat\03llstat\14phdpx\15pfdpx"

/* ENC28J60 PHY PHSTAT2 Register Bit Definitions */
#define	PHSTAT2_TXSTAT		0x2000
#define	PHSTAT2_RXSTAT		0x1000
#define	PHSTAT2_COLSTAT		0x0800
#define	PHSTAT2_LSTAT		0x0400
#define	PHSTAT2_DPXSTAT		0x0200
#define	PHSTAT2_PLRITY		0x0010
#define PHSTAT2_BITS		\
"\20\05plrity\12dpxstat\13lstat\14colstat\15rxstat\16txstat"

/* ENC28J60 PHY PHCON2 Register Bit Definitions */
#define PHCON2_FRCLINK		0x4000
#define PHCON2_TXDIS		0x2000
#define PHCON2_JABBER		0x0400
#define PHCON2_HDLDIS		0x0100

/* ENC28J60 Packet Control Byte Bit Definitions */
#define PKTCTRL_PHUGEEN		0x08
#define PKTCTRL_PPADEN		0x04
#define PKTCTRL_PCRCEN		0x02
#define PKTCTRL_POVERRIDE	0x01

/* ENC28J60 ERXFCON Register Bit Definitions */
#define ERXFCON_UCEN		0x80
#define ERXFCON_ANDOR		0x40
#define ERXFCON_CRCEN		0x20
#define ERXFCON_PMEN		0x10
#define ERXFCON_MPEN		0x08
#define ERXFCON_HTEN		0x04
#define ERXFCON_MCEN		0x02
#define ERXFCON_BCEN		0x01

/* SPI operation codes */
#define ENC28J60_READ_CTRL_REG	0x00
#define ENC28J60_READ_BUF_MEM	0x3A
#define ENC28J60_WRITE_CTRL_REG	0x40
#define ENC28J60_WRITE_BUF_MEM	0x7A
#define ENC28J60_BIT_FIELD_SET	0x80
#define ENC28J60_BIT_FIELD_CLR	0xA0
#define ENC28J60_SOFT_RESET	0xFF

/* buffer boundaries applied to internal 8K ram
 * entire available packet buffer space is allocated */
#define TXSTART_INIT		0x0000	/* start TX buffer at 0 */
#define RXSTART_INIT		0x0600	/* give TX buffer space for one full ethernet frame (~1500 bytes) */
#define RXSTOP_INIT		0x1FFF	/* receive buffer gets the rest */

/* ENC28J60 upper 16 bits of Receive Status Vector */
#define RSV_RXLONGEVDROPEV	0x0001
#define RSV_CARRIEREV		0x0004
#define RSV_CRCERROR		0x0010
#define RSV_LENCHECKERR		0x0020
#define RSV_LENOUTOFRANGE	0x0040
#define RSV_RXOK		0x0080
#define RSV_RXMULTICAST		0x0100
#define RSV_RXBROADCAST		0x0200
#define RSV_DRIBBLENIBBLE	0x0400
#define RSV_RXCONTROLFRAME	0x0800
#define RSV_RXPAUSEFRAME	0x1000
#define RSV_RXUNKNOWNOPCODE	0x2000
#define RSV_RXTYPEVLAN		0x4000

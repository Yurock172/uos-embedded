/*
 * Driver for Nokia 5110 display.
 * See Philips PCD8544 datasheet.
 */
#include <runtime/lib.h>
#include <stream/stream.h>
#include <gpanel/gpanel.h>

#ifdef LCD_SPI
#ifdef ARM_STM32F10x
#include <stm32f10x/spi.h>
#endif
#ifdef ARM_STM32F4
#include <stm32f4/spi.h>
#endif

#ifdef ARM_STM32F10x
stm32_spim_t spi;
#endif

#ifdef ARM_STM32F4
stm32f4_spim_t spi;
#endif

spimif_t *spimif;
spi_message_t msg;
uint8_t spiCh;

#if !defined(LCD_DC) || !defined(LCD_BL) || !defined(LCD_RST)
#   error "LCD pins  and ports is not assigned in CFLAGS of target.cfg"
#endif

#endif

/*
 * Pinout for SainSmart Graphic LCD4884 Shield.
 */
#ifdef PIC32MX2                     /* Olimex Pinguino-mx220 */
#define MASKC_LCD_SCK   (1 << 2)    /* signal D2, pin RC2 */
#define MASKC_LCD_MOSI  (1 << 3)    /* signal D3, pin RC3 */
#define MASKC_LCD_DC    (1 << 4)    /* signal D4, pin RC4 */
#define MASKC_LCD_CS    (1 << 5)    /* signal D5, pin RC5 */
#define MASKC_LCD_RST   (1 << 6)    /* signal D6, pin RC6 */
#define MASKC_LCD_BL    (1 << 7)    /* signal D7, pin RC7 */
#endif

#ifdef PIC32MX1                     /* Firewing board */
#define MASKB_LCD_SCK   (1 << 6)    /* signal D2, pin RB6 */
#define MASKB_LCD_MOSI  (1 << 7)    /* signal D3, pin RB7 */
#define MASKA_LCD_DC    (1 << 4)    /* signal D4, pin RA4 */
#define MASKB_LCD_CS    (1 << 8)    /* signal D5, pin RB8 */
#define MASKB_LCD_RST   (1 << 9)    /* signal D6, pin RB9 */
#define MASKA_LCD_BL    (1 << 3)    /* signal D7, pin RA3 */
#endif

#define MAXROW  48
#define MAXCOL  84

static unsigned char gpanel_screen [MAXROW*MAXCOL/8];

static inline void lcd_cs (unsigned on)
{
    if (on) {
#ifdef LCD_SPI

#else
#ifdef MASKC_LCD_CS
        LATCSET = MASKC_LCD_CS;
#else
        LATBSET = MASKB_LCD_CS;
#endif
#endif
    } else {
#ifdef LCD_SPI

#else
#ifdef MASKC_LCD_CS
        LATCCLR = MASKC_LCD_CS;
#else
        LATBCLR = MASKB_LCD_CS;
#endif
#endif
    }
}

static inline void lcd_rst (unsigned on)
{
    if (on) {
#ifdef LCD_RST
    	LCD_RST_PORT->BSRR = GPIO_SET(PIN(LCD_RST));
#else
#ifdef MASKC_LCD_RST
        LATCSET = MASKC_LCD_RST;
#else
        LATBSET = MASKB_LCD_RST;
#endif
#endif
    } else {
#ifdef LCD_RST
    	LCD_RST_PORT->BSRR = GPIO_RESET(PIN(LCD_RST));
#else
#ifdef MASKC_LCD_RST
        LATCCLR = MASKC_LCD_RST;
#else
        LATBCLR = MASKB_LCD_RST;
#endif
#endif
    }
}

static inline void lcd_dc (unsigned on)
{
    if (on) {
#ifdef LCD_DC
    	LCD_DC_PORT->BSRR = GPIO_SET(PIN(LCD_DC));
#else
#ifdef MASKC_LCD_DC
        LATCSET = MASKC_LCD_DC;
#else
        LATASET = MASKA_LCD_DC;
#endif
#endif
    } else {
#ifdef LCD_DC
    	LCD_DC_PORT->BSRR = GPIO_RESET(PIN(LCD_DC));
#else
#ifdef MASKC_LCD_DC
        LATCCLR = MASKC_LCD_DC;
#else
        LATACLR = MASKA_LCD_DC;
#endif
#endif
    }
}

static inline void lcd_bl (unsigned on)
{
    if (on) {
#ifdef LCD_BL
    	LCD_BL_PORT->BSRR = GPIO_SET(PIN(LCD_BL));
#else
#ifdef MASKC_LCD_BL
        LATCSET = MASKC_LCD_BL;
#else
        LATASET = MASKA_LCD_BL;
#endif
#endif
    } else {
#ifdef LCD_BL
    	LCD_BL_PORT->BSRR = GPIO_RESET(PIN(LCD_BL));
#else
#ifdef MASKC_LCD_BL
        LATCCLR = MASKC_LCD_BL;
#else
        LATACLR = MASKA_LCD_BL;
#endif
#endif
    }
}

static inline void lcd_mosi (unsigned on)
{
#ifndef LCD_SPI
    if (on) {
#ifdef MASKC_LCD_MOSI
        LATCSET = MASKC_LCD_MOSI;
#else
        LATBSET = MASKB_LCD_MOSI;
#endif
    } else {
#ifdef MASKC_LCD_MOSI
        LATCCLR = MASKC_LCD_MOSI;
#else
        LATBCLR = MASKB_LCD_MOSI;
#endif
    }
#else
    (void) on;
#endif
}

static inline void lcd_sck (unsigned on)
{
#ifndef LCD_SPI
    if (on) {
#ifdef MASKC_LCD_SCK
        LATCSET = MASKC_LCD_SCK;
#else
        LATBSET = MASKB_LCD_SCK;
#endif
    } else {
#ifdef MASKC_LCD_SCK
        LATCCLR = MASKC_LCD_SCK;
#else
        LATBCLR = MASKB_LCD_SCK;
#endif
    }
#else
    (void) on;
#endif

}

static void lcd_write (unsigned byte, unsigned data_flag)
{
#ifdef LCD_SPI
	lcd_dc (data_flag);
	spiCh = byte;
	msg.word_count = 1;
	msg.tx_data = &spiCh;
	spimif->trx(&spi.spimif, &msg);
#else
    unsigned i;

    lcd_cs (0);
    lcd_dc (data_flag);
    for (i=0; i<8; i++, byte<<=1) {
        lcd_mosi (byte & 0x80);         /* SDIN = bit[i] */
        lcd_sck (0);                    /* SCLK = 0 */
        lcd_sck (1);                    /* SCLK = 1 */
    }
    lcd_cs (1);
#endif
}

/*
 * Fill the LCD screen with a given color: black or white.
 */
void gpanel_clear (gpanel_t *gp, unsigned color)
{
    unsigned i;

    if (color)
        color = 0xFF;
    else
        color = 0;

    /* Clear data */
    lcd_write (0x40, 0);
    lcd_write (0x80, 0);
#ifdef LCD_SPI
    for (i=0; i<MAXROW*MAXCOL/8; i++) {
    	gpanel_screen[i] = color;
    	lcd_write (color, 1);
    }
//    msg.tx_data = gpanel_screen;
//    msg.word_count = MAXROW*MAXCOL/8;
//    spimif->trx(&spi.spimif, &msg);
//
//    msg.tx_data = &spiCh;
//    msg.word_count = MAXROW*MAXCOL/8;

#else
    for (i=0; i<MAXROW*MAXCOL/8; i++) {
        gpanel_screen[i] = color;
        lcd_write (color, 1);
    }
#endif
    gp->row = 0;
    gp->col = 0;
}

/*
 * Set up hardware for communication to Nokia 5110 LCD Display.
 * Do not clear the display.
 * Leave backlight turned off.
 */
void gpanel_init (gpanel_t *gp, const gpanel_font_t *font)
{
    extern stream_interface_t gpanel_interface;

    gp->interface = &gpanel_interface;
    gp->nrow = MAXROW;
    gp->ncol = MAXCOL;
    gp->font = font;
    gp->foreground = GPANEL_WHITE;
    gp->background = GPANEL_BLACK;
    gp->contrast = 0x38;
    gp->row = 0;
    gp->col = 0;
    gp->c1 = 0;
    gp->c2 = 0;

    /*
     * Set pins as outputs.
     */
#ifdef PIC32MX2                     /* Olimex Pinguino-mx220 */
    LATCSET = MASKC_LCD_RST | MASKC_LCD_CS;
    TRISCCLR = MASKC_LCD_SCK | MASKC_LCD_MOSI | MASKC_LCD_DC |
               MASKC_LCD_CS  | MASKC_LCD_RST  | MASKC_LCD_BL;
#endif

#ifdef PIC32MX1                     /* Firewing board */
    LATBSET = MASKB_LCD_RST | MASKB_LCD_CS;
    TRISACLR = MASKA_LCD_DC | MASKA_LCD_BL;
    TRISBCLR = MASKB_LCD_SCK | MASKB_LCD_MOSI |
               MASKB_LCD_CS  | MASKB_LCD_RST;
#endif

#ifdef LCD_SPI

#ifdef ARM_STM32F4

//    gpio_bl->CR[PIN(LCD_BL)>>3] = (gpio_bl->CR[PIN(LCD_BL)>>3] & ~GPIO_MASK(PIN(LCD_BL))) | GPIO_OUT_PP_10MHz(PIN(LCD_BL));
//
//    gpio_dc->CR[PIN(LCD_DC)>>3] = (gpio_dc->CR[PIN(LCD_DC)>>3] & ~GPIO_MASK(PIN(LCD_DC))) | GPIO_OUT_PP_10MHz(PIN(LCD_DC));
//    gpio_rst->CR[PIN(LCD_RST)>>3] = (gpio_rst->CR[PIN(LCD_RST)>>3] & ~GPIO_MASK(PIN(LCD_RST))) | GPIO_OUT_PP_10MHz(PIN(LCD_RST));


//    stm32f4_config_pin(gpio_bl, LCD_BL, SET_CONFIG_MODER(GPIO_MODER_OUT) |
//    		                            SET_CONFIG_OTYPER(GPIO_OTYPER_PP) | SET_CONFIG_OSPEEDR(GPIO_OSPEEDR_50MHz) |
//			                            SET_CONFIG_PUPDR(GPIO_PUPDR_NO_PULL) );
//    stm32f4_config_pin(gpio_dc, LCD_DC, GPIO_MODER_OUT   | GPIO_OTYPER_PP | GPIO_50MHz(0) | GPIO_NO_PULL(0) | GPIO_RESET(0));
//    stm32f4_config_pin(gpio_rst, LCD_RST, GPIO_MODER_OUT | GPIO_OTYPER_PP | GPIO_50MHz(0) | GPIO_NO_PULL(0) | GPIO_RESET(0));



    stm32f4_spi_init(&spi, LCD_SPI, 0);
#endif

    msg.tx_data = &spiCh;
    msg.mode = SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_MODE_NB_BITS(8);
    msg.freq = 3000000;
    msg.word_count = 1;
    spimif = (spimif_t*)&spi.spimif;
#endif

    /* Turn off backlight. */
    lcd_bl (0);

    /* Reset the display. */
    lcd_rst (0);
    udelay (1);
    lcd_rst (1);
    udelay (1);

    lcd_write (0x21, 0);    // Enable extended instruction set
    lcd_write (0xbf, 0);    // Set Vop - contrast level
    lcd_write (0x04, 0);    // Set temperature coefficient to 0
    lcd_write (0x14, 0);    // Set bias to 4
    lcd_write (0x20, 0);    // Back to normal instruction set
    lcd_write (0x0c, 0);    // Set normal mode

//    lcd_write (0x0b, 0);    // Set display configuration
}

/*
 * Lights a single pixel in the specified color
 * at the specified x and y addresses
 */
void gpanel_pixel (gpanel_t *gp, int x, int y, int color)
{
    unsigned char *data;

    if (x >= gp->ncol || y >= gp->nrow)
        return;
    data = &gpanel_screen [(y >> 3) * MAXCOL + x];

    if (color)
        *data |= 1 << (y & 7);
    else
        *data &= ~(1 << (y & 7));

    lcd_write (0x40 | (y >> 3), 0);
    lcd_write (0x80 | x, 0);
    lcd_write (*data, 1);
}

/*
 * Draw a part of glyph, up to 8 pixels in height.
 */
static void graw_glyph8 (gpanel_t *gp, unsigned width, unsigned height,
    const unsigned short *bits, unsigned ypage, unsigned yoffset)
{
    unsigned char *data;
    unsigned x, y;

    if (height > 8 - yoffset)
        height = 8 - yoffset;
    if (width > 32)
        width = 32;
    data = &gpanel_screen [ypage * MAXCOL + gp->col % MAXCOL];
    /*debug_printf ("glyph8 %ux%u at %u-%u offset %u-%u\n", width, height, gp->col, gp->row, ypage, yoffset);*/

    /* Clear glyph background. */
    if (yoffset == 0 && height == 8) {
        for (x=0; x<width; x++)
            data[x] = 0;
    } else {
        unsigned mask = ~(((1 << height) - 1) << yoffset);
        for (x=0; x<width; x++)
            data[x] &= mask;
    }

    /* Place glyph image. */
    unsigned words_per_row = (width + 15) / 16;
    for (x=0; x<width; x++) {
        for (y=0; y<height; y++) {
            if (bits [y*words_per_row + x/16] & (0x8000 >> (x & 15))) {
                data[x] |= 1 << (y + yoffset);
            }
        }
    }

    /* Write graphics memory. */
    lcd_write (0x40 | ypage, 0);
    lcd_write (0x80 | gp->col, 0);
    for (x=0; x<width; x++) {
        lcd_write (data[x], 1);
    }
}

/*
 * Draw a glyph of one symbol.
 */
void gpanel_glyph (gpanel_t *gp, unsigned width, const unsigned short *bits)
{
    unsigned ypage = gp->row >> 3;
    unsigned yoffset = gp->row & 7;
    unsigned words_per_row = (width + 15) / 16;
    int height = gp->font->height;

    /*debug_printf ("<glyph %d at %d-%d>", width, gp->col, gp->row);*/
    for (;;) {
        graw_glyph8 (gp, width, height, bits, ypage, yoffset);
        height -= 8 - yoffset;
        if (height <= 0)
            break;
        ypage++;
        if (ypage >= 8)
            break;
        bits += (8 - yoffset) * words_per_row;
        yoffset = 0;
    }
}

/*
 * Draw a filled rectangle in the specified color from (x1,y1) to (x2,y2).
 *
 * The best way to fill a rectangle is to take advantage of the "wrap-around" featute
 * built into the Philips PCF8833 controller. By defining a drawing box, the memory can
 * be simply filled by successive memory writes until all pixels have been illuminated.
 */
void gpanel_rect_filled (gpanel_t *gp, int x0, int y0, int x1, int y1, int color)
{
    /* Temporary solution */
    int xmin, xmax, ymin, ymax, x, y;

    /* calculate the min and max for x and y directions */
    if (x0 <= x1) {
        xmin = x0;
        xmax = x1;
    } else {
        xmin = x1;
        xmax = x0;
    }
    if (y0 <= y1) {
        ymin = y0;
        ymax = y1;
    } else {
        ymin = y1;
        ymax = y0;
    }
    for (y=ymin; y<=ymax; y++)
        for (x=xmin; x<=xmax; x++)
            gpanel_pixel (gp, x, y, color);
}

/*
 * Contrast control.
 */
void gpanel_contrast (gpanel_t *gp, int contrast)
{
    lcd_write (0x21, 0);            // Enable extended instruction set
    lcd_write (0x80 | contrast, 0); // Set Vop - contrast level
    lcd_write (0x20, 0);            // Back to normal instruction set
}

/*
 * Backlight control.
 */
void gpanel_backlight (gpanel_t *gp, int on)
{
    lcd_bl (on);
}

/*
 * Write an image to LCD screen.
 */
void gpanel_image (gpanel_t *gp, int x, int y, int width, int height,
    const unsigned short *data)
{
    // TODO
}

/* выключение дисплея, перевод его в спящий режим */
void gpanel_off(gpanel_t *lcd) {
	// ToDo
}

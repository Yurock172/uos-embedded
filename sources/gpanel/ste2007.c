/*
 * Драйвер для дисплея nokia 1202/1203/1260 на контроллере ste2007
 *
 *  Created on: 20 июня 2019 г.
 *      Author: yurock
 */

#include <runtime/lib.h>
#include <stream/stream.h>
#include <gpanel/gpanel.h>


#ifdef ARM_STM32F10x
#include <stm32f10x/spi.h>
stm32_spim_t spi;
#endif

#ifdef ARM_STM32F4
//#include <stm32f4/spi.h>
//stm32f4_spim_t spi;
#include <stm32f4/gpio.h>
stm32f4_gpio_t pin_rst;
stm32f4_gpio_t pin_led;
stm32f4_gpio_t pin_clk;
stm32f4_gpio_t pin_cs;
stm32f4_gpio_t pin_data;
#endif

#ifdef ARM_STM32L1
//#include <stm32l1/spi.h>
#include <stm32l1/gpio.h>
//stm32l1_spim_t spi;
stm32l1_gpio_t pin_rst;
stm32l1_gpio_t pin_led;
stm32l1_gpio_t pin_clk;
stm32l1_gpio_t pin_cs;
stm32l1_gpio_t pin_data;
#endif


//spimif_t *spimif;
//spi_message_t msg;
//uint16_t spiCh;

#if !defined(LCD_LED_PIN) || !defined(LCD_LED_PORT) || !defined(LCD_RST_PIN) || !defined(LCD_RST_PORT)
#   error "LCD pins (LCD_LED_PIN and LCD_RST_PIN)  and ports (LCD_LED_PORT and LCD_LED_RST_PORT) is not assigned in CFLAGS of target.cfg"
#endif

#define COMMAND_DISPLAY_ON					0xAF		/* Включить дисплей */
#define COMMAND_DISPLAY_OFF					0xAE		/* Выключить дисплей */

#define COMMAND_DISPLAY_NORMAL				0xA6		/* Негативное изображение */
#define COMMAND_DISPLAY_REVERSE				0xA7		/* Позитивное изображение */

#define COMMAND_DISPLAY_POINTS_ON			0xA5		/* all points ON */
#define COMMAND_DISPLAY_POINTS_OFF			0xA4		/* normal display */

#define DISPLAY_SET_PAGE_ADDRES				0xB0		/* Установка Y */
#define DISPLAY_SET_COLUMN_ADDRES			0x10		/* Установка X */

#define DISPLAY_SET_START_LINE				0x40

#define DISPLAY_SEGMENT_DIRECTION_NORMAL	0xA0		/* Направление вывода сегмента нормальное */
#define DISPLAY_SEGMENT_DIRECTION_REVERSE	0xA1		/* Направление вывода сегмента инверсное */

#define DISPLAY_COMMON_DIRECTION_NORMAL		0xC0		/* Направление вывода нормальное */
#define DISPLAY_COMMON_DIRECTION_REVERSE	0xC8		/* Направление вывода инверсное */

#define DISPLAY_SELF_TEST					0xDB
#define DISPLAY_POWER_CONTROLL_SET			0x28
#define DISLPAY_VO_RANGE					0x20		/* контраст */
#define DISLPAY_ELECTRONIC_VOLUM			0x80		/* Контраст */

#define DISPLAY_INTERNAL_RESET				0xE2
#define DISPLAY_NOP							0xE3
#define COMMAND_DISPLAY_SET_VOP				0xE1		/* Сначала комманда, затем 8-ми битное значение */
#define COMMAND_TERMAL_COMPENSATION			0x38		/* Сначала комманда, следующая посылка с 3-х бинтым значением */

#define MAX_ROW								9			/* x */
#define MAX_COL  							98			/* y */

#define POINT_XMAX							96
#define POINT_YMAX							68

static unsigned char gpanel_screen [POINT_XMAX * POINT_YMAX / 8];




static inline void lcd_cs (unsigned on) {

}

static inline void lcd_rst (unsigned on)
{
	gpio_set_val(&pin_rst.gpioif, (int)on);
}

static inline void lcd_bl (unsigned on)
{
	gpio_set_val(&pin_led.gpioif, on);
}

static void lcd_write (uint8_t byte, unsigned data_flag) {
	gpio_set_val(&pin_cs.gpioif, 0);		/* Slave select */

	if (data_flag) {
		gpio_set_val(&pin_data.gpioif, 1);
	} else
		gpio_set_val(&pin_data.gpioif, 0);
	gpio_set_val(&pin_clk.gpioif, 1);
	for (uint32_t i = 0; i < 8; i++) {
		gpio_set_val(&pin_clk.gpioif, 0);
		if (byte & 0x80)
			gpio_set_val(&pin_data.gpioif, 1);
		else
			gpio_set_val(&pin_data.gpioif, 0);
		gpio_set_val(&pin_clk.gpioif, 1);
		byte <<= 1;
	}

	gpio_set_val(&pin_cs.gpioif, 1);
	gpio_set_val(&pin_clk.gpioif, 0);
}

static void lcd_go_to_xy(uint8_t x, uint8_t y) {
	lcd_write((DISPLAY_SET_PAGE_ADDRES | (y & 0x0F)), 0);
	lcd_write((DISPLAY_SET_COLUMN_ADDRES | (x >> 0x04)), 0);
	lcd_write(0x0F & x, 0);
}

/*
 * Fill the LCD screen with a given color: black or white.
 */
void gpanel_clear (gpanel_t *gp, unsigned color)
{
    unsigned i, j;

    if (color)
        color = 0xFF;
    else
        color = 0;

    /* Clear data */
    // Set {0, 0}
    lcd_write(COMMAND_DISPLAY_OFF, 0);
    for (i = 0; i < MAX_ROW; i++) {
    	lcd_go_to_xy(0, i);
    	for (j = 0; j < MAX_COL; j++) {
    		lcd_write (color, 1);
    	}
    }
    lcd_go_to_xy(0, 0);

    lcd_write(COMMAND_DISPLAY_ON, 0);

    for (i = 0; i < POINT_XMAX * POINT_YMAX / 8; i++) {
    	gpanel_screen[i] = color;
    }
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
    debug_printf("Init lsd ste2007\r\n");

    gp->interface = &gpanel_interface;
    gp->nrow = POINT_YMAX;
    gp->ncol = POINT_XMAX;
    gp->font = font;
    gp->foreground = GPANEL_WHITE;
    gp->background = GPANEL_BLACK;
    gp->contrast = 0x11;
    gp->row = 0;
    gp->col = 0;
    gp->c1 = 0;
    gp->c2 = 0;

    /*
     * Set pins as outputs.
     */

#ifdef ARM_STM32F4
    stm32f4_gpio_init(&pin_rst,  LCD_RST_PORT, LCD_RST_PIN, GPIO_FLAGS_OUTPUT | GPIO_FLAGS_50MHZ);
    stm32f4_gpio_init(&pin_led,  LCD_LED_PORT, LCD_LED_PIN, GPIO_FLAGS_OUTPUT | GPIO_FLAGS_50MHZ);
    stm32f4_gpio_init(&pin_cs,   LCD_CS_PORT,  LCD_CS_PIN,  GPIO_FLAGS_OUTPUT | GPIO_FLAGS_100MHZ);
    stm32f4_gpio_init(&pin_clk,  LCD_CLK_PORT, LCD_CLK_PIN, GPIO_FLAGS_OUTPUT | GPIO_FLAGS_100MHZ);
    stm32f4_gpio_init(&pin_data, LCD_SDA_PORT, LCD_SDA_PIN, GPIO_FLAGS_OUTPUT | GPIO_FLAGS_100MHZ);
    gpio_set_val(&pin_data.gpioif, 0);
    gpio_set_val(&pin_clk.gpioif, 0);
    gpio_set_val(&pin_cs.gpioif, 0);
    gpio_set_val(&pin_rst.gpioif, 0);
#endif

#ifdef ARM_STM32L1

    stm32l1_gpio_init(&pin_rst,  LCD_RST_PORT, LCD_RST_PIN, GPIO_FLAGS_OUTPUT | GPIO_FLAGS_100MHZ);
    stm32l1_gpio_init(&pin_led,  LCD_LED_PORT, LCD_LED_PIN, GPIO_FLAGS_OUTPUT | GPIO_FLAGS_100MHZ);
    stm32l1_gpio_init(&pin_cs,   LCD_CS_PORT,  LCD_CS_PIN,  GPIO_FLAGS_OUTPUT | GPIO_FLAGS_100MHZ);
    stm32l1_gpio_init(&pin_clk,  LCD_CLK_PORT, LCD_CLK_PIN, GPIO_FLAGS_OUTPUT | GPIO_FLAGS_100MHZ);
    stm32l1_gpio_init(&pin_data, LCD_SDA_PORT, LCD_SDA_PIN, GPIO_FLAGS_OUTPUT | GPIO_FLAGS_100MHZ);
    gpio_set_val(&pin_data.gpioif, 0);
    gpio_set_val(&pin_clk.gpioif, 0);
    gpio_set_val(&pin_cs.gpioif, 0);
    gpio_set_val(&pin_rst.gpioif, 0);

#endif
//
//    msg.tx_data = &spiCh;
//    msg.mode = SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_MODE_NB_BITS(16);
//    msg.freq = 3000000;
//    msg.word_count = 1;
//    spimif = (spimif_t*)&spi.spimif;

    /* Turn off backlight. */
    lcd_bl (0);

    mdelay(30);
    lcd_rst (1);
    gpio_set_val(&pin_cs.gpioif, 1);
    udelay(5);
    /* Reset the display. */
    lcd_rst (0);
    mdelay (15);
    lcd_rst (1);
    mdelay (2);

    lcd_write (DISPLAY_INTERNAL_RESET, 0);// Internal reset
    mdelay(3);
// для выключения дисплея отправить на него команды 0xA5, 0xAE

    lcd_write (0xEB, 0);    // Thermal comp. on
    mdelay(3);
    lcd_write (DISPLAY_POWER_CONTROLL_SET | 0x7, 0);    // Supply mode
    lcd_write (DISPLAY_COMMON_DIRECTION_REVERSE, 0);
    lcd_write (DISPLAY_SEGMENT_DIRECTION_REVERSE, 0);    //

    lcd_write (COMMAND_DISPLAY_POINTS_OFF, 0);    // Clear screen
    lcd_write (COMMAND_DISPLAY_NORMAL, 0);    // Positive - A7, Negative - A6
    lcd_write(DISLPAY_ELECTRONIC_VOLUM | gp->contrast, 0);
    lcd_write(COMMAND_DISPLAY_ON, 0); 		// Enable LCD
    lcd_bl (1);
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
	data = &gpanel_screen [(y >> 3) * POINT_XMAX + x];

	if (color)
		*data |= 1 << (y & 7);
	else
		*data &= ~(1 << (y & 7));

//	lcd_write (0x40 | (y >> 3), 0);
//	lcd_write (0x80 | x, 0);
	gpanel_move(gp, x, y);
	lcd_go_to_xy(x, y >> 3);
//	debug_printf("x = %d, y = %d\r\n", x, y);

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
	 data = &gpanel_screen [ypage * POINT_XMAX + gp->col % POINT_XMAX];
	 /*debug_printf ("glyph8 %ux%u at %u-%u offset %u-%u\n", width, height, gp->col, gp->row, ypage, yoffset);*/

	 /* Clear glyph background. */
	 if (yoffset == 0 && height == 8) {
		 for (x = 0; x < width; x++)
			 data[x] = 0;
	 } else {
		 unsigned mask = ~(((1 << height) - 1) << yoffset);
		 for (x = 0; x < width; x++)
			 data[x] &= mask;
	 }

	 /* Place glyph image. */
	 unsigned words_per_row = (width + 15) / 16;
	 for (x = 0; x < width; x++) {
		 for (y = 0; y < height; y++) {
			 if (bits [y * words_per_row + x/16] & (0x8000 >> (x & 15))) {
				 data[x] |= 1 << (y + yoffset);
			 }
		 }
	 }

	 /* Write graphics memory. */
	 lcd_go_to_xy(gp->col, ypage);
//	 debug_printf("lcd goto {%d, %d}\r\n", gp->col, ypage);
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
		for (x=xmin; x<=xmax; x++) // Линия по оси х
			gpanel_pixel (gp, x, y, color);
}

/*
 * Contrast control.
 */
void gpanel_contrast (gpanel_t *gp, int contrast)
{
	if ( (contrast < 0) || (contrast > 31) )
		contrast = 10;
	gp->contrast = contrast;
	lcd_write(DISLPAY_ELECTRONIC_VOLUM | gp->contrast, 0);
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
	gpanel_move(gp, x, y);
	unsigned ypage = gp->row >> 3;
	unsigned yoffset = gp->row & 7;
	unsigned words_per_row = (width + 15) / 16;
	//	int height = gp->font->height;


	/*debug_printf ("<glyph %d at %d-%d>", width, gp->col, gp->row);*/
	for (;;) {
		graw_glyph8 (gp, width, height, data, ypage, yoffset);
		height -= 8 - yoffset;
		if (height <= 0)
			break;
		ypage++;
		if (ypage >= 8)
			break;
		data += (8 - yoffset) * words_per_row;
		yoffset = 0;
	}
}

/* выключение дисплея, перевод его в спящий режим */
void gpanel_off(gpanel_t *lcd) {
	lcd_write(COMMAND_DISPLAY_POINTS_ON, 0);
	lcd_write(COMMAND_DISPLAY_OFF, 0);
}

/*

void gpanel_line (gpanel_t *lcd, int x0, int y0, int x1, int y1, int color) {
	signed char dx, dy, sx, sy;
	unsigned char x, y, mdx, mdy, l;

	dx = x1 - x0;
	dy = y1 - y0;

	if (dx >= 0) {
		mdx = dx;
		sx = 1;
	} else {
		mdx = x0 - x1;
		sx = -1;
	}
	if (dy >= 0) {
		mdy = dy;
		sy = 1;
	} else {
		mdy = y0 - y1;
		sy = -1;
	}

	x = x0;
	y = y0;

	if (mdx >= mdy) {
		l = mdx;
		while (l > 0) {
			if (dy > 0) {
				y = y0 + mdy * (x - x0) / mdx;
			} else {
				y = y0 - mdy * (x - x0) / mdx;
			}
			gpanel_pixel(lcd, x, y, color);
			x = x + sx;
			l--;
		}
	} else {
		l = mdy;
		while (l > 0) {
			if (dy > 0) {
				x = x0 + ((mdx * (y - y0)) / mdy);
			} else {
				x = x0 + ((mdx * (y0 - y)) / mdy);
			}
			gpanel_pixel(lcd, x, y, color);
			y = y + sy;
			l--;
		}
	}
	gpanel_pixel(lcd, x1, y1, color);
}

void gpanel_rect (gpanel_t *lcd, int x0, int y0, int x1, int y1, int color) {
	gpanel_line (lcd, x0, y0, x1, y0, color);
	gpanel_line (lcd, x1, y0, x1, y1, color);
	gpanel_line (lcd, x0, y0, x0, y1, color);
	gpanel_line (lcd, x0, y1, x1, y1, color);
}
*/

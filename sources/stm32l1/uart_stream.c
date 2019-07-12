/*
 * Драйвер UART STM32L1 для использования со встроенной функцией printf.
 *
 */
#include <runtime/lib.h>
#include <kernel/uos.h>
#include <kernel/internal.h>
#include <uart/uart.h>
//#include <uart/uart-interface.h>

static int UART_IRQ(small_uint_t port) {
	small_uint_t irq = 0;
	if ((port) == STM_USART1_BASE)
		irq = IRQ_USART1;
	else if ((port) == STM_USART2_BASE)
		irq = IRQ_USART2;
	else if ((port) == STM_USART3_BASE)
		irq = IRQ_USART3;
#if defined(STM32L_CAT4) || defined(STM32L_CAT5) || defined(STM32L_CAT6)
	else if ((port) == STM_UART4_BASE)
		irq = IRQ_UART4;
	else if ((port) == STM_UART5_BASE)
		irq = IRQ_UART5;
#endif
	return irq;
}

/*
 * Ожидание окончания передачи данных..
 */
static void uart_fflush(uart_t *u) {
	USART_t *reg = (USART_t*) u->port;

	mutex_lock(&u->receiver);

	/* Проверяем, что передатчик включен. */
	if (reg->CR1 & USART_UE)
		while (u->out_first != u->out_last)
			mutex_wait(&u->receiver);

	mutex_unlock(&u->receiver);
}

/*
 * Отправка одного байта.
 */
void uart_putchar(uart_t *u, short c) {
	USART_t *reg = (USART_t*) u->port;
	unsigned char *newlast;

	mutex_lock(&u->receiver);

	/* Check that transmitter is enabled. */
	if (reg->CR1 & USART_UE) {
		newlast = u->out_last + 1;
		if (newlast >= u->out_buf + UART_OUTBUFSZ)
			newlast = u->out_buf;
		while (u->out_first == newlast)
			mutex_wait(&u->receiver);

		*u->out_last = c;
		u->out_last = newlast;
		if (reg->SR & USART_TXE) {
			/* В буфере FIFO передатчика есть место. */
			reg->DR = *u->out_first++;
			if (u->out_first >= u->out_buf + UART_OUTBUFSZ)
				u->out_first = u->out_buf;
		}

	}
	mutex_unlock(&u->receiver);
}

/*
 * Приём одного байта с ожиданием.
 */
unsigned short uart_getchar(uart_t *u) {
	unsigned char c;

	mutex_lock(&u->receiver);

	/* Wait until receive data available. */
	while (u->in_first == u->in_last)
		mutex_wait(&u->receiver);

	c = *u->in_first++;
	if (u->in_first >= u->in_buf + UART_INBUFSZ)
		u->in_first = u->in_buf;

	mutex_unlock(&u->receiver);
	return c;
}

/*
 * Просмотр первого принятого байта, без ожидания.
 * Байт остаётся в буфере и должен быть извлечён вызовом uart_getchar().
 * Если буфер пустой, возвращается -1.
 */
int uart_peekchar(uart_t *u) {
	int c;

	mutex_lock(&u->receiver);
	c = (u->in_first == u->in_last) ? -1 : *u->in_first;
	mutex_unlock(&u->receiver);
	return c;
}

/*
 * Быстрый обработчик прерывания.
 * Возвращает 0, если требуется послать сигнал мутексу.
 * Возвращает 1, если прерывание полностью обработано и
 * нет необходимости беспокоить мутекс.
 */
static bool_t uart_interrupt(void *arg) {
	uart_t *u = arg;
	USART_t *reg = (USART_t*) u->port;
	//bool_t passive = 1;

	/* Приём. */
	while ((reg->SR & USART_RXNE)) {
		/* В буфере FIFO приемника есть данные. */
		unsigned c = reg->DR;

		unsigned char *newlast = u->in_last + 1;
		if (newlast >= u->in_buf + UART_INBUFSZ)
			newlast = u->in_buf;

		/* Если нет места в буфере - теряем данные. */
		if (u->in_first != newlast) {
			*u->in_last = c;
			u->in_last = newlast;
		}
		//passive = 0;
	}

	/* Передача. */
	if (reg->SR & USART_TC) {
		//reg->SR &= ~USART_TC;
		if (u->out_first != u->out_last) {
			/* Шлём очередной байт. */
			reg->DR = *u->out_first;
			if (++u->out_first >= u->out_buf + UART_OUTBUFSZ)
				u->out_first = u->out_buf;
		} else {
			/* Нет данных для передачи - сброс прерывания. */
			reg->SR &= ~USART_TC;
			//passive = 0;
		}
	}
	arch_intr_allow(UART_IRQ(u->port));
	return 0;
}

/*
 * Возвращает адрес мутекса, получающего сигналы при приёме новых данных.
 */
mutex_t *
uart_receive_lock(uart_t *u) {
	return &u->receiver;
}

/*
 * Stream-интерфейс для UART.
 */
static stream_interface_t uart_interface = {
    .putc = (void (*)(stream_t*, short)) uart_putchar,
    .getc = (unsigned short (*)(stream_t*)) uart_getchar,
    .peekc = (int (*)(stream_t*)) uart_peekchar,
		.flush = (void (*)(stream_t*)) uart_fflush,
		.receiver = (mutex_t *(*)( stream_t*)) uart_receive_lock, };

//static void stm32f4_set_pin_uart(GPIO_t *port, unsigned pin, unsigned funk) {
//	RCC->AHB1ENR |= (1 << PORT(pin));	// Включение тактовой порта
//	// Настройка порта
//	port->MODER = (port->MODER & ~GPIO_MASK(PIN(pin))) | GPIO_ALT(PIN(pin)); //GPIO_ALT(PIN(pin));
//	if (PIN(pin) < 8)
//		port->AFRL = (port->AFRL & ~GPIO_AF_MASK(PIN(pin))) | funk;
//	else
//		port->AFRH = (port->AFRH & ~GPIO_AF_MASK(PIN(pin))) | funk;
//	port->OSPEEDR |= GPIO_100MHz(PIN(pin));
//	port->OTYPER &= ~GPIO_OD(PIN(pin));
//	port->PUPDR &= ~GPIO_PULL_MASK(PIN(pin));
//}

//#define PLL_GET_M			(RCC->PLLCFGR & 0x1F)
//#define PLL_GET_N			((RCC->PLLCFGR >> 6) & 0x1FF)
//#define PLL_GET_P			((RCC->PLLCFGR >> 16)& 0x3)
//#define PLL_GET_Q			((RCC->PLLCFGR >> 24) & 0xF)
//
//unsigned getHclk(void){
//	unsigned hclk = 0;
//	// PLLCFGR = 0x07405408
//	// M = 8		mask 0x1F
//	// N = 336		mask 0x1FF
//	// P = 0		mask 0x3
//	// Q = 3		mask 0x7
//	if(RCC->PLLCFGR & RCC_PLLSRC_HSE) // HSE
//		hclk = (KHZ_CLKIN * 1000 / PLL_GET_M) * PLL_GET_N / (PLL_GET_P * 2 + 2);
//	else // HSI
//		hclk = (16000000 / PLL_GET_M) * PLL_GET_N / (PLL_GET_P * 2 + 2);
//	return hclk;
//}
//
//unsigned getAPB1Clk(void){
//	unsigned hclk = getHclk();
//	unsigned clk = hclk >> (((RCC->CFGR >> 10) & 0x7) - 3);
//	return clk;
//}
//
//unsigned getAPB2Clk(void){
//	unsigned hclk = getHclk();
//	unsigned clk = hclk >> (((RCC->CFGR >> 13) & 0x7) - 3);
//	return clk;
//}

#define UART_BITS(n)            (n)         // число бит в слове
#define UART_PE_E               (1 << 4)    // контроль чётности: нечётность
#define UART_PE_O               (1 << 5)    // контроль чётности: чётность
#define UART_STOP_05            (1 << 8)    // 0,5 стоп-бит
#define UART_STOP_1             0           // по умолчанию 1 стоп-бит
#define UART_STOP_15            (1 << 9)    // 1,5 стоп-бит
#define UART_STOP_2             (1 << 10)   // 2 стоп-бит

#define UART_BITS_MASK          0xF         // маска для поля с числом бит
#define UART_PARITY_MASK        (0xF << 4)  // маска для полей контроля чётности
#define UART_STOP_MASK          (0xF << 8)  // маска для кол-ва стоп-бит

const unsigned USART_CR1_COMMON = USART_UE | USART_TE | USART_RE | USART_RXNEIE | USART_TCIE;

static int stm32l1_uart_set_param(uart_t *uart, unsigned params, unsigned bytes_per_sec)
{
    USART_t *uregs = (USART_t*) uart->port;
    uint32_t cr1 = 0;

    mutex_lock(&uart->receiver);
    uregs->CR1 = 0;

    if ((params & UART_BITS_MASK) != 8) {
        // Only 8-bit mode supported yet!
        mutex_unlock(&uart->receiver);
        return -1;
    }

    if (params & UART_PE_E)
        cr1 |= USART_PCE;
    else if (params & UART_PE_O)
        cr1 |= USART_PCE | USART_PS;

    uregs->CR2 &= ~USART_STOP_MASK;
    switch (params & UART_STOP_MASK) {
    case UART_STOP_1:
        uregs->CR2 |= USART_STOP_1;
        break;
    case UART_STOP_05:
        uregs->CR2 |= USART_STOP_05;
        break;
    case UART_STOP_2:
        uregs->CR2 |= USART_STOP_2;
        break;
    case UART_STOP_15:
        uregs->CR2 |= USART_STOP_15;
        break;
    }

    uregs->BRR = KHZ * 1000 / bytes_per_sec;
    uregs->CR1 = cr1 | USART_CR1_COMMON;

    //  reg->CR1 |= USART_TE | USART_RE;  // | USART_SBK;
    //  reg->CR1 |= USART_RXNEIE | USART_TCIE;  //0x0525 | 0x0626;
    //
    //  reg->CR1 |= USART_UE;

    mutex_unlock(&uart->receiver);

    return 0;
}

/*
 * Инициализация UART:
 * port	- номер порта, 0 или 1
 * prio - приоритет задачи обработки прерываний
 * khz  - вместо khz параметры
 * baud - требуемая скорость передачи данных, бит/сек
 */
void uart_init(uart_t *u, small_uint_t port, int prio, unsigned int khz,
		unsigned long baud) {
	u->interface = &uart_interface;
	u->in_first = u->in_last = u->in_buf;
	u->out_first = u->out_last = u->out_buf;
	//u->khz = khz;
	u->onlcr = 1;
	if (port == 1)
		u->port = STM_USART1_BASE;
	else if (port == 2)
		u->port = STM_USART2_BASE;
	else if (port == 3)
		u->port = STM_USART3_BASE;
#if defined(STM32L_CAT4) || defined(STM32L_CAT5) || defined(STM32L_CAT6)
	else if (port == 4)
		u->port = STM_UART4_BASE;
	else if (port == 5)
		u->port = STM_UART5_BASE;
#endif

	/*
	 * Enable UART.
	 */
	USART_t *reg = (USART_t*) u->port;
	//mutex_lock_irq(&u->receiver, UART_IRQ(u->port), uart_interrupt, u);
	mutex_lock(&u->receiver);
	mutex_attach_irq(&u->receiver, UART_IRQ(u->port), uart_interrupt, u);
	/*
	 * Установка скорости передачи данных.
	 */
	//unsigned fclk = 0;
	reg->CR1 = 0;						// останов приемопередатчика
	reg->CR2 = 0;
	reg->CR3 = 0;
	reg->BRR = 0;
	reg->SR = 0;
	if (port == 1) {
		RCC->APB2ENR &= ~RCC_USART1EN;
		RCC->APB2ENR |= RCC_USART1EN;
	} else if (port == 2) {
		RCC->APB1ENR |= RCC_USART2EN;
	} else if (port == 3) {
		RCC->APB1ENR |= RCC_USART3EN;
	}
#if defined(STM32L_CAT4) || defined(STM32L_CAT5) || defined(STM32L_CAT6)
	else if (port == 4) {
		//fclk = getAPB1Clk();;
		RCC->APB1ENR |= RCC_UART4EN;
		stm32f4_set_pin_uart(UART4_RX_GPIO, UART4_RX, GPIO_AF_USART4(PIN(UART4_RX)));
		stm32f4_set_pin_uart(UART4_TX_GPIO, UART4_TX, GPIO_AF_USART4(PIN(UART4_TX)));
	} else if (port == 5) {
		//fclk = getAPB1Clk();;
		RCC->APB1ENR |= RCC_UART5EN;
		stm32f4_set_pin_uart(UART5_RX_GPIO, UART5_RX, GPIO_AF_USART5(PIN(UART5_RX)));
		stm32f4_set_pin_uart(UART5_TX_GPIO, UART5_TX, GPIO_AF_USART5(PIN(UART5_TX)));
	}
#endif
//	reg->CR2 |= USART_STOP_1;
//	//reg->BRR = tmpreg;//USART_DIV_MANTISSA(mant/100) | USART_DIV_FRACTION(frac);
//	reg->CR1 |= USART_TE | USART_RE;	// | USART_SBK;
//	reg->CR1 |= USART_RXNEIE | USART_TCIE;	//0x0525 | 0x0626;
//
//	reg->CR1 |= USART_UE;
	mutex_unlock(&u->receiver);
//	setBaudRate(u, baud);
	stm32l1_uart_set_param(u, khz, baud);
}



//void setBaudRate(uart_t *u, unsigned long baud) {
//	unsigned fclk = 0;
//	USART_t *reg = (USART_t*) u->port;
//	mutex_lock(&u->receiver);
//	reg->CR1 &= ~USART_UE;
//	if (u->port == STM_USART1_BASE){
//		// APB2 max 84MHz
//		fclk = getAPB2Clk();
//	} else if (u->port == STM_USART2_BASE) {
//		// APB1 max 42MHz
//		fclk = getAPB1Clk();
//	} else if (u->port == STM_USART3_BASE) {
//		// APB1 max 42MHz
//		fclk = getAPB1Clk();
//	} else if (u->port == STM_UART4_BASE) {
//		// APB1 max 42MHz
//		fclk = getAPB1Clk();
//	} else if (u->port == STM_UART5_BASE) {
//		// APB1 max 42MHz
//		fclk = getAPB1Clk();
//	} else if (u->port == STM_USART6_BASE) {
//		// APB2 max 84MHz
//		fclk = getAPB2Clk();
//	}
//
//	unsigned mant = (unsigned) ((25 * fclk) / (4 * (baud)));
//	unsigned tmpreg = (mant / 100) << 4;
//	unsigned frac = (unsigned) (mant - (100 * tmpreg >> 4));
//	tmpreg |= ((((frac * 16) + 50) / 100)) & ((uint8_t) 0x0F);
//	reg->BRR = tmpreg;
//	reg->CR1 |= USART_UE;
//	mutex_unlock(&u->receiver);
//}


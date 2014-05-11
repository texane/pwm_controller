#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#define CONFIG_DEBUG 1
#define CONFIG_LCD 0
#define CONFIG_UART 1


#if CONFIG_LCD

/* note: lcd model MC21605A6W */
/* note: DB0:3 and RW must be grounded */

#define LCD_POS_DB 0x02
#define LCD_PORT_DB PORTD
#define LCD_DIR_DB DDRD
#define LCD_MASK_DB (0x0f << LCD_POS_DB)

#define LCD_POS_EN 0x06
#define LCD_PORT_EN PORTD
#define LCD_DIR_EN DDRD
#define LCD_MASK_EN (0x01 << LCD_POS_EN)

#define LCD_POS_RS 0x07
#define LCD_PORT_RS PORTD
#define LCD_DIR_RS DDRD
#define LCD_MASK_RS (0x01 << LCD_POS_RS)

static inline void wait_50_ns(void)
{
  __asm__ __volatile__ ("nop\n\t");
}

static inline void wait_500_ns(void)
{
  /* 8 cycles at 16mhz */
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
  __asm__ __volatile__ ("nop\n\t");
}

static inline void wait_50_us(void)
{
  /* 800 cycles at 16mhz */
  uint8_t x;
  for (x = 0; x < 100; ++x) wait_500_ns();
}

static inline void wait_2_ms(void)
{
  wait_50_us();
  wait_50_us();
  wait_50_us();
  wait_50_us();
}

static inline void wait_50_ms(void)
{
  /* FIXME: was _delay_ms(50), but not working */
  uint8_t x;
  for (x = 0; x < 25; ++x) wait_2_ms();
}

static inline void lcd_pulse_en(void)
{
  /* assume EN low */
  LCD_PORT_EN |= LCD_MASK_EN;
  wait_50_us();
  LCD_PORT_EN &= ~LCD_MASK_EN;
  wait_2_ms();
}

static inline void lcd_write_db4(uint8_t x)
{
  /* configured in 4 bits mode */

  LCD_PORT_DB &= ~LCD_MASK_DB;
  LCD_PORT_DB |= (x >> 4) << LCD_POS_DB;
  lcd_pulse_en();

  LCD_PORT_DB &= ~LCD_MASK_DB;
  LCD_PORT_DB |= (x & 0xf) << LCD_POS_DB;
  lcd_pulse_en();
}

static inline void lcd_write_db8(uint8_t x)
{
  /* configured in 8 bits mode */

  /* only hi nibble transmitted, (0:3) grounded */
  LCD_PORT_DB &= ~LCD_MASK_DB;
  LCD_PORT_DB |= (x >> 4) << LCD_POS_DB;
  lcd_pulse_en();
}

static inline void lcd_setup(void)
{
  LCD_DIR_DB |= LCD_MASK_DB;
  LCD_DIR_RS |= LCD_MASK_RS;
  LCD_DIR_EN |= LCD_MASK_EN;

  LCD_PORT_DB &= ~LCD_MASK_DB;
  LCD_PORT_RS &= ~LCD_MASK_RS;
  LCD_PORT_EN &= ~LCD_MASK_EN;

  /* small delay for the lcd to boot */
  wait_50_ms();

  /* datasheet init sequence */

#define LCD_MODE_BLINK (1 << 0)
#define LCD_MODE_CURSOR (1 << 1)
#define LCD_MODE_DISPLAY (1 << 2)

  lcd_write_db8(0x30);
  wait_2_ms();
  wait_2_ms();
  wait_500_ns();

  lcd_write_db8(0x30);
  wait_2_ms();

  lcd_write_db4(0x32);
  wait_2_ms();

  lcd_write_db4(0x28);
  wait_2_ms();

  lcd_write_db4((1 << 3) | LCD_MODE_DISPLAY);
  wait_2_ms();

  lcd_write_db4(0x01);
  wait_2_ms();

  lcd_write_db4(0x0f);
  wait_2_ms();
}

static inline void lcd_clear(void)
{
  /* clear lcd */
  lcd_write_db4(0x01);
  wait_2_ms();
}

static inline void lcd_home(void)
{
  /* set cursor to home */
  lcd_write_db4(0x02);
  wait_2_ms();
}

static inline void lcd_set_ddram(uint8_t addr)
{
  lcd_write_db4((1 << 7) | addr);
  wait_2_ms();
}

static inline void lcd_goto_xy(uint8_t x, uint8_t y)
{
  /* assume 0 <= x < 8 */
  /* assume 0 <= y < 2 */

  /* from datasheet: */
  /* first line is 0x00 to 0x27 */
  /* second line is 0x40 to 0x67 */
  static const uint8_t row[] = { 0x00, 0x40 };
  lcd_set_ddram(row[y] | x);
}

static void lcd_write(const uint8_t* s, unsigned int n)
{
  wait_50_ns();

  LCD_PORT_RS |= LCD_MASK_RS;
  for (; n; --n, ++s)
  {
    lcd_write_db4(*s);
    wait_2_ms();
  }
  LCD_PORT_RS &= ~LCD_MASK_RS;
}

#endif /* CONFIG_LCD */


#if CONFIG_UART /* uart */

static inline void set_baud_rate(long baud)
{
  uint16_t UBRR0_value = ((F_CPU / 16 + baud / 2) / baud - 1);
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;
}

static void uart_setup(void)
{
  /* #define CONFIG_FOSC (F_CPU * 2) */
  /* const uint16_t x = CONFIG_FOSC / (16 * BAUDS) - 1; */
#if 0 /* (bauds == 9600) */
  const uint16_t x = 206;
#elif 0 /* (bauds == 115200) */
  const uint16_t x = 16;
#elif 0 /* (bauds == 500000) */
  const uint16_t x = 3;
#elif 0 /* (bauds == 1000000) */
  const uint16_t x = 1;
#endif

  set_baud_rate(9600);

  /* baud doubler off  - Only needed on Uno XXX */
  UCSR0A &= ~(1 << U2X0);

  UCSR0B = 1 << TXEN0;

  /* default to 8n1 framing */
  UCSR0C = (3 << 1);
}

static void uart_write(const uint8_t* s, uint8_t n)
{
  for (; n; --n, ++s)
  {
    /* wait for transmit buffer to be empty */
    while (!(UCSR0A & (1 << UDRE0))) ;
    UDR0 = *s;
  }

  /* wait for last byte to be sent */
  while ((UCSR0A & (1 << 6)) == 0) ;
}

static void uart_write_cstring(const char* s)
{
  uint8_t n;
  for (n = 0; s[n]; ++n) ;
  uart_write((const uint8_t*)s, n);
}

#endif /* CONFIG_UART */


static inline uint8_t nibble(uint32_t x, uint8_t i)
{
  return (x >> (i * 4)) & 0xf;
}

static inline uint8_t hex(uint8_t x)
{
  return (x >= 0xa) ? 'a' + x - 0xa : '0' + x;
}

static uint8_t* uint8_to_string(uint8_t x)
{
  static uint8_t buf[2];

  buf[1] = hex(nibble(x, 0));
  buf[0] = hex(nibble(x, 1));

  return buf;
}

__attribute__((unused))
static uint8_t* uint16_to_string(uint16_t x)
{
  static uint8_t buf[4];

  buf[3] = hex(nibble(x, 0));
  buf[2] = hex(nibble(x, 1));
  buf[1] = hex(nibble(x, 2));
  buf[0] = hex(nibble(x, 3));

  return buf;
}

static unsigned int ulong_to_string(unsigned long n, char* s)
{
  unsigned char buf[8];
  unsigned long i = 0;
  unsigned int len = 0;
  if (n == 0) { *s = '0'; return 1; }
  while (n > 0) { buf[i++] = n % 10; n /= 10; }
  for (; i > 0; i--, ++len) s[len] = '0' + buf[i - 1];
  return len;
}

__attribute__((unused))
static unsigned int double_to_string(double x, const uint8_t** s)
{
  static char buf[32];
  uint8_t digits = 3;
  double rounding = 0.5;
  unsigned int len = 0;
  uint8_t i;
  unsigned long int_part;
  double remainder;

  if (x < 0.0) { buf[len++] = '-'; x = -x; }

  for (i = 0; i < digits; ++i) rounding /= 10.0;
  x += rounding;

  int_part = (unsigned long)x;
  remainder = x - (double)int_part;

  len += ulong_to_string(int_part, buf + len);

  if (digits > 0) buf[len++] = '.';

  while ((digits--) > 0)
  {
    unsigned long xx;
    remainder *= 10.0;
    xx = (unsigned long)remainder;
    len += ulong_to_string(xx, buf + len);
    remainder -= xx; 
  }

  *s = (const uint8_t*)buf;

  return len;
}


/* application */

/* timer configuration */

#define TIMER_FREQ 100
#define TIMER_MS_TO_TICKS(__x) \
  (((uint32_t)(__x) * (uint32_t)TIMER_FREQ) / (uint32_t)1000)
#define TIMER_TICKS_TO_MS(__x) \
  (((uint32_t)(__x) * (uint32_t)1000) / (uint32_t)TIMER_FREQ)

/* buttons ATMEGA328P pin configuration */

#define BUT_COMMON_DDR DDRC
#define BUT_COMMON_PIN PINC
#define BUT_COMMON_PORT PORTC

#define BUT_SAVE_POS 2
#define BUT_MODE_POS 3
#define BUT_PLUS_POS 4
#define BUT_MINUS_POS 5

#define BUT_SAVE_MASK (1 << BUT_SAVE_POS)
#define BUT_MODE_MASK (1 << BUT_MODE_POS)
#define BUT_PLUS_MASK (1 << BUT_PLUS_POS)
#define BUT_MINUS_MASK (1 << BUT_MINUS_POS)

#define BUT_COMMON_MASK \
  (BUT_SAVE_MASK | BUT_MODE_MASK | BUT_PLUS_MASK | BUT_MINUS_MASK)

/* button counters */

static volatile uint8_t but_states;

static void but_setup(void)
{
  /* enable pullups and set as input */
  BUT_COMMON_PORT |= BUT_COMMON_MASK;
  BUT_COMMON_DDR &= ~BUT_COMMON_MASK;

  /* reset button states */
  but_states = 0;
}

static inline void but_update_one
(uint8_t pre, uint8_t cur, uint8_t* counter, uint8_t mask)
{
  const uint8_t is_pressed = (cur & mask) == 0;

  if (is_pressed && ((pre & mask) == (cur & mask)))
  {
    if ((++*counter) == TIMER_MS_TO_TICKS(125))
    {
      but_states |= mask;
      *counter = 0;
    }
  }
  else
  {
    but_states &= ~mask;
    *counter = 0;
  }
}

static void but_update_states(void)
{
  static uint8_t save_counter = 0;
  static uint8_t mode_counter = 0;
  static uint8_t plus_counter = 0;
  static uint8_t minus_counter = 0;

  static uint8_t pre = 0;

  const uint8_t cur = BUT_COMMON_PIN & BUT_COMMON_MASK;

  but_update_one(pre, cur, &save_counter, BUT_SAVE_MASK);
  but_update_one(pre, cur, &mode_counter, BUT_MODE_MASK);
  but_update_one(pre, cur, &plus_counter, BUT_PLUS_MASK);
  but_update_one(pre, cur, &minus_counter, BUT_MINUS_MASK);

  /* affect current value to previous */
  pre = cur;
}

static uint8_t but_read_states(void)
{
  const uint8_t x = but_states;
  but_states &= ~(x);
  return x;
}

static uint8_t but_is_pressed(uint8_t x, uint8_t m)
{
  return x & m;
}


/* timer1 interrupt handler */

static volatile uint8_t timer_ticks = 0;

ISR(TIMER0_COMPA_vect)
{
  /* update button state */
  but_update_states();

  /* prevent overflow */
  if (timer_ticks == 0xff) return ;
  ++timer_ticks;
}

static void timer_enable(void)
{
  /* 8 bits timer0 is used */
  /* interrupt at TIMER_FREQ hz */
  /* fcpu / (1024 * 156) = 100 hz */

  /* stop timer */
  TCCR0B = 0;

  /* CTC mode, overflow when OCR1A reached */
  TCCR0A = 1 << 2;
  OCR0A = 156;
  TCNT0 = 0;

  /* interrupt on OCIE0A match */
  TIMSK0 = 1 << 1;

  /* reset timer tick counter */
  timer_ticks = 0;

  /* start timer */
  /* prescaler set to 1024 */
  TCCR0B = (1 << 3) | (5 << 0);
}

__attribute__((unused))
static void timer_disable(void)
{
  TCCR0B = 0;
}


/* absolute difference */

__attribute__((unused))
static uint16_t abs_diff(uint16_t a, uint16_t b)
{
  if (a > b) return a - b;
  return b - a;
}


/* printing routines */

static void uint32_to_string(uint32_t x, uint32_t r, uint8_t* buf)
{
  /* buf is right aligned */

  uint8_t zero = (x == 0) ? '0' : ' ';

  uint8_t i;

  for (i = 0; r; ++i, r /= 10)
  {
    const uint32_t q = x / r;

    /* assume zero */
    buf[i] = zero;

    if (q)
    {
      buf[i] = '0' + q;
      x -= q * r;
      zero = '0';
    }
  }
}

__attribute__((unused))
static void ticks_to_ms_string(uint8_t ticks, uint8_t* buf)
{
  /* assume ms <= 9999 */
  const uint32_t x = TIMER_TICKS_TO_MS(ticks);
  uint32_to_string(x, 1000, buf);
}

static void print_params(uint32_t duty, uint32_t freq, uint32_t dead)
{
  uint8_t buf[8];

  /* duty in percent */
  uint32_to_string(duty, 100, buf);
  uart_write(buf, 3);
  uart_write_cstring(" ");

  /* freq in hz */
  uint32_to_string(freq, 100000, buf);
  uart_write(buf, 6);
  uart_write_cstring(" ");

  /* dead in us */
  uint32_to_string(dead, 100000, buf);
  uart_write(buf, 6);
  uart_write_cstring("\r\n");
}


/* pwm control */

static void pwm_setup(void)
{
#define PWM_CHANA_MASK (1 << 1)
#define PWM_CHANB_MASK (1 << 2)
#define PWM_DDR DDRB

  /* outputs: OC1A / PORTB1, OC1B / PORTB2 */

  /* the 16 bits tcnt1 counter is compared */
  /* to the top (icr1) and ocr1a and ocr1b values. */
  /* this allow the frequecy and duty to be defined. */

  /* stop the counter */
  TCCR1B = 0;

  /* set OC1A and OC1B as outputs */
  /* enable pins output drivers */
  /* OC1A as non inverted and OC1B as inverted */
  PWM_DDR |= PWM_CHANA_MASK | PWM_CHANB_MASK;
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
  TCCR1A |= (0 << COM1A0) | (1 << COM1B0);
    
  /* set pwm mode */
  /* phase and frequency correct mode */
  /* must be set before ICR1 written */
  TCCR1A |= (0 << WGM10) | (0 << WGM11);
  TCCR1B |= (0 << WGM12) | (1 << WGM13);
}

static void pwm_set_params(uint32_t duty, uint32_t freq, uint32_t dead)
{
  /* duty in percent */
  /* freq in hz */
  /* dead in us */

  uint32_t x;

  /* stop during setup */
  TCCR1B &= ~(1 << CS10);

  /* frequency. icr1 acts as top value. */
  ICR1 = F_CPU / (2 * freq);

  /* ocr1a, channel a duty */
  /* ocr1b, channel b duty */
  x = (duty * F_CPU) / (2 * freq * 100UL);
  OCR1A = x;
  OCR1B = x + dead * F_CPU / 1000000UL;

  /* start with no prescaler */
  TCCR1B |= (1 << CS10);
}

/* main */

int main(void)
{
#define MODE_DUTY 0
#define MODE_FREQ 1
#define MODE_DEAD 2
#define MODE_COUNT 3
  uint8_t mode = MODE_DUTY;
  uint8_t but;
  uint8_t has_changed = 1;
  uint32_t duty = 50;
  uint32_t freq = 500;
  uint32_t dead = 0;
  uint32_t* value = &duty;
  uint32_t min = 0;
  uint32_t max = 100;
  uint32_t step = 5;

#if CONFIG_LCD
  /* lcd */
  lcd_setup();
  lcd_clear();
  lcd_home();
#endif

#if CONFIG_UART
  uart_setup();
#endif

  but_setup();
  timer_enable();
  pwm_setup();

  sei();

  /* application logic */

  goto on_changed;

  while (1)
  {
    but = but_read_states();

    if (but_is_pressed(but, BUT_MODE_MASK))
    {
      mode = (mode + 1) % MODE_COUNT;

      switch (mode)
      {
      case MODE_DUTY:
	min = 0;
	max = 100;
	step = 5;
	value = &duty;
	break ;

      case MODE_FREQ:
	min = 0;
	max = 400000;
	step = 100;
	value = &freq;
	break ;

      case MODE_DEAD:
      default:
	min = 0;
	max = 5000000;
	step = 10;
	value = &dead;
	break ;
      }

      uart_write_cstring("mode: ");
      if (mode == MODE_DUTY) uart_write_cstring("duty");
      else if (mode == MODE_FREQ) uart_write_cstring("frequency");
      else uart_write_cstring("deadtime");
      uart_write_cstring("\r\n");
    }

    if (but_is_pressed(but, BUT_MINUS_MASK))
    {
      if (*value >= (min + step))
      {
	*value -= step;
	has_changed = 1;
      }
    }

    if (but_is_pressed(but, BUT_PLUS_MASK))
    {
      if (*value <= (max - step))
      {
	*value += step;
	has_changed = 1;
      }
    }

  on_changed:
    if (has_changed)
    {
      /* convert to register values */
      pwm_set_params(duty, freq, dead);
      print_params(duty, freq, dead);
    }

    has_changed = 0;
  }

  return 0;
}

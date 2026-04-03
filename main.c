#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>


#define LSM_ADDR        0x6B
#define LSM_CTRL2_G     0x11
#define LSM_CTRL3_C     0x12
#define LSM_OUTX_L_G    0x22


#define LSM_GYRO_CFG    0x4C
#define DPS_PER_LSB     0.070f

#define TW_MT_SLA_ACK   0x18
#define TW_MT_DATA_ACK  0x28
#define TW_MR_SLA_ACK   0x40
#define TWI_STATUS      (TWSR0 & 0xF8)

#define SERVO_MIN   2000U
#define SERVO_MID   3000U
#define SERVO_MAX   4000U

#define TICKS_PER_DEG  11.0f

#define UBRR_VAL  ((F_CPU / (16UL * 9600UL)) - 1)

static void uart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)(UBRR_VAL);
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
static void uart_putchar(char c)
{
    while (!(UCSR0A & (1 << UDRE0))); UDR0 = c;
}
static void uart_print(const char *s) { while (*s) uart_putchar(*s++); }
static void uart_int(int32_t v)
{
    char buf[12]; int8_t i = 0;
    if (v == 0) { uart_putchar('0'); return; }
    uint8_t neg = (v < 0); if (neg) v = -v;
    while (v) { buf[i++] = '0' + v % 10; v /= 10; }
    if (neg) buf[i++] = '-';
    while (i--) uart_putchar(buf[i]);
}

static void i2c_init(void)
{
    TWBR0 = (uint8_t)((F_CPU / 400000UL - 16) / 2);
    TWSR0 = 0x00;
}
static uint8_t i2c_start(uint8_t addr_rw)
{
    TWCR0 = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while (!(TWCR0 & (1<<TWINT)));
    TWDR0 = addr_rw;
    TWCR0 = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR0 & (1<<TWINT)));
    return (TWI_STATUS == TW_MT_SLA_ACK) || (TWI_STATUS == TW_MR_SLA_ACK);
}
static void i2c_stop(void) { TWCR0 = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); }
static void i2c_write(uint8_t d)
{
    TWDR0 = d; TWCR0 = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR0 & (1<<TWINT)));
}
static uint8_t i2c_read(uint8_t ack)
{
    TWCR0 = (1<<TWINT)|(1<<TWEN)|(ack ? (1<<TWEA) : 0);
    while (!(TWCR0 & (1<<TWINT)));
    return TWDR0;
}

static void lsm_write(uint8_t reg, uint8_t val)
{
    i2c_start((LSM_ADDR << 1)); i2c_write(reg); i2c_write(val); i2c_stop();
}
static uint8_t lsm_read_reg(uint8_t reg)
{
    i2c_start((LSM_ADDR << 1)); i2c_write(reg);
    i2c_start((LSM_ADDR << 1) | 1);
    uint8_t v = i2c_read(0); i2c_stop(); return v;
}
static int16_t lsm_read_gyro_x(void)
{
    i2c_start((LSM_ADDR << 1)); i2c_write(LSM_OUTX_L_G);
    i2c_start((LSM_ADDR << 1) | 1);
    uint8_t lo = i2c_read(1);
    uint8_t hi = i2c_read(0);
    i2c_stop();
    return (int16_t)((uint16_t)(hi << 8) | lo);
}

static uint8_t lsm_init(void)
{
    _delay_ms(20);
    uint8_t ack = i2c_start((LSM_ADDR << 1)); i2c_stop();
    uart_print("ACK: "); uart_print(ack ? "YES\r\n" : "NO\r\n");

    uint8_t who = lsm_read_reg(LSM_WHO_AM_I);
    uart_print("WHO_AM_I: "); uart_int(who);
    uart_print(who == 0x6C ? " OK\r\n" : " FAIL (want 108)\r\n");
    if (who != 0x6C) return 0;

    lsm_write(LSM_CTRL3_C, 0x01); /* software reset */
    _delay_ms(50);
    lsm_write(LSM_CTRL2_G, LSM_GYRO_CFG); /* gyro on, 104 Hz, ±2000 dps */
    _delay_ms(10);
    return 1;
}

static void servo_init(void)
{
    DDRB |= (1 << PB1);
    ICR1   = 39999;
    OCR1A  = SERVO_MID;
    TCCR1A = (1<<COM1A1)|(1<<WGM11);
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
}
static void servo_set(int32_t ticks)
{
    if (ticks < SERVO_MIN) ticks = SERVO_MIN;
    if (ticks > SERVO_MAX) ticks = SERVO_MAX;
    OCR1A = (uint16_t)ticks;
}

/* ?? Main ?????????????????????????????????????????????????????? */
int main(void)
{
    uart_init();
    i2c_init();
    servo_init();

    uart_print("\r\n=== Servo Stabiliser ===\r\n");
    while (!lsm_init()) { uart_print("retrying...\r\n"); _delay_ms(1000); }

    uart_print("raw_gyro,angle_x100,servo\r\n");

    float   angle    = 0.0f;   /* integrated angle in degrees        */
    uint8_t log_tick = 0;

    while (1)
    {
        int16_t raw = lsm_read_gyro_x();

        /*
         * Integrate: angle += rate_dps * dt
         * Small drift is normal; add a gyro bias calibration step
         * (average 100 readings at rest) if drift becomes a problem.
         */
        angle += (float)raw * DPS_PER_LSB * 0.010f;  /* dt = 10 ms */

        /*
         * Counteract the rotation: if the mount tilts +angle degrees,
         * drive the servo -angle degrees to hold the original pointing.
         * Flip the sign if the servo corrects in the wrong direction.
         */
        int32_t ticks = (int32_t)SERVO_MID - (int32_t)(angle * TICKS_PER_DEG);
        servo_set(ticks);

        /* Print every 100 ms */
        if (++log_tick >= 10)
        {
            log_tick = 0;
            uart_int(raw);          uart_putchar(',');
            uart_int((int32_t)(angle * 100.0f)); uart_putchar(',');
            uart_int(ticks);        uart_print("\r\n");
        }

        _delay_ms(10);
    }
}
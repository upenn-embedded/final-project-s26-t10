#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

/* ── IMU (LSM6DSO) ──────────────────────────────────────── */
#define LSM_ADDR        0x6B
#define LSM_CTRL2_G     0x11
#define LSM_CTRL3_C     0x12
#define LSM_OUTX_L_G    0x22
#define LSM_OUTY_L_G    0x24
#define LSM_OUTZ_L_G    0x26


#define LSM_GYRO_CFG    0x4C      /* 104 Hz, ±2000 dps */
#define DPS_PER_LSB     0.070f

/* ── I2C status codes ────────────────────────────────────── */
#define TW_MT_SLA_ACK   0x18
#define TW_MR_SLA_ACK   0x40
#define TWI_STATUS      (TWSR0 & 0xF8)

/* ── DS3218 270° servo on Timer1 (OC1A / PB1) ───────────── *
 * PWM range: 500–2500 µs  →  0–270°
 * With prescaler /8 and 16 MHz: 1 tick = 0.5 µs
 *   500 µs = 1000 ticks,  2500 µs = 5000 ticks
 *   centre (135°) = 3000 ticks
 */
#define SERVO_MIN    1000U    /* 500 µs  →   0° */
#define SERVO_MID    3000U    /* 1500 µs → 135° */
#define SERVO_MAX    5000U    /* 2500 µs → 270° */

/* ticks per degree: (5000-1000)/270 ≈ 14.81 */
#define TICKS_PER_DEG  14.81f

/* ── PID gains (tune these) ──────────────────────────────── */
#define KP   1.0f
#define KI   0.2f
#define KD   0.01f
#define I_LIMIT  30.0f   /* anti-windup clamp in degrees */

/* ── UART ────────────────────────────────────────────────── */
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
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
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

/* ── I2C ─────────────────────────────────────────────────── */
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
static void i2c_stop(void)  { TWCR0 = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); }
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

/* ── LSM6DSO helpers ─────────────────────────────────────── */
static void lsm_write(uint8_t reg, uint8_t val)
{
    i2c_start(LSM_ADDR << 1); i2c_write(reg); i2c_write(val); i2c_stop();
}
static int16_t lsm_read_gyro_x(void)
{
    i2c_start(LSM_ADDR << 1); i2c_write(LSM_OUTX_L_G);
    i2c_start((LSM_ADDR << 1) | 1);
    uint8_t lo = i2c_read(1);
    uint8_t hi = i2c_read(0);
    i2c_stop();
    return (int16_t)((uint16_t)(hi << 8) | lo);
}
static int16_t lsm_read_gyro_z(void)
{
    i2c_start(LSM_ADDR << 1); i2c_write(LSM_OUTZ_L_G);
    i2c_start((LSM_ADDR << 1) | 1);
    uint8_t lo = i2c_read(1);
    uint8_t hi = i2c_read(0);
    i2c_stop();
    return (int16_t)((uint16_t)(hi << 8) | lo);
}
static void lsm_init(void)
{
    _delay_ms(20);
    lsm_write(LSM_CTRL3_C, 0x01);        /* software reset */
    _delay_ms(50);
    lsm_write(LSM_CTRL2_G, LSM_GYRO_CFG); /* 104 Hz, ±2000 dps */
    _delay_ms(10);
}

/* ── Gyro bias calibration ───────────────────────────────── */
static float gyro_bias = 0.0f;

static void calibrate_gyro(void)
{
    uart_print("Calibrating gyro (hold still)...\r\n");
    int32_t sum = 0;
    for (uint16_t i = 0; i < 200; i++) {
        sum += lsm_read_gyro_x();
        _delay_ms(5);
    }
    gyro_bias = (float)sum / 200.0f;
    uart_print("Bias: "); uart_int((int32_t)(gyro_bias * 100)); uart_print("/100\r\n");
}

/* ── Timer2 for precise dt measurement ───────────────────── *
 * Timer0 is 8-bit — we use it in normal mode with /1024 prescaler.
 * 16 MHz / 1024 = 15625 Hz → 1 tick = 64 µs
 * At ~10 ms loop: ~156 ticks, fits in 8 bits.
 *
 * NOTE: Timer1 is used by the servo PWM so we use Timer0 here.
 * If Timer0 overflows (>16.4 ms) we detect via TOV0 and cap.
 */
static void dt_timer_init(void)
{
    TCCR0A = 0;                       /* normal mode */
    TCCR0B = (1 << CS02) | (1 << CS00); /* /1024 */
    TCNT0  = 0;
    TIFR0  = (1 << TOV0);            /* clear overflow flag */
}

static float dt_read_and_reset(void)
{
    uint8_t cnt = TCNT0;
    uint8_t ovf = TIFR0 & (1 << TOV0);
    TCNT0  = 0;
    TIFR0  = (1 << TOV0);            /* clear overflow */

    uint16_t ticks = ovf ? (256U + cnt) : cnt;
    /* each tick = 1024 / 16e6 = 64 µs */
    return (float)ticks * (1024.0f / (float)F_CPU);
}

/* ── Servo PWM (Timer1) ──────────────────────────────────── */
static void servo_init(void)
{
    DDRB  |= (1 << PB1);
    ICR1   = 39999;           /* 20 ms period */
    OCR1A  = SERVO_MID;
    TCCR1A = (1<<COM1A1)|(1<<WGM11);
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);  /* /8 */
}
static void servo_set(int32_t ticks)
{
    if (ticks < (int32_t)SERVO_MIN) ticks = SERVO_MIN;
    if (ticks > (int32_t)SERVO_MAX) ticks = SERVO_MAX;
    OCR1A = (uint16_t)ticks;
}

/* ── Main ────────────────────────────────────────────────── */
int main(void)
{
    uart_init();
    i2c_init();
    servo_init();
    dt_timer_init();

    uart_print("\r\n=== Servo Stabiliser (PID) ===\r\n");
    lsm_init();
    calibrate_gyro();

    uart_print("dt_us,raw,angle_x100,servo\r\n");

    float angle     = 0.0f;
    float integral  = 0.0f;
    float prev_err  = 0.0f;
    uint8_t log_div = 0;

    while (1)
    {
        /* ---- Measure precise dt ---- */
        float dt = dt_read_and_reset();
        if (dt < 0.001f) dt = 0.001f;   /* floor at 1 ms */
        if (dt > 0.050f) dt = 0.050f;   /* cap at 50 ms  */

        /* ---- Read gyro & integrate ---- */
        int16_t raw = -lsm_read_gyro_x();
        float rate  = ((float)raw - gyro_bias) * DPS_PER_LSB;  /* °/s */
        angle += rate * dt;

        /* ---- PID on angle error ---- */
        float error = -angle;           /* target = 0° */

        integral += error * dt;
        if (integral >  I_LIMIT) integral =  I_LIMIT;
        if (integral < -I_LIMIT) integral = -I_LIMIT;

        float derivative = (error - prev_err) / dt;
        prev_err = error;

        float output = KP * error + KI * integral + KD * derivative;

        /* ---- Drive servo ---- */
        int32_t ticks = (int32_t)SERVO_MID + (int32_t)(output * TICKS_PER_DEG);
        servo_set(ticks);

        /* ---- Log every ~100 ms ---- */
        if (++log_div >= 10)
        {
            log_div = 0;
            uart_int((int32_t)(dt * 1000000.0f)); uart_putchar(',');
            uart_int(raw);                         uart_putchar(',');
            uart_int((int32_t)(angle * 100.0f));   uart_putchar(',');
            uart_int(ticks);                       uart_print("\r\n");
        }

        _delay_ms(8);  /* ~10 ms loop with I2C overhead */
    }
}
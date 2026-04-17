#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

/* ── LSM6DSO ─────────────────────────────────────────────── */
#define LSM_ADDR        0x6B
#define LSM_CTRL2_G     0x11
#define LSM_CTRL3_C     0x12
#define LSM_OUTX_L_G    0x22   /* burst start: X(L/H) Y(L/H) Z(L/H) */
#define LSM_GYRO_CFG    0x4C   /* 104 Hz, ±2000 dps */
#define DPS_PER_LSB     0.070f

/* ── I2C status codes ────────────────────────────────────── */
#define TW_MT_SLA_ACK   0x18
#define TW_MR_SLA_ACK   0x40
#define TWI_STATUS      (TWSR0 & 0xF8)

/* ── Servos ──────────────────────────────────────────────── *
 * Servo 1 (roll)  — Timer1 OC1A / PB1                       *
 * Servo 2 (pitch) — Timer1 OC1B / PB2                       *
 *                                                            *
 * DS3218 270°: PWM 500–2500 µs → 0–270°                     *
 * Prescaler /8, 16 MHz → 1 tick = 0.5 µs                    *
 *   500 µs = 1000 ticks  2500 µs = 5000 ticks               *
 */
#define SERVO_MIN      1000U
#define SERVO_MID      3000U
#define SERVO_MAX      5000U
#define TICKS_PER_DEG  14.81f
#define OUT_LIMIT      135.0f

/* ── PID gains ───────────────────────────────────────────── *
 * Roll and pitch may need different tuning depending on the  *
 * inertia of each axis. Start equal and tune separately.     */
#define KP_ROLL   2.0f
#define KI_ROLL   0.2f
#define KD_ROLL   0.01f

#define KP_PITCH  2.3f
#define KI_PITCH  0.2f
#define KD_PITCH  0.01f

#define I_LIMIT   30.0f

/* ── UART ────────────────────────────────────────────────── */
#define UBRR_VAL  ((F_CPU / (16UL * 9600UL)) - 1)

static void uart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)(UBRR_VAL);
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
static void uart_putchar(char c) { while (!(UCSR0A & (1<<UDRE0))); UDR0 = c; }
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

/* ── LSM6DSO ─────────────────────────────────────────────── */
static void lsm_write(uint8_t reg, uint8_t val)
{
    i2c_start(LSM_ADDR << 1); i2c_write(reg); i2c_write(val); i2c_stop();
}

/* Single burst read: all three gyro axes from one transaction.
 * IF_INC is set in CTRL3_C so the register pointer auto-increments. */
static void lsm_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    i2c_start(LSM_ADDR << 1);
    i2c_write(LSM_OUTX_L_G);
    i2c_start((LSM_ADDR << 1) | 1);
    uint8_t xl = i2c_read(1); uint8_t xh = i2c_read(1);
    uint8_t yl = i2c_read(1); uint8_t yh = i2c_read(1);
    uint8_t zl = i2c_read(1); uint8_t zh = i2c_read(0);
    i2c_stop();
    *gx = (int16_t)((uint16_t)(xh << 8) | xl);
    *gy = (int16_t)((uint16_t)(yh << 8) | yl);
    *gz = (int16_t)((uint16_t)(zh << 8) | zl);
}

static void lsm_init(void)
{
    _delay_ms(20);
    lsm_write(LSM_CTRL3_C, 0x01);          /* software reset */
    _delay_ms(50);
    lsm_write(LSM_CTRL3_C, 0x44);          /* BDU=1, IF_INC=1 */
    lsm_write(LSM_CTRL2_G, LSM_GYRO_CFG);  /* 104 Hz, ±2000 dps */
    _delay_ms(10);
}

/* ── Gyro bias (both axes calibrated together) ───────────── */
static float bias_y = 0.0f;
static float bias_z = 0.0f;

static void calibrate_gyro(void)
{
    uart_print("Calibrating (hold still)...\r\n");
    int32_t sy = 0, sz = 0;
    for (uint16_t i = 0; i < 100; i++) {
        int16_t gx, gy, gz;
        lsm_read_gyro(&gx, &gy, &gz);
        sy += gy; sz += gz;
        _delay_ms(5);
    }
    bias_y = (float)sy / 100.0f;
    bias_z = (float)sz / 100.0f;
    uart_print("Bias Y: "); uart_int((int32_t)(bias_y * 100)); uart_print("/100\r\n");
    uart_print("Bias Z: "); uart_int((int32_t)(bias_z * 100)); uart_print("/100\r\n");
}

/* ── dt timer (Timer0, /1024, 64 µs/tick) ───────────────── */
static void dt_timer_init(void)
{
    TCCR0A = 0;
    TCCR0B = (1 << CS02) | (1 << CS00);
    TCNT0  = 0;
    TIFR0  = (1 << TOV0);
}
static float dt_read_and_reset(void)
{
    uint8_t cnt = TCNT0;
    uint8_t ovf = TIFR0 & (1 << TOV0);
    TCNT0 = 0; TIFR0 = (1 << TOV0);
    uint16_t ticks = ovf ? (256U + cnt) : cnt;
    return (float)ticks * (1024.0f / (float)F_CPU);
}

/* ── Servos (Timer1, both channels) ─────────────────────── */
static void servo_init(void)
{
    DDRB  |= (1 << PB1) | (1 << PB2);   /* OC1A + OC1B outputs */
    ICR1   = 39999;                      /* 20 ms period */
    OCR1A  = SERVO_MID;
    OCR1B  = SERVO_MID;
    /* COM1A1 + COM1B1: non-inverting PWM on both channels */
    TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);
    TCCR1B = (1<<WGM13)  | (1<<WGM12)  | (1<<CS11); /* /8 */
}
static void servo_set_a(int32_t ticks)  /* roll */
{
    if (ticks < (int32_t)SERVO_MIN) ticks = SERVO_MIN;
    if (ticks > (int32_t)SERVO_MAX) ticks = SERVO_MAX;
    OCR1A = (uint16_t)ticks;
}
static void servo_set_b(int32_t ticks)  /* pitch */
{
    if (ticks < (int32_t)SERVO_MIN) ticks = SERVO_MIN;
    if (ticks > (int32_t)SERVO_MAX) ticks = SERVO_MAX;
    OCR1B = (uint16_t)ticks;
}

/* ── PID helper — one call per axis ─────────────────────── */
static float pid_update(float error,
                        float kp, float ki, float kd,
                        float *integral, float *prev_err,
                        uint8_t *first, float dt)
{
    if (*first) { *prev_err = error; *first = 0; }

    float derivative = (error - *prev_err) / dt;
    *prev_err = error;

    float output = kp * error + ki * (*integral) + kd * derivative;

    uint8_t sat_hi = (output >  OUT_LIMIT) && (error > 0.0f);
    uint8_t sat_lo = (output < -OUT_LIMIT) && (error < 0.0f);
    if (!sat_hi && !sat_lo) {
        *integral += error * dt;
        if (*integral >  I_LIMIT) *integral =  I_LIMIT;
        if (*integral < -I_LIMIT) *integral = -I_LIMIT;
        output = kp * error + ki * (*integral) + kd * derivative;
    }
    if (output >  OUT_LIMIT) output =  OUT_LIMIT;
    if (output < -OUT_LIMIT) output = -OUT_LIMIT;

    return output;
}

/* ── Main ────────────────────────────────────────────────── */
int main(void)
{
    uart_init();
    i2c_init();
    servo_init();
    dt_timer_init();

    uart_print("\r\n=== 2-Axis Stabiliser ===\r\n");
    lsm_init();
    calibrate_gyro();

    uart_print("dt_us,pitch_x100,roll_x100,s1,s2\r\n");

    /* pitch (Y axis) state */
    float angle_pitch  = 0.0f;
    float integral_p   = 0.0f;
    float prev_err_p   = 0.0f;
    uint8_t first_p    = 1;

    /* roll (Z axis) state */
    float angle_roll   = 0.0f;
    float integral_r   = 0.0f;
    float prev_err_r   = 0.0f;
    uint8_t first_r    = 1;

    uint8_t log_div    = 0;

    (void)dt_read_and_reset();   /* prime timer */

    while (1)
    {
        float dt = dt_read_and_reset();
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.050f) dt = 0.050f;

        /* Single burst read — both axes from the same sample */
        int16_t gx, gy, gz;
        lsm_read_gyro(&gx, &gy, &gz);

        float rate_pitch = ((float)gy) * DPS_PER_LSB;
        float rate_roll  = ((float)gz) * DPS_PER_LSB;
        
//        float rate_pitch = ((float)gy - bias_y) * DPS_PER_LSB;
//        float rate_roll  = ((float)gz - bias_z) * DPS_PER_LSB;

        angle_pitch += rate_pitch * dt;
        angle_roll  += rate_roll  * dt;

        float out_pitch = pid_update(-angle_pitch,
                                     KP_PITCH, KI_PITCH, KD_PITCH,
                                     &integral_p, &prev_err_p, &first_p, dt);

        float out_roll  = pid_update(-angle_roll,
                                     KP_ROLL, KI_ROLL, KD_ROLL,
                                     &integral_r, &prev_err_r, &first_r, dt);

        int32_t ticks_pitch = (int32_t)SERVO_MID + (int32_t)(out_pitch * TICKS_PER_DEG);
        int32_t ticks_roll  = (int32_t)SERVO_MID + (int32_t)(out_roll  * TICKS_PER_DEG);

        servo_set_b(ticks_pitch);   /* OC1B / PB2 */
        servo_set_a(ticks_roll);    /* OC1A / PB1 */

        if (++log_div >= 10) {
            log_div = 0;
            uart_int((int32_t)(dt * 1000000.0f)); uart_putchar(',');
            uart_int((int32_t)(angle_pitch * 100.0f)); uart_putchar(',');
            uart_int((int32_t)(angle_roll  * 100.0f)); uart_putchar(',');
            uart_int(ticks_pitch); uart_putchar(',');
            uart_int(ticks_roll);  uart_print("\r\n");
        }

        _delay_ms(8);
    }
}
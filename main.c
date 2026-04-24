#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>

#define LSM_ADDR_BASE   0x6A
#define LSM_ADDR_ARM    0x6B

#define LSM_CTRL2_G     0x11
#define LSM_CTRL3_C     0x12
#define LSM_OUTX_L_G    0x22

#define LSM_GYRO_CFG    0x4C
#define DPS_PER_LSB     0.105f

#define TW_MT_SLA_ACK   0x18
#define TW_MR_SLA_ACK   0x40
#define TWI_STATUS      (TWSR0 & 0xF8)

/* ── Drift compensation ────────────────────────────────────── */
#define STILL_THRESH    0.5f
#define STILL_COUNT_MIN 30
#define BIAS_ALPHA      0.02f

/* ── Per-axis frame alignment (applied to raw rate) ────────── */
#define BASE_RATE_SIGN_X   (-1)
#define BASE_RATE_SIGN_Y   (+1)
#define BASE_RATE_SIGN_Z   (-1)

#define ARM_RATE_SIGN_X    (+1)
#define ARM_RATE_SIGN_Y    (-1)
#define ARM_RATE_SIGN_Z    (-1)

/* ── Servos ────────────────────────────────────────────────── */
#define SERVO_MIN       1000U    /*  500 µs →   0° */
#define SERVO_MID       3000U    /* 1500 µs → 135° */
#define SERVO_ROLL      1666U
#define SERVO_MAX       5000U    /* 2500 µs → 270° */
#define TICKS_PER_DEG   14.81f

#define Y_SERVO_PORT        PORTD
#define Y_SERVO_DDR         DDRD
#define Y_SERVO_PIN         PD4

#define Y_SERVO_MIN         31U     /*  496 µs */
#define Y_SERVO_MID         94U     /* 1504 µs */
#define Y_SERVO_MAX         156U    /* 2496 µs */
#define Y_TICKS_PER_DEG     (125.0f / 270.0f)   /* ≈ 0.463 */

/* ── Controller gains ──────────────────────────────────────── */
#define KP              1.0f
#define KI              0.1f
#define KD              0.01f
#define I_MAX           500.0f

#define AXIS_SIGN_Z     (-1)
#define AXIS_SIGN_X     (-1)
#define AXIS_SIGN_Y     (-1)

/* ── Debug / bring-up switches ─────────────────────────────── */
#define FF_ONLY_X       0
#define FF_ONLY_Z       1
#define FF_ONLY_Y       1

/* ── UART ──────────────────────────────────────────────────── */
#define UBRR_VAL        ((F_CPU / (16UL * 9600UL)) - 1)

/* ── I2C ───────────────────────────────────────────────────── */
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

/* ── LSM6DSO helpers ───────────────────────────────────────── */
static void lsm_write(uint8_t addr, uint8_t reg, uint8_t val)
{
    i2c_start(addr << 1); i2c_write(reg); i2c_write(val); i2c_stop();
}
static void lsm_read_gyro_xyz(uint8_t addr,
                              int16_t *x, int16_t *y, int16_t *z)
{
    i2c_start(addr << 1); i2c_write(LSM_OUTX_L_G);
    i2c_start((addr << 1) | 1);
    uint8_t xl = i2c_read(1);
    uint8_t xh = i2c_read(1);
    uint8_t yl = i2c_read(1);
    uint8_t yh = i2c_read(1);
    uint8_t zl = i2c_read(1);
    uint8_t zh = i2c_read(0);
    i2c_stop();
    *x = (int16_t)((uint16_t)(xh << 8) | xl);
    *y = (int16_t)((uint16_t)(yh << 8) | yl);
    *z = (int16_t)((uint16_t)(zh << 8) | zl);
}
static void lsm_init(uint8_t addr)
{
    _delay_ms(20);
    lsm_write(addr, LSM_CTRL3_C, 0x01);
    _delay_ms(50);
    lsm_write(addr, LSM_CTRL2_G, LSM_GYRO_CFG);
    _delay_ms(10);
}

/* ── Per-IMU state ─────────────────────────────────────────── */
typedef struct {
    float    bias_x,  bias_y,  bias_z;
    float    angle_x, angle_y, angle_z;
    uint16_t still_x, still_y, still_z;
    float    rate_x,  rate_y,  rate_z;
    int8_t   sign_x,  sign_y,  sign_z;
} imu_state_t;

static imu_state_t arm_imu;
static imu_state_t base_imu;

/* ── Startup bias estimation ───────────────────────────────── */
static void calibrate_imu(uint8_t addr, const char *label, imu_state_t *st)
{
    int32_t sx = 0, sy = 0, sz = 0;
    const uint16_t N = 1000;
    for (uint16_t i = 0; i < N; i++) {
        int16_t rx, ry, rz;
        lsm_read_gyro_xyz(addr, &rx, &ry, &rz);
        sx += rx; sy += ry; sz += rz;
        _delay_ms(3);
    }
    st->bias_x = (float)sx / (float)N;
    st->bias_y = (float)sy / (float)N;
    st->bias_z = (float)sz / (float)N;
}

/* ── Timer0 — precise dt ───────────────────────────────────── */
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
    TCNT0 = 0;
    TIFR0 = (1 << TOV0);
    uint16_t ticks = ovf ? (256U + cnt) : cnt;
    return (float)ticks * (1024.0f / (float)F_CPU);
}


static volatile uint8_t y_pulse_ticks = Y_SERVO_MID;

static inline void y_servo_set(uint8_t ticks)
{
    /* 8-bit store is atomic on AVR; no cli/sei needed. */
    y_pulse_ticks = ticks;
}

ISR(TIMER1_OVF_vect)
{
    Y_SERVO_PORT |= (1 << Y_SERVO_PIN);
    TCNT2  = 0;
    OCR2A  = y_pulse_ticks;
    TIFR2  = (1 << OCF2A);          /* clear any stale flag */
    TIMSK2 |= (1 << OCIE2A);        /* arm the end-of-pulse */
}

ISR(TIMER2_COMPA_vect)
{
    Y_SERVO_PORT &= ~(1 << Y_SERVO_PIN);
    TIMSK2 &= ~(1 << OCIE2A);       /* one-shot: disarm */
}

static void servo_init(void)
{
    /* Timer1: OC1A = Z-servo (PB1), OC1B = X-servo (PB2). */
    DDRB  |= (1 << PB1) | (1 << PB2);
    ICR1   = 39999;
    OCR1A  = SERVO_MID;
    OCR1B  = SERVO_ROLL;
    TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);
    TCCR1B = (1<<WGM13)  | (1<<WGM12)  | (1<<CS11);
    TIMSK1 = (1 << TOIE1);          /* OVF drives Y-servo frame */

    /* Timer2: Normal mode, /256 → 16 µs per tick.
     * No OCx pin output — OCR2A is used only for its ISR. */
    Y_SERVO_DDR  |=  (1 << Y_SERVO_PIN);
    Y_SERVO_PORT &= ~(1 << Y_SERVO_PIN);
    TCCR2A = 0;
    TCCR2B = (1 << CS22) | (1 << CS21);   /* /256 */
    TCNT2  = 0;
    OCR2A  = Y_SERVO_MID;
    TIFR2  = (1 << OCF2A);
    TIMSK2 = 0;                     /* armed per-frame in TIMER1_OVF */

    sei();
}


/* ── Integration helpers ───────────────────────────────────── */
static void integrate_axis(int16_t raw, int8_t sign,
                           float *bias, float *angle,
                           uint16_t *still_cnt, float dt, float *rate_out)
{
    float rate = ((float)raw - *bias) * DPS_PER_LSB * (float)sign;

    if (fabsf(rate) < STILL_THRESH) {
        if (*still_cnt < 0xFFFF) (*still_cnt)++;
        if (*still_cnt > STILL_COUNT_MIN) {
            *bias = (1.0f - BIAS_ALPHA) * (*bias) + BIAS_ALPHA * (float)raw;
        }
    } else {
        *still_cnt = 0;
    }

    *angle += rate * dt;

    if (rate_out) *rate_out = rate;
}

static void update_imu(uint8_t addr, imu_state_t *st, float dt)
{
    int16_t rx, ry, rz;
    lsm_read_gyro_xyz(addr, &rx, &ry, &rz);
    integrate_axis(rx, st->sign_x,
                   &st->bias_x, &st->angle_x, &st->still_x, dt, &st->rate_x);
    integrate_axis(ry, st->sign_y,
                   &st->bias_y, &st->angle_y, &st->still_y, dt, &st->rate_y);
    integrate_axis(rz, st->sign_z,
                   &st->bias_z, &st->angle_z, &st->still_z, dt, &st->rate_z);
}

static uint16_t clamp_ticks(int32_t t)
{
    if (t < (int32_t)SERVO_MIN) return SERVO_MIN;
    if (t > (int32_t)SERVO_MAX) return SERVO_MAX;
    return (uint16_t)t;
}

void init_sequence() {
    i2c_init();
    servo_init();
    dt_timer_init();

    base_imu.sign_x = BASE_RATE_SIGN_X;
    base_imu.sign_y = BASE_RATE_SIGN_Y;
    base_imu.sign_z = BASE_RATE_SIGN_Z;
    arm_imu.sign_x  = ARM_RATE_SIGN_X;
    arm_imu.sign_y  = ARM_RATE_SIGN_Y;
    arm_imu.sign_z  = ARM_RATE_SIGN_Z;

    lsm_init(LSM_ADDR_BASE);
    lsm_init(LSM_ADDR_ARM);

    calibrate_imu(LSM_ADDR_BASE, "BASE", &base_imu);
    calibrate_imu(LSM_ADDR_ARM,  "ARM ", &arm_imu);
    
    PORTD |= (1 << PORTD0);
    _delay_ms(1000);
    PORTD &= ~(1 << PORTD0);
}

/* ── Main ──────────────────────────────────────────────────── */
int main(void)
{
    DDRD |= (1<<DDD0);
    DDRD &= ~(1 <<DDD1);
    PORTD |= (1 << PORTC1); 
    init_sequence();
    
    float integ_z = 0.0;
    float integ_x = 0.0f;
    float integ_y = 0.0f;

    (void)dt_read_and_reset();
    
    while (1)
    {
        if (!(PIND & (1 << PIND1))) {
            init_sequence();
            
            base_imu.angle_x = base_imu.angle_y = base_imu.angle_z = 0.0f;
            arm_imu.angle_x  = arm_imu.angle_y  = arm_imu.angle_z  = 0.0f;
            base_imu.still_x = base_imu.still_y = base_imu.still_z = 0;
            arm_imu.still_x  = arm_imu.still_y  = arm_imu.still_z  = 0;
            
            integ_z = 0.0f;
            integ_x = 0.0f;
            integ_y = 0.0f;
            
            (void)dt_read_and_reset();
        }
        /* ── Timing ── */
        float dt = dt_read_and_reset();
        if (dt < 0.002f) dt = 0.002f;
        if (dt > 0.050f) dt = 0.050f;

        /* ── Read + integrate both IMUs ── */
        update_imu(LSM_ADDR_BASE, &base_imu, dt);
        update_imu(LSM_ADDR_ARM,  &arm_imu,  dt);

        float theta = base_imu.angle_x * (M_PI / 180.0f);
        float cx = cosf(theta), sx = sinf(theta);

        float psi   = base_imu.angle_z * (M_PI / 180.0f);
        float cz = cosf(psi),   sz = sinf(psi);
        
        float u_ff_z = -(base_imu.angle_z * cx + base_imu.angle_y * sx);
        float err_z  = arm_imu.angle_x;
        float dE_z   = arm_imu.rate_x;
        integ_z += err_z * dt;
        if (integ_z >  I_MAX) integ_z =  I_MAX;
        if (integ_z < -I_MAX) integ_z = -I_MAX;

#if FF_ONLY_Z
        float u_pid_z = 0.0f;
#else
        float u_pid_z = (KP * err_z) + (KI * integ_z) + (KD * dE_z);
#endif
        float u_z = (u_ff_z + u_pid_z) * (float)AXIS_SIGN_Z;

        int32_t ticks_z = (int32_t)SERVO_MID + (int32_t)(u_z * TICKS_PER_DEG);
        OCR1A = clamp_ticks(ticks_z);

        float u_ff_x = -(base_imu.angle_y * cz + base_imu.angle_x * sz);
        float err_x  =  arm_imu.angle_y;
        float dE_x   =  arm_imu.rate_y;
        integ_x += err_x * dt;
        if (integ_x >  I_MAX) integ_x =  I_MAX;
        if (integ_x < -I_MAX) integ_x = -I_MAX;

#if FF_ONLY_X
        float u_pid_x = 0.0f;
#else
        float u_pid_x = (KP * err_x) + (KI * integ_x) + (KD * dE_x);
#endif
        float u_x = (u_ff_x + u_pid_x) * (float)AXIS_SIGN_X;

        int32_t ticks_x = (int32_t)SERVO_ROLL + (int32_t)(u_x * TICKS_PER_DEG);
        OCR1B = clamp_ticks(ticks_x);

        float u_ff_y = -(base_imu.angle_x * cz - base_imu.angle_y * sz);
        float err_y  =  arm_imu.angle_z;
        float dE_y   =  arm_imu.rate_z;
        integ_y += err_y * dt;
        if (integ_y >  I_MAX) integ_y =  I_MAX;
        if (integ_y < -I_MAX) integ_y = -I_MAX;

#if FF_ONLY_Y
        float u_pid_y = 0.0f;
#else
        float u_pid_y = (KP * err_y) + (KI * integ_y) + (KD * dE_y);
#endif
        float u_y = (u_ff_y + u_pid_y) * (float)AXIS_SIGN_Y;

//        int32_t ticks_y = (int32_t)SERVO_Y_MID + (int32_t)(u_y * TICKS_PER_DEG);
//        y_servo_set(clamp_ticks(ticks_y));
        int32_t ticks_y = (int32_t)Y_SERVO_MID + (int32_t)(u_y * Y_TICKS_PER_DEG);
        if (ticks_y < (int32_t)Y_SERVO_MIN) ticks_y = Y_SERVO_MIN;
        if (ticks_y > (int32_t)Y_SERVO_MAX) ticks_y = Y_SERVO_MAX;
        y_servo_set((uint8_t)ticks_y);

        _delay_ms(5);
    }
}
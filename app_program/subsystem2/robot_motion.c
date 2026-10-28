#include "robot_motion.h"
#include "../subsystem3/robot_line.h"

/* Motor outputs: Cytron Robo Pico M1=GP8/9 M2=GP10/11.
 * Verify board driver and wheel polarity BEFORE enabling hardware.
 * Exclusive motor writer: this task only.
 */
#define MOTOR_L_A 8U
#define MOTOR_L_B 9U
#define MOTOR_R_A 10U
#define MOTOR_R_B 11U
#define MOTOR_MAX_PERCENT 20
#define MOTOR_PERIOD_MS 20U
static volatile int requested_left;
static volatile int requested_right;
static volatile int emergency_stop;
static ID motion_tid;

static int limit(int n)
{
    if (n > MOTOR_MAX_PERCENT) return MOTOR_MAX_PERCENT;
    if (n < -MOTOR_MAX_PERCENT) return -MOTOR_MAX_PERCENT;
    return n;
}

#if ROBOT_MOTOR_ENABLE
static void apply_one(UINT pin_f, UINT pin_r, int speed)
{
    /* Each motor uses one forward PWM and one reverse PWM.
     * Requires confirmation of exact Robo Pico bridge mapping.
     * pwm_set_cc takes the GPIO pin and channel compare value.
     */
    UW duty = (UW)((speed < 0 ? -speed : speed) * 100U);
    if (speed > 0) {
        pwm_set_cc(pin_r, 0);
        pwm_set_cc(pin_f, duty);
    } else if (speed < 0) {
        pwm_set_cc(pin_f, 0);
        pwm_set_cc(pin_r, duty);
    } else {
        pwm_set_cc(pin_f, 0);
        pwm_set_cc(pin_r, 0);
    }
}
#endif

static void hardware_stop(void)
{
#if ROBOT_MOTOR_ENABLE
    apply_one(MOTOR_L_A, MOTOR_L_B, 0);
    apply_one(MOTOR_R_A, MOTOR_R_B, 0);
#endif
}

void robot_motion_command(int left_pct, int right_pct)
{
    /* Both tasks are pinned to core 0; command changes occur in
     * the line task, and outputs are written only in motion task. */
    requested_left = limit(left_pct);
    requested_right = limit(right_pct);
}
void robot_motion_stop(void)
{
    requested_left = 0;
    requested_right = 0;
    emergency_stop = 1;
    /* Hardware output is disabled at the next motion task tick.
       20 ms nominal, verify worst-case latency on target. */
}
void robot_motion_fault(void) { robot_motion_stop(); }

static void motion_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;
    for (;;) {
        int l = requested_left;
        int r = requested_right;
        if (emergency_stop || robot_line_is_faulted()) {
            l = 0; r = 0;
        }
#if ROBOT_MOTOR_ENABLE
        apply_one(MOTOR_L_A, MOTOR_L_B, l);
        apply_one(MOTOR_R_A, MOTOR_R_B, r);
#else
        (void)l; (void)r;
#endif
        tk_dly_tsk(MOTOR_PERIOD_MS);
    }
}
ER robot_motion_init(void)
{
    requested_left = requested_right = 0;
    emergency_stop = 0;
#if ROBOT_MOTOR_ENABLE
    const UINT pins[] = {MOTOR_L_A,MOTOR_L_B,MOTOR_R_A,MOTOR_R_B};
    UINT i;
    for (i = 0; i < 4; ++i) {
        if (pwm_set_pin(pins[i]) < E_OK) return -1;
        if (pwm_set_wrap(pins[i], 9999U) < E_OK) return -1;
        if (pwm_set_cc(pins[i], 0U) < E_OK) return -1;
        if (pwm_set_enabled(pins[i], TRUE) < E_OK) return -1;
    }
#endif
    hardware_stop();
    return E_OK;
}
ER robot_motion_start(void)
{
    motion_tid = app_start_task((FP)motion_task, 10, TP_PRC1, "robot_motion");
    return motion_tid < E_OK ? motion_tid : E_OK;
}

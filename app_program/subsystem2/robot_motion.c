#include "robot_motion.h"
#include "../subsystem3/robot_line.h"

#define M1A 8U
#define M1B 9U
#define M2A 10U
#define M2B 11U
#define PWM_TOP 9999U
#define TASK_MS 20U

static volatile int demand_l, demand_r, fault_latched;
static ID motor_tid;
static int clamp_pwm(int v) {
    if (v > MOTOR_PWM_MAX_PERCENT) return MOTOR_PWM_MAX_PERCENT;
    if (v < -MOTOR_PWM_MAX_PERCENT) return -MOTOR_PWM_MAX_PERCENT;
    return v;
}
static void output_motor(UINT a, UINT b, int percent) {
    UW compare = (UW)((percent < 0 ? -percent : percent) * 100);
    /* Both outputs are 0 on stop; never command forward and reverse together. */
    if (percent > 0) { pwm_set_cc(b,0); pwm_set_cc(a,compare); }
    else if (percent < 0) { pwm_set_cc(a,0); pwm_set_cc(b,compare); }
    else { pwm_set_cc(a,0); pwm_set_cc(b,0); }
}
static void stop_hw(void) { output_motor(M1A,M1B,0); output_motor(M2A,M2B,0); }
void robot_motion_command(int l, int r) {
    if (fault_latched) return;
    demand_l=clamp_pwm(l);
    demand_r=clamp_pwm(r);
}
void robot_motion_stop(void) {
    fault_latched=1;
    demand_l=demand_r=0;
    /* Safety bypasses task period: force PWM to zero now. */
    stop_hw();
}
void robot_motion_fault(void) { robot_motion_stop(); }
static void task_motion(INT stacd, void *exinf) {
    (void)stacd; (void)exinf;
    tm_printf((UB*)"[MOTOR] motion task active; PWM enabled\n");
    for (;;) {
        if (fault_latched || robot_line_is_faulted()) stop_hw();
        else {
            output_motor(M1A,M1B,demand_l * LEFT_FORWARD_SIGN);
            output_motor(M2A,M2B,demand_r * RIGHT_FORWARD_SIGN);
        }
        tk_dly_tsk(TASK_MS);
    }
}
ER robot_motion_init(void) {
    static const UINT pins[]={M1A,M1B,M2A,M2B};
    UINT i;
    fault_latched=0;
    demand_l=demand_r=0;
    for(i=0;i<4;i++) {
        ER er;
        if ((er=pwm_set_cc(pins[i],0))<E_OK) return er;
        if ((er=pwm_set_wrap(pins[i],PWM_TOP))<E_OK) return er;
        if ((er=pwm_set_pin(pins[i]))<E_OK) return er;
        if ((er=pwm_set_enabled(pins[i],TRUE))<E_OK) return er;
    }
    stop_hw();
    tm_printf((UB*)"[MOTOR] GP8/9 M1, GP10/11 M2; max duty=%d%%\n", MOTOR_PWM_MAX_PERCENT);
    return E_OK;
}
ER robot_motion_start(void) {
    motor_tid=app_start_task((FP)task_motion,10,TP_PRC1,"robot_motion");
    return motor_tid < E_OK ? motor_tid : E_OK;
}

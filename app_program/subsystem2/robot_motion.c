#include "robot_motion.h"
#include "drv_pwm.h"
#include "../subsystem3/robot_line.h"

#define MOTOR_L_A_PIN 8U
#define MOTOR_L_B_PIN 9U
#define MOTOR_R_A_PIN 10U
#define MOTOR_R_B_PIN 11U
#define MOTOR_L_SLICE 4U
#define MOTOR_R_SLICE 5U
#define MOTOR_PWM_HZ 10000U
#define MOTOR_PWM_TOP (PWM_CLK_HZ / MOTOR_PWM_HZ - 1UL)
#define MOTOR_TASK_MS 20U

static volatile int demand_left, demand_right, fault_latched;
static ID motor_tid;
static int limit(int v) {
    if (v > MOTOR_PWM_MAX_PERCENT) return MOTOR_PWM_MAX_PERCENT;
    if (v < -MOTOR_PWM_MAX_PERCENT) return -MOTOR_PWM_MAX_PERCENT;
    return v;
}
static void motor_write(UW slice, int percent) {
    UW cc = ((UW)(percent < 0 ? -percent : percent) * (MOTOR_PWM_TOP + 1U))/100U;
    if (percent > 0) pwm_slice_write_cc(slice,cc,0);
    else if (percent < 0) pwm_slice_write_cc(slice,0,cc);
    else pwm_slice_write_cc(slice,0,0);
}
static void stop_hw(void) {
    pwm_slice_write_cc(MOTOR_L_SLICE,0,0);
    pwm_slice_write_cc(MOTOR_R_SLICE,0,0);
}
void robot_motion_command(int left, int right) {
    if (fault_latched) return;
    demand_left=limit(left);
    demand_right=limit(right);
}
void robot_motion_stop(void) {
    fault_latched=1;
    demand_left=demand_right=0;
    stop_hw();
}
void robot_motion_fault(void) {robot_motion_stop();}
static void task_motion(INT stacd, void *exinf) {
    unsigned log_ticks=0;
    int l,r;
    (void)stacd;(void)exinf;
    tm_printf((UB*)"[MOTOR] legacy PWM block initialized, 10 kHz\n");
    for (;;) {
        l=demand_left; r=demand_right;
        if (fault_latched || robot_line_is_faulted()) {l=r=0;stop_hw();}
        else {
            motor_write(MOTOR_L_SLICE,l*LEFT_FORWARD_SIGN);
            motor_write(MOTOR_R_SLICE,r*RIGHT_FORWARD_SIGN);
        }
        if (++log_ticks >= 50U) {
            log_ticks=0;
            tm_printf((UB*)"[MOTOR] L=%d R=%d fault=%d state=%d\n",
                       l,r,fault_latched,(int)robot_line_state());
        }
        tk_dly_tsk(MOTOR_TASK_MS);
    }
}
ER robot_motion_init(void) {
    ER er;
    if (!pwm_block_enable()) return E_IO;
    pwm_slice_setup(MOTOR_L_SLICE,1U,MOTOR_PWM_TOP);
    pwm_slice_setup(MOTOR_R_SLICE,1U,MOTOR_PWM_TOP);
    if ((er=pwm_set_pin(MOTOR_L_A_PIN))<E_OK) return er;
    if ((er=pwm_set_pin(MOTOR_L_B_PIN))<E_OK) return er;
    if ((er=pwm_set_pin(MOTOR_R_A_PIN))<E_OK) return er;
    if ((er=pwm_set_pin(MOTOR_R_B_PIN))<E_OK) return er;
    fault_latched=0;demand_left=demand_right=0;
    stop_hw();
    tm_printf((UB*)"[MOTOR] GP8/9 GP10/11 slices 4/5 enabled; cap=%d%%\n",
        MOTOR_PWM_MAX_PERCENT);
    return E_OK;
}
ER robot_motion_start(void) {
    motor_tid=app_start_task((FP)task_motion,10,TP_PRC1,"robot_motion");
    return motor_tid<E_OK?motor_tid:E_OK;
}

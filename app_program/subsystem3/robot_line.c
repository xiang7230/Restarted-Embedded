#include "robot_line.h"
#include "../subsystem2/robot_motion.h"
/* Set this to 0 if the sensor reports LOW over black. */
#define IR_BLACK_LEVEL 1U
#define IR_L 2U
#define IR_C 6U
#define IR_R 27U
#define PERIOD_MS 20U
#define STARTUP_SETTLE_MS 800U
#define STARTUP_OBSERVE_MS 240U
#define SEARCH_LIMIT_MS 2500U
#define RECOVERY_LIMIT_MS 2500U
#define CONFIRM_SAMPLES 3U
#define LOSS_SAMPLES 3U
/* Conservative starting values: adjust for motor minimum running PWM. */
#define CRUISE 24
#define SLOW 17
#define SWEEP 18
static volatile robot_line_state_t state;
static unsigned elapsed, present, absent;
static int last_side;
static ID tid_line;
static void enter(robot_line_state_t s) {
    state=s; elapsed=present=absent=0;
    tm_printf((UB*)"[LINE] state=%d\n",(int)s);
}
int robot_line_is_faulted(void){ return state==ROBOT_STOPPED; }
robot_line_state_t robot_line_state(void){ return state; }
static unsigned read_line(void){
    unsigned r=0;
    if(gpio_get_val(IR_L)==IR_BLACK_LEVEL) r|=4U;
    if(gpio_get_val(IR_C)==IR_BLACK_LEVEL) r|=2U;
    if(gpio_get_val(IR_R)==IR_BLACK_LEVEL) r|=1U;
    return r;
}
static int valid(unsigned b) { return b!=0 && b!=5; }
static void task_line(INT stacd, void *exinf){
    unsigned raw, previous=255U, logging=0;
    (void)stacd;(void)exinf;
    robot_motion_command(0,0);
    tk_dly_tsk(STARTUP_SETTLE_MS);
    tm_printf((UB*)"[LINE] sensor ready; black level=%u\n",IR_BLACK_LEVEL);
    enter(ROBOT_STARTUP);
    for(;;){
        raw=read_line();
        if(raw!=previous || ++logging>=50U){
            tm_printf((UB*)"[IR] L=%u C=%u R=%u state=%d\n",(raw>>2)&1U,(raw>>1)&1U,raw&1U,(int)state);
            previous=raw;logging=0;
        }
        if(elapsed<500000U)elapsed+=PERIOD_MS;
        switch(state){
        case ROBOT_STARTUP:
            robot_motion_command(0,0);
            if(valid(raw)){
                if(++present>=CONFIRM_SAMPLES)enter(ROBOT_FOLLOW);
            }else present=0;
            if(state==ROBOT_STARTUP && elapsed>=STARTUP_OBSERVE_MS) enter(ROBOT_INITIAL_SEARCH);
            break;
        case ROBOT_FOLLOW:
            if(raw==0){
                robot_motion_command(0,0); /* prevent continuing off line */
                if(++absent>=LOSS_SAMPLES)enter(ROBOT_RECOVERY);
            }else{
                absent=0;
                if(raw==4 || raw==6){last_side=-1;robot_motion_command(SLOW,CRUISE);}
                else if(raw==1 || raw==3){last_side=1;robot_motion_command(CRUISE,SLOW);}
                else if(raw==2){robot_motion_command(CRUISE,CRUISE);}
                else {robot_motion_command(SLOW,SLOW);} /* wide / ambiguous */
            }
            break;
        case ROBOT_INITIAL_SEARCH:
        case ROBOT_RECOVERY:
            if(valid(raw)){
                if(++present>=CONFIRM_SAMPLES){
                    enter(ROBOT_FOLLOW);
                    robot_motion_command(0,0);
                    break;
                }
            }else present=0;
            if(elapsed>=((state==ROBOT_INITIAL_SEARCH)?SEARCH_LIMIT_MS:RECOVERY_LIMIT_MS)){
                robot_motion_fault();enter(ROBOT_STOPPED);break;
            }
            {
                /* alternating search sweep; recovery favors last known side */
                int side = (state==ROBOT_RECOVERY && last_side!=0 && elapsed<650U)
                         ? last_side
                         : (((elapsed/350U)&1U)?1:-1);
                robot_motion_command(side<0?-SWEEP:SWEEP,side<0?SWEEP:-SWEEP);
            }
            break;
        default: robot_motion_fault();break;
        }
        tk_dly_tsk(PERIOD_MS);
    }
}
ER robot_line_init(void){
    const UINT pins[]={IR_L,IR_C,IR_R};
    UINT i;ER er;
    state=ROBOT_STARTUP;elapsed=present=absent=0;last_side=0;
    for(i=0;i<3;i++)if((er=gpio_set_pin(pins[i],GPIO_MODE_IN))<E_OK)return er;
    return E_OK;
}
ER robot_line_start(void){
    tid_line=app_start_task((FP)task_line,11,TP_PRC1,"robot_line");
    return tid_line<E_OK?tid_line:E_OK;
}

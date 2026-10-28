#include "robot_line.h"
#include "../subsystem2/robot_motion.h"

/* Verify IR sensor levels on your physical module. */
#define IR_BLACK_LEVEL 1U
#define IR_L 2U
#define IR_C 6U
#define IR_R 27U
#define LINE_PERIOD_MS 20U
#define SEARCH_TIMEOUT_MS 3000U
#define RECOVERY_TIMEOUT_MS 3000U
#define LOST_SAMPLES 3
#define FOUND_SAMPLES 3
/* Wheel percentages: cap is enforced by Buddy 2. */
#define CRUISE 15
#define STEER_SLOW 10
#define SEARCH 10

static volatile robot_line_state_t state;
static unsigned state_elapsed;
static unsigned seen;
static unsigned lost;
static int last_direction;
static ID line_tid;

int robot_line_is_faulted(void) { return state == ROBOT_STOPPED; }
robot_line_state_t robot_line_state(void) { return state; }

static void set_state(robot_line_state_t s)
{
    state=s;
    state_elapsed=0;
    seen=0;
    lost=0;
}
static unsigned sample_bits(void)
{
    unsigned b=0;
    if (gpio_get_val(IR_L) == IR_BLACK_LEVEL) b |= 4U;
    if (gpio_get_val(IR_C) == IR_BLACK_LEVEL) b |= 2U;
    if (gpio_get_val(IR_R) == IR_BLACK_LEVEL) b |= 1U;
    return b;
}
static void line_task(INT stacd, void *exinf)
{
    unsigned bits;
    (void)stacd; (void)exinf;
    tk_dly_tsk(250U); /* allow sensor power to stabilize */
    set_state(ROBOT_STARTUP);
    for (;;) {
        bits=sample_bits();
        if (state_elapsed < 1000000U) state_elapsed+=LINE_PERIOD_MS;
        if (state == ROBOT_STARTUP) {
            if (bits != 0U && bits != 5U) {
                if (++seen >= FOUND_SAMPLES) set_state(ROBOT_FOLLOW);
            } else if (state_elapsed >= 200U) {
                set_state(ROBOT_INITIAL_SEARCH);
            }
            robot_motion_command(0,0);
        } else if (state == ROBOT_FOLLOW) {
            if (bits == 0U) {
                if (++lost >= LOST_SAMPLES) {
                    set_state(ROBOT_RECOVERY);
                    robot_motion_command(0,0);
                }
            } else {
                lost=0;
                if (bits == 4U || bits == 6U) last_direction=-1;
                if (bits == 1U || bits == 3U) last_direction=1;
                if (bits == 2U) last_direction=0;
                if (bits == 4U || bits == 6U)
                    robot_motion_command(STEER_SLOW, CRUISE);
                else if (bits == 1U || bits == 3U)
                    robot_motion_command(CRUISE, STEER_SLOW);
                else if (bits == 2U)
                    robot_motion_command(CRUISE, CRUISE);
                else
                    robot_motion_command(STEER_SLOW,STEER_SLOW);
            }
        } else if (state == ROBOT_INITIAL_SEARCH ||
                   state == ROBOT_RECOVERY) {
            unsigned timeout = (state == ROBOT_INITIAL_SEARCH)
                             ? SEARCH_TIMEOUT_MS : RECOVERY_TIMEOUT_MS;
            if (bits != 0U && bits != 5U) {
                if (++seen >= FOUND_SAMPLES) {
                    set_state(ROBOT_FOLLOW);
                    robot_motion_command(0,0);
                }
            } else {
                seen=0;
            }
            if (state != ROBOT_FOLLOW) {
                if (state_elapsed >= timeout) {
                    set_state(ROBOT_STOPPED);
                    robot_motion_fault();
                } else {
                    /* First turn toward last-seen side for recovery.
                     * Without a known direction, alternate small sweeps.
                     * Timed sweeps are provisional until encoder control.
                     */
                    int side = (state == ROBOT_INITIAL_SEARCH)
                             ? ((state_elapsed / 400U) % 2U ? 1 : -1)
                             : (last_direction == 0
                                 ? ((state_elapsed / 400U) % 2U ? 1 : -1)
                                 : last_direction);
                    robot_motion_command(
                        side < 0 ? -SEARCH : SEARCH,
                        side < 0 ? SEARCH : -SEARCH
                    );
                }
            }
        } else {
            robot_motion_command(0,0);
        }
        tk_dly_tsk(LINE_PERIOD_MS);
    }
}
ER robot_line_init(void)
{
    const UINT pins[] = {IR_L,IR_C,IR_R};
    UINT i;
    for (i=0;i<3;++i) if (gpio_set_pin(pins[i],GPIO_MODE_IN)<E_OK) return -1;
    state=ROBOT_STARTUP;
    state_elapsed=seen=lost=0;
    last_direction=0;
    return E_OK;
}
ER robot_line_start(void)
{
    line_tid=app_start_task((FP)line_task, 11, TP_PRC1, "robot_line");
    return line_tid<E_OK ? line_tid:E_OK;
}

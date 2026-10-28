#include "robot_line.h"
#include "../subsystem2/robot_motion.h"

/* L/C/R digital IR. Confirm each sensor's electrical polarity before driving. */
#define IR_BLACK_LEVEL 1U
#define IR_L 2U
#define IR_C 6U
#define IR_R 27U

#define PERIOD_MS 20U
#define STARTUP_SETTLE_MS 650U
#define STARTUP_OBSERVE_MS 240U
#define START_SEARCH_LIMIT_MS 4000U
#define LOST_SEARCH_LIMIT_MS 4000U
#define REACQUIRE_SAMPLES 3U
#define LOST_SAMPLES 3U

/* Slow differential steering.
 * MIN_RUNNING=18 may need adjustment for your motors; cruise 26 <= max 30.
 * Sharp turn is controlled by only one wheel forward (not reverse),
 * except bounded recovery sweeps.
 */
#define CRUISE 26
#define INNER_GENTLE 18
#define INNER_SHARP 0
#define SEARCH_TURN 20

static volatile robot_line_state_t state;
static unsigned elapsed_ms, good_count, missing_count;
static int last_side; /* -1 = line on left, +1 = line on right, 0 = unknown */
static ID tid_line;

static void enter(robot_line_state_t next)
{
    state = next;
    elapsed_ms = 0;
    good_count = 0;
    missing_count = 0;
    tm_printf((UB*)"[LINE] state=%d\n", (int)next);
}
int robot_line_is_faulted(void) { return state == ROBOT_STOPPED; }
robot_line_state_t robot_line_state(void) { return state; }

static unsigned read_line_bits(void)
{
    unsigned bits = 0;
    if (gpio_get_val(IR_L) == IR_BLACK_LEVEL) bits |= 4U;
    if (gpio_get_val(IR_C) == IR_BLACK_LEVEL) bits |= 2U;
    if (gpio_get_val(IR_R) == IR_BLACK_LEVEL) bits |= 1U;
    return bits;
}
/* 101 is ambiguous and 111 can mean a barcode/junction. */
static int line_is_track(unsigned bits)
{
    return bits == 2U || bits == 4U || bits == 6U ||
           bits == 1U || bits == 3U;
}
static void steer_follow(unsigned bits)
{
    switch (bits) {
    case 2U: /* centered: preserve last_side for recovery */
        robot_motion_command(CRUISE, CRUISE);
        break;
    case 6U: /* line left of center */
        last_side = -1;
        robot_motion_command(INNER_GENTLE, CRUISE);
        break;
    case 4U: /* line far left: stronger correction */
        last_side = -1;
        robot_motion_command(INNER_SHARP, CRUISE);
        break;
    case 3U: /* line right of center */
        last_side = 1;
        robot_motion_command(CRUISE, INNER_GENTLE);
        break;
    case 1U: /* line far right: stronger correction */
        last_side = 1;
        robot_motion_command(CRUISE, INNER_SHARP);
        break;
    default:
        /* Wide mark or ambiguous sensor layout: avoid an aggressive turn. */
        robot_motion_command(0, 0);
        break;
    }
}
/* Two-sided slow turn, with longer sweep opposite the first attempt.
 * Initial search has no preferred side; recovery remembers last seen side.
 * Search does not travel forward blindly.
 */
static void steer_search(unsigned ms, int preferred)
{
    int first = preferred == 1 ? 1 : -1;
    int side;
    if (ms < 850U) side = first;
    else if (ms < 2550U) side = -first;
    else side = first;
    if (side < 0) robot_motion_command(-SEARCH_TURN, SEARCH_TURN);
    else          robot_motion_command(SEARCH_TURN, -SEARCH_TURN);
}
static void task_line(INT stacd, void *exinf)
{
    unsigned bits, previous = 255U, print_cycles = 0;
    (void)stacd; (void)exinf;
    robot_motion_command(0, 0);
    tk_dly_tsk(STARTUP_SETTLE_MS);
    tm_printf((UB*)"[LINE] IR black level=%u; L=GP2 C=GP6 R=GP27\n",
              IR_BLACK_LEVEL);
    enter(ROBOT_STARTUP);
    for (;;) {
        bits = read_line_bits();
        if (bits != previous || ++print_cycles >= 50U) {
            tm_printf((UB*)"[IR] L=%u C=%u R=%u state=%d\n",
                (bits >> 2)&1U, (bits >> 1)&1U, bits&1U, (int)state);
            previous = bits;
            print_cycles = 0;
        }
        if (elapsed_ms < 100000U) elapsed_ms += PERIOD_MS;
        switch (state) {
        case ROBOT_STARTUP:
            robot_motion_command(0, 0);
            if (line_is_track(bits)) {
                if (++good_count >= REACQUIRE_SAMPLES)
                    enter(ROBOT_FOLLOW);
            } else good_count = 0;
            if (state == ROBOT_STARTUP &&
                elapsed_ms >= STARTUP_OBSERVE_MS)
                enter(ROBOT_INITIAL_SEARCH);
            break;

        case ROBOT_FOLLOW:
            if (line_is_track(bits)) {
                missing_count = 0;
                steer_follow(bits);
            } else if (bits == 0U) {
                /* No black anywhere. Preserve previous direction. */
                robot_motion_command(0, 0);
                if (++missing_count >= LOST_SAMPLES)
                    enter(ROBOT_RECOVERY);
            } else {
                /* 101 / 111: no clear center line; stop rather than guess. */
                robot_motion_command(0, 0);
                if (++missing_count >= LOST_SAMPLES)
                    enter(ROBOT_RECOVERY);
            }
            break;

        case ROBOT_INITIAL_SEARCH:
        case ROBOT_RECOVERY:
            if (line_is_track(bits)) {
                if (++good_count >= REACQUIRE_SAMPLES) {
                    /* Apply correction immediately so a bend isn't overshot. */
                    enter(ROBOT_FOLLOW);
                    steer_follow(bits);
                    break;
                }
            } else {
                good_count = 0;
            }
            if (elapsed_ms >=
                (state == ROBOT_INITIAL_SEARCH ?
                 START_SEARCH_LIMIT_MS : LOST_SEARCH_LIMIT_MS)) {
                robot_motion_command(0,0);
                robot_motion_fault();
                enter(ROBOT_STOPPED);
                tm_printf((UB*)"[LINE] search timed out - motors stopped\n");
                break;
            }
            steer_search(elapsed_ms,
                (state == ROBOT_RECOVERY) ? last_side : 0);
            break;

        default:
            robot_motion_command(0,0);
            break;
        }
        tk_dly_tsk(PERIOD_MS);
    }
}
ER robot_line_init(void)
{
    const UINT pins[] = {IR_L, IR_C, IR_R};
    UINT i;
    ER er;
    state = ROBOT_STARTUP;
    elapsed_ms = good_count = missing_count = 0;
    last_side = 0;
    for (i = 0; i < 3; ++i) {
        er = gpio_set_pin(pins[i], GPIO_MODE_IN);
        if (er < E_OK) return er;
    }
    return E_OK;
}
ER robot_line_start(void)
{
    tid_line = app_start_task((FP)task_line, 11, TP_PRC1, "robot_line");
    return tid_line < E_OK ? tid_line : E_OK;
}

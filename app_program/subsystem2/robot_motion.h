#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H
#include "../app.h"
/* Hardware is deliberately disabled until wiring/polarity are verified. */
#ifndef ROBOT_MOTOR_ENABLE
#define ROBOT_MOTOR_ENABLE 0
#endif
ER robot_motion_init(void);
ER robot_motion_start(void);
void robot_motion_command(int left_pct, int right_pct);
void robot_motion_stop(void);
void robot_motion_fault(void);
#endif

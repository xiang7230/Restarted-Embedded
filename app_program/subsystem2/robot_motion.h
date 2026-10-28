#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H
#include "../app.h"
/* Hardware-enabled test firmware. Raise wheels for the first test. */
#define ROBOT_MOTOR_ENABLE 1
/* Set -1 if a wheel runs backwards, +1 normally. */
#define LEFT_FORWARD_SIGN 1
#define RIGHT_FORWARD_SIGN 1
#define MOTOR_PWM_MAX_PERCENT 30
ER robot_motion_init(void);
ER robot_motion_start(void);
void robot_motion_command(int left_pct, int right_pct);
void robot_motion_stop(void);
void robot_motion_fault(void);
#endif

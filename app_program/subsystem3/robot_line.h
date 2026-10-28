#ifndef ROBOT_LINE_H
#define ROBOT_LINE_H
#include "../app.h"
typedef enum { ROBOT_STARTUP=0, ROBOT_INITIAL_SEARCH, ROBOT_FOLLOW, ROBOT_RECOVERY, ROBOT_STOPPED } robot_line_state_t;
ER robot_line_init(void);
ER robot_line_start(void);
int robot_line_is_faulted(void);
robot_line_state_t robot_line_state(void);
#endif

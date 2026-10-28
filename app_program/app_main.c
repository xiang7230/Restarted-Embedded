
/*
 * app_main.c
 *
 * CSC14 Embedded Systems Project
 * Raspberry Pi Pico W + micro T-Kernel 3.0
 *
 * Initial integration:
 *   Buddy 2 - Motion Control
 *   Buddy 3 - IR Line Following
 *
 * Startup:
 *   Power ON
 *      -> micro T-Kernel boots
 *      -> Initialize motors and IR sensors
 *      -> Start motion control task
 *      -> Start line following task
 *
 * The line-following task automatically chooses:
 *
 *   Scenario A:
 *     Black line detected -> Follow line
 *
 *   Scenario B:
 *     No black line -> Initial line search
 *
 * If the line is lost during movement:
 *     Enter line recovery
 *     Search for line
 *     Resume following when found
 *
 * IMPORTANT:
 *   Motor output remains controlled by ROBOT_MOTOR_ENABLE
 *   in subsystem2/robot_motion.h.
 */

#include "app.h"

/* Buddy 2 - Motion Control */
#include "subsystem2/robot_motion.h"

/* Buddy 3 - Line Following */
#include "subsystem3/robot_line.h"

/*
 * Create and start a micro T-Kernel task.
 *
 * prc:
 *   TP_PRC1 = core 0
 *   TP_PRC2 = core 1
 *   0       = any available core
 */
EXPORT ID app_start_task(
    FP entry,
    PRI pri,
    UW prc,
    const char *name
)
{
    T_CTSK ctsk = {0};
    ID tid;
    ER er;

    ctsk.task = entry;
    ctsk.itskpri = pri;
    ctsk.stksz = TASK_STACK_SIZE;

    ctsk.tskatr = TA_HLNG | TA_RNG3;

#if TK_SUPPORT_SMP
    if (prc != 0)
    {
        ctsk.tskatr |= TA_ASSPRC;
        ctsk.assprc = prc;
    }
#else
    (void)prc;
#endif

    /* Create the task */
    tid = tk_cre_tsk(&ctsk);

    if (tid < E_OK)
    {
        CHECK(tid, name);
        return tid;
    }

    /* Start the task */
    er = tk_sta_tsk(tid, 0);

    if (er < E_OK)
    {
        CHECK(er, name);
        return er;
    }

    return tid;
}

/*
 * Main application entry point.
 *
 * Called automatically by micro T-Kernel
 * after every normal firmware boot.
 */
EXPORT INT usermain(void)
{
    T_RVER ver;
    ER er;

    /* =========================================
     * STEP 1: Kernel Information
     * ========================================= */

    tk_ref_ver(&ver);

    tm_printf(
        (UB*)"\n==================================\n"
    );

    tm_printf(
        (UB*)"CSC14 ROBOT FIRMWARE\n"
    );

    tm_printf(
        (UB*)"Raspberry Pi Pico W\n"
    );

    tm_printf(
        (UB*)"micro T-Kernel 3.0\n"
    );

    tm_printf(
        (UB*)"==================================\n"
    );

#if TK_SUPPORT_SMP

    tm_printf(
        (UB*)"RTOS: Dual-core SMP enabled\n"
    );

#else

    tm_printf(
        (UB*)"RTOS: Single-core\n"
    );

#endif

    tm_printf(
        (UB*)"Kernel spec: 0x%04x\n",
        ver.spver
    );

    /* =========================================
     * STEP 2: Initialize Buddy 2
     * ========================================= */

    tm_printf(
        (UB*)"\n[BOOT] Initializing Buddy 2...\n"
    );

    er = robot_motion_init();

    if (er < E_OK)
    {
        tm_printf(
            (UB*)"[ERROR] Buddy 2 initialization failed: %d\n",
            er
        );

        goto boot_failed;
    }

    tm_printf(
        (UB*)"[BOOT] Buddy 2 initialized\n"
    );

    /* =========================================
     * STEP 3: Initialize Buddy 3
     * ========================================= */

    tm_printf(
        (UB*)"\n[BOOT] Initializing Buddy 3...\n"
    );

    er = robot_line_init();

    if (er < E_OK)
    {
        tm_printf(
            (UB*)"[ERROR] Buddy 3 initialization failed: %d\n",
            er
        );

        goto boot_failed;
    }

    tm_printf(
        (UB*)"[BOOT] Buddy 3 initialized\n"
    );

    /* =========================================
     * STEP 4: Start Buddy 2 Task
     * ========================================= */

    tm_printf(
        (UB*)"\n[BOOT] Starting Motion Control...\n"
    );

    er = robot_motion_start();

    if (er < E_OK)
    {
        tm_printf(
            (UB*)"[ERROR] Motion task start failed: %d\n",
            er
        );

        goto boot_failed;
    }

    tm_printf(
        (UB*)"[BOOT] Motion Control running\n"
    );

    /* =========================================
     * STEP 5: Start Buddy 3 Task
     * ========================================= */

    tm_printf(
        (UB*)"\n[BOOT] Starting Line Following...\n"
    );

    er = robot_line_start();

    if (er < E_OK)
    {
        tm_printf(
            (UB*)"[ERROR] Line task start failed: %d\n",
            er
        );

        goto boot_failed;
    }

    tm_printf(
        (UB*)"[BOOT] Line Following running\n"
    );

    /* =========================================
     * STEP 6: Startup Complete
     * ========================================= */

    tm_printf(
        (UB*)"\n==================================\n"
    );

    tm_printf(
        (UB*)"[BOOT] ROBOT SYSTEM READY\n"
    );

    tm_printf(
        (UB*)"Scenario A: Line following\n"
    );

    tm_printf(
        (UB*)"Scenario B: Initial line search\n"
    );

    tm_printf(
        (UB*)"Line recovery: Enabled\n"
    );

    tm_printf(
        (UB*)"Safety timeout: Enabled\n"
    );

    tm_printf(
        (UB*)"==================================\n\n"
    );

    /*
     * Motion and line following now run
     * in their own micro T-Kernel tasks.
     *
     * Sleep this startup task forever.
     */
    tk_slp_tsk(TMO_FEVR);

    return 0;

    /* =========================================
     * BOOT FAILURE
     * ========================================= */

boot_failed:

    /*
     * Latch a stop request if initialization
     * or task creation fails.
     */
    robot_motion_fault();

    tm_printf(
        (UB*)"\n[SAFE STOP] Robot startup failed.\n"
    );

    tm_printf(
        (UB*)"Motor movement is not permitted.\n"
    );

    tk_slp_tsk(TMO_FEVR);

    return 0;
}

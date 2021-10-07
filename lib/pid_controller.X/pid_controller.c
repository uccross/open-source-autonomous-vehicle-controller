/*
 * File:   pid_controller.c
 * Author: Pavlo Vlastos
 * Brief:  A generic pid_controller module. This is used things like for linear 
 * trajectory-tracking an autonomous system, rudder command, and potentially
 * more
 * Created on March 12, 2021, 9:58 PM
 */

/*******************************************************************************
 * #INCLUDES
 ******************************************************************************/
#include "pid_controller.h"
#include "board.h"

/*******************************************************************************
 * PRIVATE #DEFINES
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE DATATYPES 
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE VARIABLES 
 ******************************************************************************/

/*******************************************************************************
 * Public Function Implementations
 ******************************************************************************/
int pid_controller_init(pid_controller_t *cntrl, float dt, float kp, float ki,
        float kd, float max_act_lim, float min_act_lim) {
    cntrl->dt = dt;
    cntrl->kp = kp;
    cntrl->ki = ki;
    cntrl->kd = kd;
    cntrl->max_act_lim = max_act_lim;
    cntrl->min_act_lim = min_act_lim;
    cntrl->accumulator = 0.0;
    cntrl->prev_error = 0.0;
    cntrl->prev_u = 0.0;

    return SUCCESS;
}

float pid_controller_update(pid_controller_t *cntrl, float commanded,
        float measured, float derivative) {
    float error = 0.0;
    float u = 0.0;

    error = commanded - measured;

    /* Trapezoidal integration */
    cntrl->accumulator += (0.5 * cntrl->dt * (error + cntrl->prev_error));

    u = (cntrl->kp * error) + (cntrl->ki * cntrl->accumulator)
            + (cntrl->kd * derivative);

    /* Account for physical actuator limits */
    if (u > cntrl->max_act_lim) {
        cntrl->max_act_lim -= (0.5 * cntrl->dt * (error + cntrl->prev_error));
        u = cntrl->max_act_lim;
    }

    if (u < cntrl->min_act_lim) {
        cntrl->max_act_lim -= (0.5 * cntrl->dt * (error + cntrl->prev_error));
        u = cntrl->min_act_lim;
    }

    cntrl->prev_error = error;
    cntrl->prev_u = u;

    return u;
}

#ifdef UNIT_TESTING
#include "serial.h"
int main () {
    board_init();
    serial_init();
    pid_controller_t a; 
    
    pid_controller_init(&a, 0.1, 10.0, 0.001, 1.0, 0.8, -0.8);
    
    pid_controller_update(&a, 5.0, 0.0, 0.1);
    
    printf("resulting control effort = %f\r\n", a.prev_u);
    
    while (1);
    return 1;
}
#endif
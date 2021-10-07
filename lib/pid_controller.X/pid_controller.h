/*
 * File:   pid_controller.h
 * Author: Pavlo Vlastos
 * Brief:  A linear trajectory-tracking controller for an autonomous system
 * Created on March 12, 2021, 9:58 PM
 */

#ifndef LIN_TRACK_CON_H
#define	LIN_TRACK_CON_H


/*******************************************************************************
 * #INCLUDES
 ******************************************************************************/
#include "board.h"

/*******************************************************************************
 * #DEFINES
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC DATATYPES 
 ******************************************************************************/
typedef struct pid_controller {
    float dt;
    float kp;
    float ki;
    float kd;
    float max_act_lim;
    float min_act_lim;
    float prev_error;
    float accumulator;
    float prev_u;
} pid_controller_t; /* Please don't change this struct unless you know optimal
                     * control. Please!*/

/*******************************************************************************
 * PUBLIC VARIABLES 
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @function pid_controller_init(pid_controller_t *cntrl, float dt, float kp, 
 *      float ki, float kd, float max_act_lim, float min_act_lim)
 * @brief Initializes the controllers sample time, gains, and actuator limits
 * @param cntrl A pointer to the pid_controller_t struct to be initialized
 * @param dt The sample time
 * @param kp The initial proportional gain
 * @param ki The initial integral gain
 * @param kd The initial derivative gain
 * @param max_act_lim The maximum actuator limit. This could be a motor, servo,
 * or something similar
 * @param min_act_lim The minimum actuator limit.
 * @return SUCCESS or ERROR
 */
int pid_controller_init(pid_controller_t *cntrl, float dt, float kp, float ki,
        float kd, float max_act_lim, float min_act_lim);
/**
 * @function pid_controller_update(pid_controller_t *cntrl, float commanded, 
 *      float measured, float derivative)
 * @brief Initializes the controllers sample time, gains, and actuator limits
 * @param cntrl A pointer to the pid_controller_t struct to be updated
 * @param commanded The commanded value for control
 * @param measured The measured value of the system
 * @param deriavtive The measured rate-of-change of the system (if applicable)
 * @return u The resulting control effort/input to the system.
 */
float pid_controller_update(pid_controller_t *cntrl, float commanded, 
        float measured, float derivative);

#endif	/* LIN_TRACK_CON_H */
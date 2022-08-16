/* 
 * File:   PID.c
 * Author: Aaron Hunter
 * Brief: PID controller module
 * Created on 8/15/2022 3:40 pm
 * Modified 
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/

#include "PID.h" // The header file for this source file. 
#include <stdio.h>

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/


/*******************************************************************************
 * PRIVATE TYPEDEFS                                                            *
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 *
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/**
 * @Function PID_init(*pid,float kp, float ki, float kd,
        float max_output, float min_output);
 * @param *pid, pointer to PID_controller type
 * @param kp, proportional gain constant
 * @param, ki, integral gain constant
 * @param, kd, derivative gain constant
 * @param max_output, actuator upper bound
 * @param min_output, actuator lower bound
 * @brief initializes the PID_controller struct
 * @note 
 * @author Aaron Huter,
 * @modified */
void PID_init(PID_controller *pid, float dt, float kp, float ki, float kd,
        float max_output, float min_output) {
    int i;
    pid->dt = dt;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->u_max = max_output;
    pid->u_min = min_output;
    /*pre compute the constant calculations*/
    pid->c0 = kp + ki * dt + kd / dt;
    pid->c1 = -kp - 2 * kd / dt;
    pid->c2 = kd / dt;
    /* initialize variables*/
    pid->u = 0;
    for (i = 0; i < 3; i++) {
        pid->error[i] = 0;
    }
}

/**
 * @Function PID_update(PID_controller *pid, float reference, float measurement)
 * @param *pid, pointer to PID_controller type
 * @param, reference, the current process setpoint
 * @param measurmeent, the current process measurement
 * @brief implements a standard parallel PID
 * @note derivative filtering is not implemented
 * @author Aaron Hunter,
 * @modified  */
void PID_update(PID_controller *pid, float reference, float measurement) {
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = reference - measurement;
    /* compute new output */
    pid->u = pid->u + pid->c0 * pid->error[0] + pid->c1 * pid->error[1] + pid->c2 * pid->error[2];
    /* clamp outputs within actuator limits*/
    if (pid->u > pid->u_max) {
        pid->u = pid->u_max;
    }
    if (pid->u < pid->u_min) {
        pid->u = pid->u_min;
    }
}

/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/


#ifdef PID_TESTING
#include "Board.h"
#include "SerialM32.h"

void main(void) {
    float dt = .02;
    float kp = 1;
    float ki = 0.5;
    float kd = 0.2;
    float ref;
    float y;
    float max_u = 2000;
    float min_u = -2000;
    PID_controller controller;


    Board_init();
    Serial_init();
    printf("PID test harness %s, %s\r\n", __DATE__, __TIME__);
    PID_init(&controller, dt, kp, ki, kd, max_u, min_u);
    printf("Controller initialized to: \r\n");
    printf("dt %f\r\n", controller.dt);
    printf("kp: %f\r\n", controller.kp);
    printf("ki: %f\r\n", controller.ki);
    printf("kd: %f\r\n", controller.kd);
    printf("max output: %f\r\n", controller.u_max);
    printf("min output: %f\r\n", controller.u_min);




}
#endif //PID_TESTING
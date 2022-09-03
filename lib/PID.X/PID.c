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
 * @Function PID_init(*pid);
 * @param *pid, pointer to PID_controller type
 * @brief initializes the PID_controller struct
 * @note computes the c0, c1, c2 constants of the controller and initializes
 * error array
 * @author Aaron Huter,
 * @modified */
void PID_init(PID_controller *pid) {
    int i;
    /*pre compute the constant calculations*/
    pid->c0 = pid->kp + pid->ki * pid->dt + pid->kd / pid->dt;
    pid->c1 = -pid->kp - 2 * pid->kd / pid->dt;
    pid->c2 = pid->kd / pid->dt;
    /* initialize error*/
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
    PID_controller controller;
    controller.dt = .02;
    controller.kp = 1;
    controller.ki = 0.5;
    controller.kd = 0.2;
    controller.u_max = 2000;
    controller.u_min = -2000;

    float ref;
    float y;



    Board_init();
    Serial_init();
    printf("PID test harness %s, %s\r\n", __DATE__, __TIME__);
    PID_init(&controller);
    printf("Controller initialized to: \r\n");
    printf("dt %f\r\n", controller.dt);
    printf("kp: %f\r\n", controller.kp);
    printf("ki: %f\r\n", controller.ki);
    printf("kd: %f\r\n", controller.kd);
    printf("max output: %f\r\n", controller.u_max);
    printf("min output: %f\r\n", controller.u_min);




}
#endif //PID_TESTING
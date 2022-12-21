/* 
 * File:   PID.h
 * Author: Aaron Hunter
 * Brief: Interface to PID controller module
 * Created on 8/15/2022 3:40 pm
 * Modified 
 */

#ifndef PID_H // Header guard
#define	PID_H //

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/
#include <stdint.h>
/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC TYPEDEFS                                                             *
 ******************************************************************************/
typedef struct PID_controller {
    float dt; //loop update time (sec)
    float kp; // proportional gain
    float ki; // integral gain
    float kd; // derivative gain
    float u_max; // output upper bound
    float u_min; //output lower bound
    float u_calc; // calculated output
    float u; // output returned (may be different due to actuator limits)
    float c0; // pre-computed constants
    float c1; 
    float c2; 
    float error[3]; // explicit array of last three error values
} PID_controller;

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

/**
 * @Function PID_init(*pid);
 * @param *pid, pointer to PID_controller type
 * @brief initializes the PID_controller struct
 * @note computes the c0, c1, c2 constants of the controller and initializes
 * error array
 * @author Aaron Huter,
 * @modified */
void PID_init(PID_controller *pid);



/**
 * @Function PID_update(PID_controller *pid, float reference, float measurement)
 * @param *pid, pointer to PID_controller type
 * @param, reference, the current process setpoint
 * @param measurmeent, the current process measurement
 * @brief implements a standard parallel PID
 * @note derivative filtering is not implemented
 * @author Aaron Hunter,
 * @modified  */
void PID_update(PID_controller *pid, float reference, float measurement);



#endif	/* PID_H */ // End of header guard


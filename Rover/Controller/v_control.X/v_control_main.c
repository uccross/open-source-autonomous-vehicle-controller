/*
 * File:   v_control_main.c
 * Author: Aaron
 *
 * Created on October 10, 2022, 2:00 PM
 */
/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "xc.h"
#include "AS5047D.h"
#include "Board.h"
#include "PID.h"
#include "RC_servo.h"
#include "SerialM32.h"
#include "System_timer.h"

/*******************************************************************************
 * #DEFINES                                                                    *
 ******************************************************************************/
#define HEARTBEAT_PERIOD 1000 //1 sec interval for hearbeat update
#define CONTROL_PERIOD 20 //Period for control loop in msec
#define UINT_16_MAX 0xffff
#define BUFFER_SIZE 1024
#define RAW 1
#define SCALED 2
#define NUM_MOTORS 3
#define DT 0.02 //integration constant
#define M_PI 3.14159265358979
#define DELAY_2SEC 2000

/*******************************************************************************
 * GLOBAL CONVERSIONS  AND VARS                                                *
 ******************************************************************************/
/* Convert 'omega' variable to radians/sec*/
const float ticks_to_w = 2 * M_PI / (16384.0 * DT);

/*******************************************************************************
 * TYPEDEFS                                                                    *
 ******************************************************************************/
enum motors {
    MOTOR_LEFT,
    MOTOR_RIGHT,
    SERVO
};

/*******************************************************************************
 * FUNCTION PROTOTYPES                                                         *
 ******************************************************************************/
/**
 * @function get_v(encoder_t enc[])
 * @param enc[] array of encoder data containers
 * @brief converts encoder omega into vehicle speed
 * @returns velocity in cm/s
 * @author Aaron Hunter
 */
float get_v(encoder_t enc[]);

/**
 * @function low_pass(x)
 * @param x, current measurement
 * @return filtered quantity
 */
float low_pass(float x);

/*******************************************************************************
 * FUNCTIONS                                                                   *
 ******************************************************************************/

/**
 * @function get_v(encoder_t enc[])
 * @param enc[] array of encoder data containers
 * @brief converts encoder omega into vehicle speed
 * @returns velocity in cm/s
 * @author Aaron Hunter
 */
float get_v(encoder_t enc[]) {
    float r = 3.25; // wheel radius, cm
    float omega; // angular velocity rad/sec
    float v; //velocity in cm/s

    omega = (float) ((enc[0].omega + enc[1].omega) >> 1) * ticks_to_w; //average angular velocity
    v = r*omega; // cm/sec
    return v;
}

/**
 * @function low_pass(x)
 * @param x, current measurement
 * @return filtered quantity
 */
float low_pass(float x) {
    static float y_prev = 0;
    float y_new;
    float alpha = 0.05;
    y_new = y_prev + alpha * (x - y_prev);
    y_prev = y_new;
    return (y_new);
}

int main(void) {
    uint32_t start_time = 0;
    uint32_t cur_time = 0;
    uint32_t control_start_time = 0;
    encoder_t encoder_data[NUM_MOTORS]; //array of encoder structs
    PID_controller v_PID = {
        .dt = DT,
        .kp = 0.8,
        .ki = 0.2,
        .kd = 0.0,
        .u_max = 500,
        .u_min = -500
    };
    float v_ref = 100.0; // cm/s
    float v_meas;
    float scale = 1.0;
    uint16_t pwm_val = 0;
    float u_cmd;

    //Initialization routines
    Board_init(); //board configuration
    Serial_init(); //start USB interface output
    Sys_timer_init(); //start the system timer
    PID_init(&v_PID); // initialize the PID control
    Encoder_init();
    RC_servo_init(ESC_BIDIRECTIONAL_TYPE, SERVO_PWM_1);
    RC_servo_init(ESC_BIDIRECTIONAL_TYPE, SERVO_PWM_2);
    RC_servo_set_pulse(1490, MOTOR_LEFT);
    RC_servo_set_pulse(1490, MOTOR_RIGHT);
    cur_time = Sys_timer_get_msec();
    while (Sys_timer_get_msec() - cur_time <= DELAY_2SEC) {
        ;
    }
    cur_time = Sys_timer_get_msec();
    control_start_time = cur_time;
    while (1) {
        cur_time = Sys_timer_get_msec();
        if (cur_time - control_start_time >= CONTROL_PERIOD) {
            control_start_time = cur_time;
            v_meas = get_v(encoder_data);
            v_meas = low_pass(v_meas);
            PID_update(& v_PID, v_ref, v_meas);
            pwm_val = (uint16_t) (v_PID.u * scale) + RC_SERVO_CENTER_PULSE;
            RC_servo_set_pulse(pwm_val, MOTOR_LEFT);
            RC_servo_set_pulse(pwm_val, MOTOR_RIGHT);
            Encoder_start_data_acq(); // start the next reading of the encoders
            printf("%f, %f \r\n", v_meas, v_PID.u);
        }
        if (Encoder_is_data_ready()) {
            Encoder_get_data(encoder_data);
        }
    }
    return 0;
}

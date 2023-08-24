/*
 * File:   gyro_main.c
 * Author: Aaron
 *
 * Created on August 23, 2022, 11:06 AM
 */
/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "AHRS.h"
#include "Board.h"
#include "ICM_20948.h"
#include "PID.h"
#include "RC_servo.h"
#include "SerialM32.h"
#include "System_timer.h"
#include "xc.h"
/*******************************************************************************
 * #DEFINES                                                                    *
 ******************************************************************************/
#define CONTROL_PERIOD 20 //Period for control loop in msec
#define DT 0.02 //integration constant

/*******************************************************************************
 * PROTOTYPES                                                                  *
 ******************************************************************************/
/**
 * @Function set_control_output(float gyros[], PID_controller * controller)
 * @param gyros[], gyro rates in rad/sec
 * @param controller, the PID controller 
 * @return none
 * @brief converts gyro rate to servo outputs
 * (servos and ESCs) to those values
 * @author Aaron Hunter
 */
void set_control_output(float gyros[], PID_controller * controller);

/*******************************************************************************
 * FUNCTIONS                                                                   *
 ******************************************************************************/
void set_control_output(float gyros[], PID_controller * controller) {
    float ref = 0; //
    float meas = gyros[0]; // x axis gyro
    float setpoint = 0;
    /* get control from PID*/
    PID_update(controller, ref, meas);
    setpoint = controller->u + RC_SERVO_CENTER_PULSE;
    RC_servo_set_pulse((int16_t) setpoint, SERVO_PWM_1);
}

int main(void) {
    // IMU 1 calibration values:
    float A_acc[MSZ][MSZ] = {
        {6.12649072e-05, -7.69323670e-07, 7.41523511e-07},
        {8.86960833e-08, 5.99163592e-05, 4.45763158e-07},
        {-8.40898963e-08, -9.22602608e-07, 5.95395979e-05}
    };
    float A_mag[MSZ][MSZ] = {
        {3.36826234e-03, -1.56956478e-05, -1.16520778e-05},
        { 4.48554406e-05, 3.54448710e-03, 1.51280264e-05},
        {-1.02003198e-04, -3.17665038e-05, 3.48151565e-03}
    };
    float b_acc[MSZ] = {0.02178154, 0.01720978, 0.01673077};
    float b_mag[MSZ] = {-0.46909233, -0.03577201, 0.46432114};

    /* data arrays */
    static float gyro_cal[MSZ] = {0, 0, 0};
    static float acc_cal[MSZ] = {0, 0, 0};
    static float mag_cal[MSZ] = {0, 0, 0};

    /* conversions */
    const float deg2rad = M_PI / 180.0;
    const float rad2deg = 180.0 / M_PI;

    /* Euler angles */
    float euler[MSZ] = {0, 0, 0};

    /* IMU return value and data struct*/
    uint8_t IMU_state;
    int8_t IMU_retry = 5;
    struct IMU_out IMU_data = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    /* Controller and gains*/
    PID_controller controller; // declare our controller object
    float dt = DT; // set update interval
    float kp = 40;
    float ki = 0.0;
    float kd = 0.0;
    float u_max = 500.0;
    float u_min = -500.0;

    /* timing constants */
    uint32_t current_time;
    uint32_t start_time;
    uint32_t warmup = 500; // msec warmup for IMU

    /* Initialize all the subsystems and sensors */
    Board_init();
    Serial_init();
    Sys_timer_init();
    RC_servo_init(RC_SERVO_TYPE, SERVO_PWM_1); // Set servo output to channel 1
    /*Allow IMU time to warm up before initializing it*/
    start_time = Sys_timer_get_msec();
    current_time = start_time;
    while (current_time - start_time <= warmup) {
        current_time = Sys_timer_get_msec();
    }
    /* initialize the IMU */
    IMU_state = IMU_init(IMU_SPI_MODE);
    if (IMU_state == ERROR && IMU_retry > 0) {
        IMU_state = IMU_init(IMU_SPI_MODE);
        printf("IMU failed init, retrying %d \r\n", IMU_retry);
        IMU_retry--;
    }
    /* set the calibration values */
    IMU_set_acc_cal(A_acc, b_acc);
    IMU_set_mag_cal(A_mag, b_mag);

    /* initialize the PID controller*/
    PID_init(&controller, dt, kp, ki, kd, u_max, u_min);

    /* Control loop*/
    start_time = Sys_timer_get_msec();
    current_time = start_time;
    while (1) {
        current_time = Sys_timer_get_msec();
        if (IMU_is_data_ready() == TRUE) {
            IMU_get_norm_data(&IMU_data);
            acc_cal[0] = IMU_data.acc.x;
            acc_cal[1] = IMU_data.acc.y;
            acc_cal[2] = IMU_data.acc.z;
            mag_cal[0] = IMU_data.mag.x;
            mag_cal[1] = IMU_data.mag.y;
            mag_cal[2] = IMU_data.mag.z;
            /*scale gyro readings into rad/sec */
            gyro_cal[0] = IMU_data.gyro.x * deg2rad;
            gyro_cal[1] = IMU_data.gyro.y * deg2rad;
            gyro_cal[2] = IMU_data.gyro.z * deg2rad;
            AHRS_update(acc_cal, mag_cal, gyro_cal, dt, euler);
            printf("%+3.1f, %+3.1f, %+3.1f, %+2.1f \r\n ", gyro_cal[0] * rad2deg, gyro_cal[1] * rad2deg, gyro_cal[2] * rad2deg, controller.u);

        }
        if (current_time - start_time >= CONTROL_PERIOD) {
            start_time = current_time; // reset timer
            set_control_output(gyro_cal, &controller); // set actuator outputs
            IMU_start_data_acq(); //initiate next IMU measurement
        }
    }

    return 0;
}

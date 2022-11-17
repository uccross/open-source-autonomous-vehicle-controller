/* 
 * File:   h_ctrl_main.c
 * Author: Aaron
 *
 * Created on November 2, 2022, 11:45 AM
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "xc.h"
#include "AHRS.h"
#include "AS5047D.h"
#include "Board.h"
#include "ICM_20948.h"
#include "Lin_alg_float.h"
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
#define DELAY_2SEC 2000

/*******************************************************************************
 * GLOBAL CONVERSIONS  AND VARS                                                *
 ******************************************************************************/
/* Convert encoder ticks to degrees*/
const float ticks_to_degrees = 360.0 / 16384.0;
const float deg2rad = M_PI / 180.0;
const float rad2deg = 180.0 / M_PI;

struct IMU_out IMU_raw = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //container for raw IMU data
struct IMU_out IMU_scaled = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //container for scaled IMU data

/*AHRS filter gains*/
float kp_a = 2.5; //accelerometer proportional gain
float ki_a = 0.05; // accelerometer integral gain
float kp_m = 2.5; // magnetometer proportional gain
float ki_m = 0.05; //magnetometer integral gain
/*timing and conversion*/
const float dt = DT;


/* Calibration matrices and offset vectors */
/* Rover IMU calibration */
float A_acc[MSZ][MSZ] = {
    6.01180201773358e-05, -6.28352073406424e-07, -3.91326747595870e-07,
    -1.18653342135860e-06, 6.01268083773005e-05, -2.97010157797952e-07,
    -3.19011230800348e-07, -3.62174516629958e-08, 6.04564465269327e-05
};
float A_mag[MSZ][MSZ] = {
    0.00351413733554131, -1.74599042407869e-06, -1.62761272908763e-05,
    6.73767225208446e-06, 0.00334531206332366, -1.35302929502152e-05,
    -3.28233797524166e-05, 9.29337701972177e-06, 0.00343350080131375
};
float b_acc[MSZ] = {-0.0156750747576770, -0.0118720194488050, -0.0240128301624044};
float b_mag[MSZ] = {-0.809679246097106, 0.700742334522691, -0.571694648765172};

// gravity inertial vector
float a_i[MSZ] = {0, 0, 1.0};
// Earth's magnetic field inertial vector, normalized 
// North 22,680.8 nT	East 5,217.6 nT	Down 41,324.7 nT, value from NOAA
// converted into ENU format and normalized:
float m_i[MSZ] = {0.110011998753301, 0.478219898291142, -0.871322609031072};

/*attitude*/
float q_vehicle[QSZ] = {1, 0, 0, 0};
/*gyro bias*/
float gyro_bias[MSZ] = {0, 0, 0};
/*euler angles (yaw, pitch, roll) */
float euler[MSZ] = {0, 0, 0};

/* data arrays */
float gyro_cal[MSZ] = {0, 0, 0};
float acc_cal[MSZ] = {0, 0, 0};
float mag_cal[MSZ] = {0, 0, 0};

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
 * Function get_delta(heading_0, encoder_t enc[])
 * @param heading_0, the encoder value when servo is centered
 * @param end[] the encoder data structs
 * @return steering angle, delta in degrees
 */
float get_delta(uint16_t heading_0, encoder_t enc[]);

/**
 * @function q_rot_v_q()
 * Rotate a vector from the inertial frame to the body frame
 * @param v_i, a 3space vector in the inertial frame
 * @param q an attitude quaternion
 * sets v_b to the rotated inertial vector in the body frame
 */
void q_rot_v_q(float v_i[MSZ], float q[QSZ], float v_b[MSZ]);
/*******************************************************************************
 * FUNCTIONS                                                                   *
 ******************************************************************************/

/**
 * Function get_delta(heading_0, encoder_t enc[])
 * @param heading_0, the encoder value when servo is centered
 * @param end[] the encoder data structs
 * @return steering angle, delta in degrees
 */
float get_delta(uint16_t heading_0, encoder_t enc[]) {
    float delta = (float) (enc[SERVO].next_theta - heading_0);
    delta = delta * ticks_to_degrees; // convert to degrees
    return delta;
}

/**
 * @function q_rot_v_q()
 * Rotate a vector from the inertial frame to the body frame
 * @param v_i, a 3space vector in the inertial frame
 * @param q an attitude quaternion
 * sets v_b to the rotated inertial vector in the body frame
 */
void q_rot_v_q(float v_i[MSZ], float q[QSZ], float v_b[MSZ]) {
    float q_i[QSZ];
    float q_temp[QSZ];
    float q_conj[QSZ];
    float q_b[QSZ]; // container for inertial vector in body frame as pure quaternion

    // calculate conjugate of q 
    q_conj[0] = q[0];
    q_conj[1] = -q[1];
    q_conj[2] = -q[2];
    q_conj[3] = -q[3];

    //   convert v_i to a pure quaternion --> q_i
    q_i[0] = 0;
    q_i[1] = v_i[0];
    q_i[2] = v_i[1];
    q_i[3] = v_i[2];
    //   first quaternion product q_i by q --> q_temp
    lin_alg_q_mult(q_i, q, q_temp);
    //   second quaternion product q_conj by q_temp -->q_b
    lin_alg_q_mult(q_conj, q_temp, q_b);
    //    set v_b to imaginary part of q_b
    v_b[0] = q_b[1];
    v_b[1] = q_b[2];
    v_b[2] = q_b[3];
}

void scale_IMU_values(void) {
    acc_cal[0] = (float) IMU_scaled.acc.x;
    acc_cal[1] = (float) IMU_scaled.acc.y;
    acc_cal[2] = (float) IMU_scaled.acc.z;
    mag_cal[0] = (float) IMU_scaled.mag.x;
    mag_cal[1] = (float) IMU_scaled.mag.y;
    mag_cal[2] = (float) IMU_scaled.mag.z;
    /*scale gyro readings into rad/sec */
    gyro_cal[0] = (float) IMU_scaled.gyro.x * deg2rad;
    gyro_cal[1] = (float) IMU_scaled.gyro.y * deg2rad;
    gyro_cal[2] = (float) IMU_scaled.gyro.z * deg2rad;
}

int main(void) {
    uint32_t cur_time = 0;
    uint32_t control_start_time = 0;
    encoder_t encoder_data[NUM_MOTORS]; //array of encoder structs
    PID_controller heading_PID = {
        .dt = DT,
        .kp = 15.0,
        .ki = 0.0,
        .kd = 0.0,
        .u_max = 500.0,
        .u_min = -500.0
    };

    uint16_t pwm_val = 0;
    float waypoint[3] = {0.0, 100.0, 0.0}; // test vector, basically north
    float position[3] = {0.0, 0.0, 0.0}; // robot position
    float heading_vec_i[MSZ] = {0.0, 0.0, 0.0}; // vector to waypoint
    float heading_vec_b[MSZ];
    uint16_t heading_0;
    float heading_ref = 0.0;
    float heading_meas;
    float u_calc;
    int8_t IMU_state = ERROR;
    int8_t IMU_retry = 5;

    /* Initialization routines */
    Board_init(); //board configuration
    Serial_init(); //start USB interface output
    Sys_timer_init(); //start the system timer
    PID_init(&heading_PID); // initialize the PID controller
    printf("c0; %E, c1: %E, c2: %E\r\n", heading_PID.c0, heading_PID.c1, heading_PID.c2);
    Encoder_init();
    RC_servo_init(RC_SERVO_TYPE, SERVO_PWM_3);
    RC_servo_set_pulse(1500, SERVO);
    /* initialize the IMU */
    IMU_state = IMU_init(IMU_SPI_MODE);
    if (IMU_state == ERROR && IMU_retry > 0) {
        IMU_state = IMU_init(IMU_SPI_MODE);
        printf("IMU failed init, retrying %d \r\n", IMU_retry);
        IMU_retry--;
    }

    /* get zero angle heading*/
    Encoder_start_data_acq();
    while (Encoder_is_data_ready() == FALSE) {
        ; // wait for encoder response
    }
    Encoder_get_data(encoder_data); // get encoder values
    heading_0 = encoder_data[SERVO].next_theta;

    /* load IMU calibrations */
    IMU_set_mag_cal(A_mag, b_mag);
    IMU_set_acc_cal(A_acc, b_acc);

    /* set filter gains and inertial guiding vectors for AHRS*/
    AHRS_set_filter_gains(kp_a, ki_a, kp_m, ki_m);
    AHRS_set_mag_inertial(m_i);

    cur_time = Sys_timer_get_msec();
    control_start_time = cur_time;
    while (1) {
        cur_time = Sys_timer_get_msec();
        if (cur_time - control_start_time >= CONTROL_PERIOD) {
            control_start_time = cur_time;
            /* update attitude estimate*/
            AHRS_update(acc_cal, mag_cal, gyro_cal, dt, q_vehicle, gyro_bias);
            /* compute vector to waypoint */
            lin_alg_v_v_sub(waypoint, position, heading_vec_i);
            //            printf("Heading vector %3.1f, %3.1f, %3.1f \r\n ", heading_vec_i[0],heading_vec_i[1], heading_vec_i[2]);
            /* rotate vector into body frame */
            q_rot_v_q(heading_vec_i, q_vehicle, heading_vec_b);
            /* compute angle to waypoint */
            heading_meas = atan2f(heading_vec_b[1], heading_vec_b[0]);
            heading_meas = heading_meas * rad2deg; // convert to degrees
            //            printf("Heading vector %3.1f, %3.1f, %3.1f, angle: %3.1f \r\n ",
            //                    heading_vec_b[0], heading_vec_b[1], heading_vec_b[2], heading_meas);
            /* compute control action */
            PID_update(&heading_PID, heading_ref, heading_meas);
            pwm_val = (uint16_t) (heading_PID.u) + RC_SERVO_CENTER_PULSE;
            /* apply control action */
            RC_servo_set_pulse(pwm_val, SERVO);
            /* initiate measurements for next period */
            Encoder_start_data_acq(); // start the next reading of the encoders
            IMU_state = IMU_start_data_acq(); //initiate IMU measurement with SPI

            //            printf("Attitude: [%1.3f, %1.3f, %1.3f, %1.3f], ", q_vehicle[0], q_vehicle[1], q_vehicle[2], q_vehicle[3]);
        }
        if (IMU_is_data_ready() == TRUE) {
            IMU_get_norm_data(&IMU_scaled); // get calibrated IMU values
            scale_IMU_values(); // set mag, acc, gyros to scaled IMU values
        }
        if (Encoder_is_data_ready()) {
            Encoder_get_data(encoder_data);
        }
    }
    return 0;
}


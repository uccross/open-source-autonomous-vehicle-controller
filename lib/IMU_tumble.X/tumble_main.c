/* 
 * File:   tumble_main.c
 * Author: Aaron Hunter
 * Brief: 
 * Created on July 20, 2022 2:38 pm
 * Modified on <month> <day>, <year>, <hour> <pm/am>
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/
#include <stdio.h>
#include <stdfix.h>
#include "ICM_20948.h"  
#include "SerialM32.h"
#include "Board.h"
#include "System_timer.h"
// #include <sys/attribs.h>  //for ISR definitions
#include <proc/p32mx795f512l.h>
#include "xc.h"

/*******************************************************************************
 * #DEFINES                                                                    *
 ******************************************************************************/
//#define RAW_DATA_OUT
#define NORM_DATA_OUT
#define MSZ 3

int main(void) {
    uint32_t start_time = 0;
    uint32_t period_start = 0;
    uint32_t current_time = 0;
    uint32_t warmup_time = 250; // IMU settling time
    uint32_t period = 20; // milliseconds
    uint32_t data_collection_time = 30000; //milliseconds
    int8_t timer_expired = FALSE;
    struct IMU_out IMU_data_out = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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


    Board_init();
    Serial_init();
    Sys_timer_init();
    printf("IMU tumble test data %s, %s\r\n", __DATE__, __TIME__);
    /* Give IMU time to stabilize*/
    start_time = Sys_timer_get_msec();
    while (!timer_expired) {
        current_time = Sys_timer_get_msec();
        if ((current_time - start_time) >= warmup_time) timer_expired = TRUE;
    }
    IMU_init(IMU_SPI_MODE);
#ifdef NORM_DATA_OUT
    IMU_set_mag_cal(A_mag, b_mag);
    IMU_set_acc_cal(A_acc, b_acc);
#endif
    timer_expired = FALSE;
    start_time = Sys_timer_get_msec();
    period_start = start_time;
    while (!timer_expired) {
        current_time = Sys_timer_get_msec();
        if ((current_time - start_time) >= data_collection_time) timer_expired = TRUE;
        if (current_time - period_start >= period) {
            IMU_start_data_acq();
            period_start = current_time;
        }
        if (IMU_is_data_ready()) {
#ifdef RAW_DATA_OUT
            IMU_get_raw_data(&IMU_data_out);
            printf("%0.0f, %0.0f, %0.0f, %0.0f, %0.0f, %0.0f\r\n",
                    IMU_data_out.acc.x, IMU_data_out.acc.y, IMU_data_out.acc.z,
                    IMU_data_out.mag.x, IMU_data_out.mag.y, IMU_data_out.mag.z);
#endif
#ifdef NORM_DATA_OUT
            IMU_get_norm_data(&IMU_data_out);
            printf("%f, %f, %f, "
                    "%f, %f, %0f, "
                    "%0.0f, %0.0f, %0.0f\r\n",
                    IMU_data_out.acc.x, IMU_data_out.acc.y, IMU_data_out.acc.z,
                    IMU_data_out.mag.x, IMU_data_out.mag.y, IMU_data_out.mag.z,
                    IMU_data_out.gyro.x, IMU_data_out.gyro.y, IMU_data_out.gyro.z);
#endif
        }

    }
    return 0;
}

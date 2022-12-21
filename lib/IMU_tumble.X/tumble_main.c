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
#define RAW_DATA_OUT
//#define NORM_DATA_OUT
#define MSZ 3

int main(void) {
    uint32_t start_time = 0;
    uint32_t period_start = 0;
    uint32_t current_time = 0;
    uint32_t warmup_time = 250; // IMU settling time
    uint32_t period = 20; // milliseconds
    uint32_t data_collection_time = 60000; //milliseconds
    int8_t timer_expired = FALSE;
    struct IMU_out IMU_data_out = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    float A_acc[MSZ][MSZ] = {
        {5.98605657636023e-05,	5.02299172664344e-08,	8.41134559461075e-07},
        {-2.82167981801537e-08,	6.05938345982234e-05,	6.95665927111956e-07},
        {4.48326742757725e-08,	-3.34771681800715e-07,	5.94633160681115e-05}
    };
    float A_mag[MSZ][MSZ] = {
        {0.00333834334834959,	2.58649731866218e-05,	-4.47182534891735e-05},
        {3.97521279910819e-05,	0.00341838979684877,	-7.55578863505947e-06},
        {-6.49436573527762e-05,	3.05050635014235e-05,	0.00334143925188739}
    };
    float b_acc[MSZ] = {0.00591423067694908, 0.0173747801090554, 0.0379428158730668};
    float b_mag[MSZ] = {0.214140746707571, -1.08116057610690, -0.727337561140470};


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

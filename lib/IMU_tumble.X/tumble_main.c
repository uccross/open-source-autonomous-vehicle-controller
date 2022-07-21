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

int main(void) {
    uint32_t start_time = 0;
    uint32_t period_start = 0;
    uint32_t current_time = 0;
    uint32_t warmup_time = 250; // IMU settling time
    uint32_t period = 20; // milliseconds
    uint32_t data_collection_time = 30000; //milliseconds
    int8_t timer_expired = FALSE;
    struct IMU_out IMU_data_raw = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


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
            IMU_get_raw_data(&IMU_data_raw);
            printf("%0.0f, %0.0f, %0.0f, %0.0f, %0.0f, %0.0f\r\n",
                    IMU_data_raw.acc.x, IMU_data_raw.acc.y, IMU_data_raw.acc.z,
                    IMU_data_raw.mag.x, IMU_data_raw.mag.y, IMU_data_raw.mag.z);
        }
    }
    return 0;
}

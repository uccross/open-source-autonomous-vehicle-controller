/*
 * File:   newmain.c
 * Author: Aaron Hunter
 * Brief:  main file for the radio-control passthrough mode for the OSAVC
 * project
 * Created on January 13, 2021, 9:41 AM
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "xc.h"
#include "Board.h"
#include "SerialM32.h"
#include "System_timer.h"
#include "Radio_serial.h"
#include "common/mavlink.h"
//#include "mavlink_helpers.h"



/*******************************************************************************
 * #DEFINES                                                                    *
 ******************************************************************************/
#define HEARTBEAT_PERIOD 1000 //1 sec interval for hearbeat update
#define BUFFER_SIZE 1024
/*******************************************************************************
 * VARIABLES                                                                   *
 ******************************************************************************/
mavlink_system_t mavlink_system = {
    1, // System ID (1-255)
    MAV_COMP_ID_AUTOPILOT1  // Component ID (a MAV_COMPONENT value)
};

/*******************************************************************************
 * TYPEDEFS                                                                    *
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES                                                         *
 ******************************************************************************/


/*******************************************************************************
 * FUNCTIONS                                                                   *
 ******************************************************************************/



int main(void) {
    uint16_t index = 0;
    uint32_t cur_time =0;
    uint32_t start_time = 0;
    mavlink_message_t msg_tx;
    mavlink_message_t msg_rx;
    mavlink_status_t msg_rx_status;
    mavlink_heartbeat_t heartbeat;
    
    uint8_t channel = MAVLINK_COMM_0;
    uint8_t msg_byte;
    uint16_t msg_length;
    uint8_t msg_buffer[BUFFER_SIZE];
    
    uint8_t mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    uint32_t custom = 0;
    uint8_t state = MAV_STATE_STANDBY;
    

    Board_init(); //board configuration
    Serial_init(); //start debug terminal (USB)
    Radio_serial_init(); //start the radios
    Sys_timer_init(); //start the system timer
    printf("\r\nMinimal Mavlink application %s, %s \r\n", __DATE__, __TIME__);
    while (1) {
        cur_time = Sys_timer_get_msec();
        if(Radio_data_available()){
            msg_byte = Radio_get_char();
            if(mavlink_parse_char(channel, msg_byte, &msg_rx, &msg_rx_status)){
                 printf("Received message with ID %d, sequence: %d from component %d of system %d\r\n", msg_rx.msgid, msg_rx.seq, msg_rx.compid, msg_rx.sysid);
                 switch(msg_rx.msgid) {
                     case MAVLINK_MSG_ID_HEARTBEAT:
                         mavlink_msg_heartbeat_decode(&msg_rx, &heartbeat);
                         if(heartbeat.type)
                         printf("heartbeat from: %d\r\n", heartbeat.type);
                         break;
                     default:
                         break;
                 }
                     
            }
        }
        if(cur_time-start_time >= HEARTBEAT_PERIOD){
            mavlink_msg_heartbeat_pack(mavlink_system.sysid
                    ,mavlink_system.compid,
                    &msg_tx, 
                    MAV_TYPE_GROUND_ROVER,MAV_AUTOPILOT_GENERIC,
                    mode,
                    custom,
                    state);
            msg_length = mavlink_msg_to_send_buffer(msg_buffer, &msg_tx);
            for(index = 0; index < msg_length; index++){
                Radio_put_char(msg_buffer[index]);
            }
            printf("heartbeat sent\r\n");
            start_time = cur_time; //reset the timer
        }
    }
    return 0;
}
/*
 * File:   main.c
 * Author: Aaron Hunter
 * Brief:  Minimal application to test MAVLink communication with QGC 
 * project
 * Created on January 27, 2021 at 2:00 pm
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
#include "NEO_M8N.h"



/*******************************************************************************
 * #DEFINES                                                                    *
 ******************************************************************************/
#define HEARTBEAT_PERIOD 1000 //1 sec interval for hearbeat update
#define BUFFER_SIZE 1024
#define UINT_16_MAX 0xffff
#define RAD2DEG 180.0/M_PI
#define DEG2RAD M_PI/180.0

/*******************************************************************************
 * VARIABLES                                                                   *
 ******************************************************************************/
mavlink_system_t mavlink_system = {
    1, // System ID (1-255)
    MAV_COMP_ID_AUTOPILOT1 // Component ID (a MAV_COMPONENT value)
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
    uint32_t cur_time = 0;
    uint32_t start_time = 0;
    mavlink_message_t msg_tx;
    mavlink_message_t msg_rx;
    mavlink_status_t msg_rx_status;
    mavlink_heartbeat_t heartbeat;
    mavlink_command_long_t command_qgc;

    uint8_t channel = MAVLINK_COMM_0;
    uint8_t msg_byte;
    uint16_t msg_length;
    uint8_t msg_buffer[BUFFER_SIZE];

    uint8_t mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    uint32_t custom = 0;
    uint8_t state = MAV_STATE_STANDBY;
    //GPS variable
    struct GPS_data GPS_data;
    uint8_t gps_fix = GPS_FIX_TYPE_NO_FIX;


    Board_init(); //board configuration
    Serial_init(); //start debug terminal (USB)
    Radio_serial_init(); //start the radios
    GPS_init();
    Sys_timer_init(); //start the system timer
    printf("\r\nMinimal Mavlink application %s, %s \r\n", __DATE__, __TIME__);
    while (1) {
        cur_time = Sys_timer_get_msec();
        //radio
        if (Radio_data_available()) {
            msg_byte = Radio_get_char();
            if (mavlink_parse_char(channel, msg_byte, &msg_rx, &msg_rx_status)) {
                printf("Received message with ID %d, sequence: %d from component %d of system %d\r\n", msg_rx.msgid, msg_rx.seq, msg_rx.compid, msg_rx.sysid);
                switch (msg_rx.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT:
                        mavlink_msg_heartbeat_decode(&msg_rx, &heartbeat);
                        if (heartbeat.type)
                            printf("heartbeat type: %d\r\n", heartbeat.type);
                        break;
                    case MAVLINK_MSG_ID_COMMAND_LONG:
                        mavlink_msg_command_long_decode(&msg_rx, &command_qgc);
                        printf("Command ID %d received from QGC\r\n", command_qgc.command);
                        break;

                    default:
                        break;
                }

            }
        }

        //GPS
        if (GPS_is_msg_avail() == TRUE) {
            GPS_parse_stream();
        }

        if (GPS_is_data_avail() == TRUE) {
            GPS_get_data(&GPS_data);
            if (GPS_has_fix() == TRUE) {
                gps_fix = GPS_FIX_TYPE_3D_FIX;
            }
            mavlink_msg_gps_raw_int_pack(mavlink_system.sysid,
                    mavlink_system.compid,
                    &msg_tx,
                    (uint64_t) Sys_timer_get_usec(),
                    gps_fix,
                    (int32_t) (GPS_data.lat * RAD2DEG * 10000000.0),
                    (int32_t) (GPS_data.lon * RAD2DEG * 10000000.0),
                    0, //altitude --can update GPS data if need this
                    UINT_16_MAX, //hdop--currently don't care
                    UINT_16_MAX, //vdop
                    (uint16_t) (GPS_data.spd * 100.0), //need to verify units and convert from knots if so
                    (uint16_t) (GPS_data.cog * 100.0),
                    255, //satellites visible again, currently don't care
                    0, //alt ellispoid
                    0, //h position uncertainty
                    0, //v position uncertainty
                    0, //velocity uncertainty
                    0, //heading uncertainty
                    0 // yaw--GPS doesn't provide
                    );
            msg_length = mavlink_msg_to_send_buffer(msg_buffer, &msg_tx);
            for (index = 0; index < msg_length; index++) {
                Radio_put_char(msg_buffer[index]);
            }
        }
        //heartbeat update
        if (cur_time - start_time >= HEARTBEAT_PERIOD) {
            mavlink_msg_heartbeat_pack(mavlink_system.sysid
                    , mavlink_system.compid,
                    &msg_tx,
                    MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
                    mode,
                    custom,
                    state);
            msg_length = mavlink_msg_to_send_buffer(msg_buffer, &msg_tx);
            for (index = 0; index < msg_length; index++) {
                Radio_put_char(msg_buffer[index]);
            }
            printf("heartbeat sent\r\n");
            start_time = cur_time; //reset the timer
        }
    }
    return 0;
}
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
#include <math.h>
#include "xc.h"
#include "Board.h"
#include "SerialM32.h"
#include "System_timer.h"
#include "Radio_serial.h"
#include "common/mavlink.h"
#include "NEO_M8N.h"
#include "RC_RX.h"
#include "RC_servo.h"
#include "ICM_20948.h"
#include "AS5047D.h"



/*******************************************************************************
 * #DEFINES                                                                    *
 ******************************************************************************/
#define HEARTBEAT_PERIOD 1000 //1 sec interval for hearbeat update
#define CONTROL_PERIOD 20 //Period for control loop in msec
#define ENCODER_PERIOD 10 //re-initiate encoder at 100 Hz
#define CONTROL_FREQUENCY (1000/CONTROL_PERIOD) //frequency in Hz
#define ENCODER_MAX_CTS 16384.0 //2^14 counts/rev for the encoder
#define ENCODER_TWO_PI 0.000383495197  //counts to 2 pi conversion = 2pi/0x4fff
#define WHEEL_RADIUS 0.0325 //meters
#define GPS_PERIOD 100 //10 Hz update rate
#define BUFFER_SIZE 1024
#define UINT_16_MAX 0xffff
#define KNOTS_TO_MPS 0.5144444444 //1 meter/second is equal to 1.9438444924406 knots
#define RAW 1
#define SCALED 2
#define CTS_2_USEC 1  //divide by 2 scaling of counts to usec with right shift

/*******************************************************************************
 * VARIABLES                                                                   *
 ******************************************************************************/
mavlink_system_t mavlink_system = {
    1, // System ID (1-255)
    MAV_COMP_ID_AUTOPILOT1 // Component ID (a MAV_COMPONENT value)
};

enum RC_channels {
    THR,
    AIL,
    ELE,
    RUD,
    SWITCH_A,
    SWITCH_B,
    SWITCH_C,
    SWITCH_D,
    SWITCH_E,
    SWITCH_1,
    SWITCH_F,
    SWITCH_G,
}; //map to the car controls from the RC receiver
//enum RC_channels {
//    ACCELERATOR = 2,
//    STEERING,
//    SWITCH_D,
//}; //map to the car controls from the RC receiver

const uint16_t RC_raw_fs_scale = RC_RAW_TO_FS;

RCRX_channel_buffer RC_channels[CHANNELS];
static struct GPS_data GPS_data;
struct IMU_output IMU_raw = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //container for raw IMU data
struct IMU_output IMU_scaled = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //container for scaled IMU data
encoder_t encoder_data[NUM_ENCODERS];
static uint8_t pub_GPS = TRUE;
static uint8_t pub_RC_servo = FALSE;
static uint8_t pub_RC_signals = TRUE;
static uint8_t pub_IMU = TRUE;
static uint8_t pub_Encoder = TRUE;

/*Pre-calculate float conversions*/
static float omega_to_dist = WHEEL_RADIUS * 2 * M_PI / ENCODER_MAX_CTS;
static float knots_to_mps = KNOTS_TO_MPS;
static float cts_to_deg = 360.0 / ENCODER_MAX_CTS;
static float freq = (float) CONTROL_FREQUENCY;
/*******************************************************************************
 * TYPEDEFS                                                                    *
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES                                                         *
 ******************************************************************************/
/**
 * @function check_IMU_events(void)
 * @param none
 * @brief detects when IMU SPI transaction completes and then publishes data over Mavlink
 * @author Aaron Hunter
 */
void check_IMU_events(void);
/**
 * @function RC_channels_init(void)
 * @param none
 * @brief set all RC channels to RC_RX_MID_COUNTS
 * @author Aaron Hunter
 */
void RC_channels_init(void);
/**
 * @function check_RC_events(void)
 * @param none
 * @brief checks for RC messages and stores data in RC channel buffer
 * @author Aaron Hunter
 */
void check_RC_events();
/**
 * @function check_radio_events(void)
 * @param none
 * @brief looks for messages sent over the radio serial port to OSAVC, parses
 * them and provides responses, if needed
 * @note currently only pushing information to usb-serial port
 * @author Aaron Hunter
 */
void check_radio_events(void);
/**
 * @function check_GPS_events(void)
 * @param none
 * @brief checks for GPS messages, parses, and stores data in module gps variable
 * @author aaron hunter
 */
void check_GPS_events(void);

/**
 * @function publish_IMU_data()
 * @param data_type RAW or SCALED
 * @brief reads module level IMU data and publishes over radio serial in Mavlink
 * @author Aaron Hunter
 */
void publish_IMU_data(uint8_t data_type);
/**
 * @function publish_RC_signals_raw(void)
 * @param none
 * @brief scales raw RC signals
 * @author Aaron Hunter
 */
void publish_RC_signals_raw(void);
/**
 * @Function publish_encoder_data()
 * @param none
 * @brief publishes motor data and steering angle data over MAVLink to radio
 * @return none
 * @author Aaron Hunter
 */
void publish_encoder_data(void);
/**
 * @function publish_GPS(void)
 * @param none
 * @brief invokes mavlink helper function to generate GPS message and sends to
 * radio serial port
 * @author aaron hunter
 */
void publish_GPS(void);
/**
 * @Function publish_heartbeat(void)
 * @param none
 * @brief invokes mavlink helper to generate heartbeat and sends out via the radio
 * @author aaron hunter
 */
void publish_heartbeat(void);

/**
 * @Function publish_parameter(uint8_t param_id[16])
 * @param parameter ID
 * @brief invokes mavlink helper to send out stored parameter 
 * @author aaron hunter
 */
void publish_parameter(uint8_t param_id[16]);

/**
 * @Function calc_pw(uint16_t raw_counts)
 * @param raw counts from the radio transmitter (11 bit unsigned int)
 * @return pulse width in microseconds
 * @brief converts the RC input into the equivalent pulsewidth output for servo
 * and ESC control
 * @author aahunter
 * @modified <Your Name>, <year>.<month>.<day> <hour> <pm/am> */
static uint16_t calc_pw(uint16_t raw_counts);

/**
 * @Function set_control_output(void)
 * @param none
 * @return none
 * @brief converts RC input signals to pulsewidth values and sets the actuators
 * (servos and ESCs) to those values
 * @author Aaron Hunter
 */
void set_control_output(void);

/*******************************************************************************
 * FUNCTIONS                                                                   *
 ******************************************************************************/

/**
 * @function check_IMU_events(void)
 * @param none
 * @brief detects when IMU SPI transaction completes and then publishes data over Mavlink
 * @author Aaron Hunter
 */
void check_IMU_events(void) {
    if (IMU_is_data_ready() == TRUE) {
        IMU_get_raw_data(&IMU_raw);
        //        publish_IMU_data(RAW);
        // IMU_get_scaled_data(&IMU_scaled);
        //        if (pub_IMU == TRUE) {
        //            publish_IMU_data(RAW);
        //        }
    }
}

/**
 * @Function check_encoder_events(void)
 * @param none
 * @brief looks for data_ready signal from encoder module and publishes if available over MAVLink
 * @return none
 * @author Aaron Hunter
 */
void check_encoder_events(void) {
    if (Encoder_is_data_ready() == TRUE) {
        Encoder_get_data(encoder_data);
        //        if (pub_Encoder == TRUE) {
        //            publish_encoder_data();
        //        }
    }
}

/**
 * @function RC_channels_init(void)
 * @param none
 * @brief set all RC channels to RC_RX_MID_COUNTS
 * @author Aaron Hunter
 */
void RC_channels_init(void) {
    uint8_t i;
    for (i = 0; i < CHANNELS; i++) {
        RC_channels[i] = RC_RX_MID_COUNTS;
    }
}

/**
 * @function check_RC_events(void)
 * @param none
 * @brief checks for RC messages and stores data in RC channel buffer
 * @author Aaron Hunter
 */
void check_RC_events() {
    if (RCRX_new_cmd_avail()) {
        RCRX_get_cmd(RC_channels);
//        if (pub_RC_signals == TRUE) {
//            publish_RC_signals_raw();
//        }
    }
}

/**
 * @function check_radio_events(void)
 * @param none
 * @brief looks for messages sent over the radio serial port to OSAVC, parses
 * them and provides responses, if needed
 * @note currently only pushing information to usb-serial port
 * @author Aaron Hunter
 */
void check_radio_events(void) {
    uint8_t channel = MAVLINK_COMM_0;
    uint8_t msg_byte;
    uint16_t msg_length;
    uint8_t msg_buffer[BUFFER_SIZE];
    mavlink_message_t msg_rx;
    mavlink_status_t msg_rx_status;

    //MAVLink command structs
    mavlink_heartbeat_t heartbeat;
    mavlink_command_long_t command_qgc;
    mavlink_param_request_read_t param_read;

    if (Radio_data_available()) {
        msg_byte = Radio_get_char();
        if (mavlink_parse_char(channel, msg_byte, &msg_rx, &msg_rx_status)) {
            switch (msg_rx.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    mavlink_msg_heartbeat_decode(&msg_rx, &heartbeat);
                    if (heartbeat.type)
                        printf("heartbeat received type(%d)\r\n", heartbeat.type);
                    break;
                case MAVLINK_MSG_ID_COMMAND_LONG:
                    mavlink_msg_command_long_decode(&msg_rx, &command_qgc);
                    printf("Command ID %d received from Ground Control\r\n", command_qgc.command);
                    break;
                case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                    mavlink_msg_param_request_read_decode(&msg_rx, &param_read);
                    printf("Parameter request ID %s received from Ground Control\r\n", param_read.param_id);
                    publish_parameter(param_read.param_id);
                    break;
                default:
                    printf("Received message with ID %d, sequence: %d from component %d of system %d\r\n",
                            msg_rx.msgid, msg_rx.seq, msg_rx.compid, msg_rx.sysid);
                    break;
            }
        }
    }
}

/**
 * @function check_GPS_events(void)
 * @param none
 * @brief checks for GPS messages, parses, and stores data in module gps variable
 * @author Aaron Hunter
 */
void check_GPS_events(void) {
    if (GPS_is_msg_avail() == TRUE) {
        GPS_parse_stream();
    }
    if (GPS_is_data_avail() == TRUE) {
        GPS_get_data(&GPS_data);
    }
}

/**
 * @function publish_IMU_data()
 * @param none
 * @brief reads module level IMU data and publishes over radio serial in Mavlink
 * @author Aaron Hunter
 */
void publish_IMU_data(uint8_t data_type) {
    mavlink_message_t msg_tx;
    uint16_t msg_length;
    uint8_t msg_buffer[BUFFER_SIZE];
    uint16_t index = 0;
    uint8_t IMU_id = 0;
    if (data_type == RAW) {
        mavlink_msg_raw_imu_pack(mavlink_system.sysid,
                mavlink_system.compid,
                &msg_tx,
                Sys_timer_get_usec(),
                (int16_t) IMU_raw.acc.x,
                (int16_t) IMU_raw.acc.y,
                (int16_t) IMU_raw.acc.z,
                (int16_t) IMU_raw.gyro.x,
                (int16_t) IMU_raw.gyro.y,
                (int16_t) IMU_raw.gyro.z,
                (int16_t) IMU_raw.mag.x,
                (int16_t) IMU_raw.mag.y,
                (int16_t) IMU_raw.mag.z,
                IMU_id,
                (int16_t) IMU_raw.temp
                );
    } else if (data_type == SCALED) {
        mavlink_msg_highres_imu_pack(mavlink_system.sysid,
                mavlink_system.compid,
                &msg_tx,
                Sys_timer_get_usec(),
                (float) IMU_scaled.acc.x,
                (float) IMU_scaled.acc.y,
                (float) IMU_scaled.acc.z,
                (float) IMU_scaled.gyro.x,
                (float) IMU_scaled.gyro.y,
                (float) IMU_scaled.gyro.z,
                (float) IMU_scaled.mag.x,
                (float) IMU_scaled.mag.y,
                (float) IMU_scaled.mag.z,
                0.0, //no pressure
                0.0, //no diff pressure
                0.0, //no pressure altitude
                (float) IMU_scaled.temp,
                0, //bitfields updated
                IMU_id
                );
    }
    msg_length = mavlink_msg_to_send_buffer(msg_buffer, &msg_tx);
    for (index = 0; index < msg_length; index++) {
        Radio_put_char(msg_buffer[index]);
    }
    /*Test for values, comment out when done:*/
    //    printf("a: %d %d %d g: %d %d %d m: %d %d %d \r",  IMU_scaled.acc.x, IMU_scaled.acc.y, IMU_scaled.acc.z, 
    //    IMU_scaled.gyro.x,IMU_scaled.gyro.y,IMU_scaled.gyro.z, IMU_scaled.mag.x, IMU_scaled.mag.x, IMU_scaled.mag.z);
}

///**
// * @Function publish_encoder_data()
// * @param none
// * @brief publishes motor data and steering angle data over MAVLink using 
// * rc_channels_scaled. This is a bit of a hack until I can generate my own message
// * @return none
// * @author Aaron Hunter
// */

void publish_encoder_data(void) {
    mavlink_message_t msg_tx;
    uint16_t msg_length;
    uint8_t msg_buffer[BUFFER_SIZE];
    uint16_t index = 0;
    uint8_t i;
    uint8_t port = 1;
    uint8_t rssi = 255; //unknown--may be able to extract from receiver
    int16_t angle[CHANNELS] = {UINT_16_MAX, UINT_16_MAX, UINT_16_MAX, UINT_16_MAX, UINT_16_MAX, UINT_16_MAX, UINT_16_MAX, UINT_16_MAX};
    // encode the distance traveled into the first NUM_ENCODER distances
    for (i = 0; i < NUM_ENCODERS; i++) {
        //        angle[i] = encoder_data[i].omega * omega_to_dist;
        angle[i] = encoder_data[i].omega;
    }
    //encode the exact angle for the steering servo--not necessary for the wheel encoders
    //    angle[NUM_ENCODERS] = encoder_data[HEADING].next_theta * cts_to_deg;
    angle[NUM_ENCODERS] = encoder_data[HEADING].next_theta;

    /* Examine output of conversion, remove this code once the output is confirmed*/
    //    printf("D; %f %f %f %f \r", distance[0], distance[1], distance[2], distance[3] );

    mavlink_msg_rc_channels_scaled_pack(
            mavlink_system.sysid,
            mavlink_system.compid,
            &msg_tx,
            (uint32_t) Sys_timer_get_msec(),
            port,
            angle[0],
            angle[1],
            angle[2],
            angle[3],
            angle[4],
            angle[5],
            angle[6],
            angle[7],
            rssi);
    msg_length = mavlink_msg_to_send_buffer(msg_buffer, &msg_tx);
    //    printf("first char %x \r\n", msg_buffer[0]);
    for (index = 0; index < msg_length; index++) {
        Radio_put_char(msg_buffer[index]);
    }
//        printf("last char %x \r\n", msg_buffer[index - 1]);
}

/**
 * @function publish_RC_signals(void)
 * @param none
 * @brief scales raw RC signals into +/- 10000
 * @author Aaron Hunter
 */
void publish_RC_signals(void) {
    mavlink_message_t msg_tx;
    uint16_t msg_length;
    uint8_t msg_buffer[BUFFER_SIZE];
    uint16_t index = 0;
    uint8_t RC_port = 0; //first 8 channels 
    int16_t scaled_channels[CHANNELS];
    uint8_t rssi = 255; //unknown--may be able to extract from receiver
    for (index = 0; index < CHANNELS; index++) {
        scaled_channels[index] = (RC_channels[index] - RC_RX_MID_COUNTS) * RC_raw_fs_scale;
    }
    mavlink_msg_rc_channels_scaled_pack(mavlink_system.sysid,
            mavlink_system.compid,
            &msg_tx,
            Sys_timer_get_msec(),
            RC_port,
            scaled_channels[0],
            scaled_channels[1],
            scaled_channels[2],
            scaled_channels[3],
            scaled_channels[4],
            scaled_channels[5],
            scaled_channels[6],
            scaled_channels[7],
            rssi);
    msg_length = mavlink_msg_to_send_buffer(msg_buffer, &msg_tx);
    for (index = 0; index < msg_length; index++) {
        Radio_put_char(msg_buffer[index]);
    }
}

/**
 * @function publish_RC_signals_raw(void)
 * @param none
 * @brief scales raw RC signals
 * @author Aaron Hunter
 */
void publish_RC_signals_raw(void) {
    mavlink_message_t msg_tx;
    uint16_t msg_length;
    uint8_t msg_buffer[BUFFER_SIZE];
    uint16_t index = 0;
    uint8_t RC_port = 0; //first 8 channels 
    uint8_t rssi = 255; //unknown--may be able to extract from receiver
    mavlink_msg_rc_channels_raw_pack(mavlink_system.sysid,
            mavlink_system.compid,
            &msg_tx,
            Sys_timer_get_msec(),
            RC_port,
            RC_channels[0],
            RC_channels[1],
            RC_channels[2],
            RC_channels[3],
            RC_channels[4],
            RC_channels[5],
            RC_channels[6],
            RC_channels[7],
            rssi);
    msg_length = mavlink_msg_to_send_buffer(msg_buffer, &msg_tx);
    for (index = 0; index < msg_length; index++) {
        Radio_put_char(msg_buffer[index]);
    }
}

/**
 * @function publish_GPS(void)
 * @param none
 * @brief invokes mavlink helper function to generate GPS message and sends to
 * radio serial port
 * @author Aaron Hunter
 */
void publish_GPS(void) {
    static uint8_t gps_fix = GPS_FIX_TYPE_NO_FIX;
    mavlink_message_t msg_tx;
    uint16_t msg_length;
    uint8_t msg_buffer[BUFFER_SIZE];
    uint16_t index = 0;

    //verify fix status
    if (GPS_has_fix() == TRUE) {
        gps_fix = GPS_FIX_TYPE_3D_FIX;
    } else {
        gps_fix = GPS_FIX_TYPE_NO_FIX;
    }
    mavlink_msg_gps_raw_int_pack(mavlink_system.sysid,
            mavlink_system.compid,
            &msg_tx,
            (uint64_t) Sys_timer_get_usec(),
            gps_fix,
            (int32_t) (GPS_data.lat * 10000000.0),
            (int32_t) (GPS_data.lon * 10000000.0),
            0, //altitude --can update GPS data if need this
            UINT_16_MAX, //hdop--currently don't care
            UINT_16_MAX, //vdop
            (uint16_t) (GPS_data.spd * KNOTS_TO_MPS * 100.0), //need to verify units and convert from knots if so
            (uint16_t) (GPS_data.cog * 100.0), //cdeg TODO verify heading angle between 0 and 359.99
            255, //satellites visible again, currently don't care
            0, //alt ellipsoid
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

/**
 * @Function publish_heartbeat(void)
 * @param none
 * @brief invokes mavlink helper to generate heartbeat and sends out via the radio
 * @author aaron hunter
 */
void publish_heartbeat(void) {
    mavlink_message_t msg_tx;
    uint16_t msg_length;
    uint8_t msg_buffer[BUFFER_SIZE];
    uint16_t index = 0;
    uint8_t mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED;
    uint32_t custom = 0;
    uint8_t state = MAV_STATE_STANDBY;
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
}

/**
 * @Function publish_parameter(uint8_t param_id[16])
 * @param parameter ID
 * @brief invokes mavlink helper to send out stored parameter 
 * @author aaron hunter
 */
void publish_parameter(uint8_t param_id[16]) {
    mavlink_message_t msg_tx;
    uint16_t msg_length;
    uint8_t msg_buffer[BUFFER_SIZE];
    uint16_t index = 0;
    float param_value = 320.0; // value of the requested parameter
    uint8_t param_type = MAV_PARAM_TYPE_INT16; // onboard mavlink parameter type
    uint16_t param_count = 1; // total number of onboard parameters
    uint16_t param_index = 1; //index of this value
    mavlink_msg_param_value_pack(mavlink_system.sysid,
            mavlink_system.compid,
            &msg_tx,
            param_id,
            param_value,
            param_type,
            param_count,
            param_index
            );
    msg_length = mavlink_msg_to_send_buffer(msg_buffer, &msg_tx);
    for (index = 0; index < msg_length; index++) {
        Radio_put_char(msg_buffer[index]);
    }
}

/**
 * @Function calc_pw(uint16_t raw_counts)
 * @param raw counts from the radio transmitter (11 bit unsigned int)
 * @return pulse width in microseconds
 * @brief converts the RC input into the equivalent pulsewidth output for servo
 * and ESC control
 * @author aahunter
 * @modified <Your Name>, <year>.<month>.<day> <hour> <pm/am> */
static uint16_t calc_pw(uint16_t raw_counts) {
    int16_t normalized_pulse; //converted to microseconds and centered at 0
    uint16_t pulse_width; //servo output in microseconds
    normalized_pulse = (raw_counts - RC_RX_MID_COUNTS) >> CTS_2_USEC;
    pulse_width = normalized_pulse + RC_SERVO_CENTER_PULSE;
    return pulse_width;
}

/**
 * @Function set_control_output(void)
 * @param none
 * @return none
 * @brief converts RC input signals to pulsewidth values and sets the actuators
 * (servos and ESCs) to those values
 * @author Aaron Hunter
 */
void set_control_output(void) {
    static uint8_t debounce = 0;
    uint8_t bounces = 5;
    int tol = 100;
    if (RC_channels[SWITCH_D] > RC_RX_MAX_COUNTS - tol) {
        if (debounce > 0) debounce--;
        if (debounce == 0) {
            // update pulsewidths for each servo output
            // we're using the elevator channel for accelerator 
            RC_servo_set_pulse(calc_pw(RC_channels[ELE]), RC_LEFT_WHEEL);
            RC_servo_set_pulse(calc_pw(RC_channels[ELE]), RC_RIGHT_WHEEL);
            // and rudder for steering
            RC_servo_set_pulse(calc_pw(RC_channels[RUD]), RC_STEERING);
        }
    } else {
        debounce++;
        if (debounce >= bounces) {
            debounce = bounces;
            RC_servo_set_pulse(calc_pw(RC_RX_MID_COUNTS), RC_LEFT_WHEEL);
            RC_servo_set_pulse(calc_pw(RC_RX_MID_COUNTS), RC_RIGHT_WHEEL);
            RC_servo_set_pulse(calc_pw(RC_RX_MID_COUNTS), RC_STEERING);
        }
    }
}

int main(void) {
    uint32_t cur_time = 0;
    uint32_t warmup_time = 250; //time in ms to allow subsystems to stabilize (IMU))
    uint32_t gps_start_time = 0;
    uint32_t control_start_time = 0;
    uint32_t encoder_start_time = 0;
    uint32_t heartbeat_start_time = 0;
    uint8_t index;
    int8_t IMU_state = ERROR;
    int8_t IMU_retry = 5;
    uint32_t IMU_error = 0;
    uint8_t error_report = 50;

    //Initialization routines
    Board_init(); //board configuration
    Serial_init(); //start debug terminal (USB)
    Sys_timer_init(); //start the system timer
    /*small delay to get all the subsystems time to get online*/
    while (cur_time < warmup_time) {
        cur_time = Sys_timer_get_msec();
    }
    Radio_serial_init(); //start the radios
    GPS_init();
    RCRX_init(); //initialize the radio control system
    RC_channels_init(); //set channels to midpoint of RC system
    RC_servo_init(); // start the servo subsystem
    IMU_state = IMU_init(IMU_SPI_MODE);
    if (IMU_state == ERROR && IMU_retry > 0) {
        IMU_state = IMU_init(IMU_SPI_MODE);
        printf("IMU failed init, retrying %d \r\n", IMU_retry);
        IMU_retry--;
    }
    Encoder_init();
    //initialize encoder data
    for (index = 0; index < NUM_ENCODERS; index++) {
        Encoder_init_encoder_data(&encoder_data[index]);
    }

    printf("\r\nMinimal Mavlink application %s, %s \r\n", __DATE__, __TIME__);

    while (1) {
        cur_time = Sys_timer_get_msec();
        //check for all events
        check_IMU_events(); //check for IMU data ready and publish when available
        check_encoder_events(); //check for encoder data ready and publish when available
        check_radio_events(); //detect and process MAVLink incoming messages
        check_RC_events(); //check incoming RC commands
        check_GPS_events(); //check and process incoming GPS messages

        //for now start encoder every ENCODER_PERIOD but publish every CONTROL_PERIOD
        if (cur_time - encoder_start_time >= ENCODER_PERIOD) {
            encoder_start_time = cur_time; //reset control loop timer
            Encoder_start_data_acq(); //initiate Encoder measurement with SPI
        }


        //publish control and sensor signals
        if (cur_time - control_start_time >= CONTROL_PERIOD) {
            control_start_time = cur_time; //reset control loop timer
            set_control_output(); // set actuator outputs
            /*publish high speed sensors*/
            if (pub_RC_signals == TRUE) {
                publish_RC_signals_raw();
            }
            if (pub_IMU == TRUE) {
                publish_IMU_data(RAW);
            }
            if (pub_Encoder == TRUE) {
                publish_encoder_data();
            }
            /*start next data acquisition round*/
            IMU_state = IMU_start_data_acq(); //initiate IMU measurement with SPI
            if (IMU_state == ERROR) {
                IMU_error++;
                if (IMU_error % error_report == 0) {
                    printf("IMU error count %d\r\n", IMU_error);
                }
            }
        }

        //publish GPS
        if (cur_time - gps_start_time > GPS_PERIOD) {
            gps_start_time = cur_time; //reset GPS timer
            if (pub_GPS == TRUE) {
                publish_GPS();
            }
        }
        //publish heartbeat
        if (cur_time - heartbeat_start_time >= HEARTBEAT_PERIOD) {
            heartbeat_start_time = cur_time; //reset the timer
            publish_heartbeat();
            //            printf("RC_RX errors: %d\r", RCRX_get_err());

        }
    }
    return 0;
}
/*
 * File:   Publisher.c
 * Author: Aaron Hunter, Modified by Pavlo Vlastos
 * Brief:  Publish events after checking
 * 
 * Updated from Mavlink_minimal.X/main.c on September 3, 2021 at 2:24 pm
 */

/******************************************************************************
 * #INCLUDES                                                                  *
 *****************************************************************************/
#include "Publisher.h"

#include "xc.h"
#ifdef USB_DEBUG
#include "SerialM32.h"
#else
#include "MavSerial.h"
#endif

/******************************************************************************
 * #DEFINES                                                                   *
 *****************************************************************************/
#define HEARTBEAT_PERIOD 1000 //1 sec interval for hearbeat update
#define GPS_PERIOD 1000 //1 Hz update rate (For the time being)
#define CONTROL_PERIOD 20 //Period for control loop in msec
#define RAW 1
#define SCALED 2
#define BUFFER_SIZE 1024
#define CTS_2_USEC 1  //divide by 2 scaling of counts to usec with right shift

/******************************************************************************
 * VARIABLES                                                                  *
 *****************************************************************************/
// GPS
int32_t rmc_lat_int = 0;
long double rmc_lat = 0.0;
int32_t rmc_long_int = 0;
long double rmc_long = 0.0;
long double rmc_cog = 0.0;
float rmc_position[DIM];
long double unw_cog = 0.0;
long double offset = 0.0;
int offsetFlag = 1;
int new_measurement = FALSE;
long double heading_angle = 0.0; // Purposefully abusing this definition
long double rmc_vel = 0.0;
long double gsa_hdop = 0.0;

nmea_frame_t cur_msg;

// MAVLink
static uint8_t mode = MAV_MODE_MANUAL_ARMED;
static uint8_t autopilot = MAV_TYPE_SURFACE_BOAT;
static uint8_t state = MAV_STATE_STANDBY;

mavlink_system_t mavlink_system = {
    1, // System ID (1-255)
    MAV_COMP_ID_AUTOPILOT1 // Component ID (a MAV_COMPONENT value)
};

static mavlink_message_t rec_msg;

// @TODO: Move RC channel specific variables to RC and use/add getters

enum RC_channels {
    ACCELERATOR = 2,
    STEERING,
    SWITCH_D,
}; //map to the car or boat controls from the RC receiver

RCRX_channel_buffer RC_channels[CHANNELS];
//static struct GPS_data GPS_data;
struct IMU_output IMU_raw; //container for raw IMU data
struct IMU_output IMU_scaled; //container for scaled IMU data
//encoder_t encoder_data[NUM_ENCODERS];
//static uint8_t pub_Encoder = TRUE;

/******************************************************************************
 * FUNCTION PROTOTYPES                                                        *
 *****************************************************************************/
void publisher_init(uint8_t desired_autopilot) {
    publisher_set_autopilot(desired_autopilot);
    publisher_set_mode(MAV_MODE_MANUAL_DISARMED);
    publisher_set_state(MAV_STATE_STANDBY);
}

void publisher_set_autopilot(uint8_t desired_autopilot) {
    autopilot = desired_autopilot;
}

void publisher_set_mode(uint8_t desired_mode) {
    mode = desired_mode;
}

void publisher_set_state(uint8_t desired_state) {
    state = desired_state;
}

uint8_t check_mavlink_mode(void) {
    if ((uint16_t) RC_channels[4] > RC_RX_MID_COUNTS) {
        mode = MAV_MODE_AUTO_ARMED;
    } else {
        mode = MAV_MODE_MANUAL_ARMED;
    }
    return mode;
}

void publisher_get_gps_rmc_position(float position[DIM]) {
    rmc_position[0] = position[0];
    rmc_position[1] = position[1];
}

void check_IMU_events(uint8_t data_type) {
    if (IMU_is_data_ready() == TRUE) {
        if (data_type == RAW) {
            IMU_get_raw_data(&IMU_raw);
        }
        if (data_type == SCALED) {
            IMU_get_scaled_data(&IMU_scaled);
        }
    }
}

void RC_channels_init(void) {
    uint8_t i;
    for (i = 0; i < CHANNELS; i++) {
        RC_channels[i] = RC_RX_MID_COUNTS;
    }
}

void check_RC_events() {
    if (RCRX_new_cmd_avail()) {
        LATCbits.LATC1 ^= 1; /* Toggle  LED5 */
        RCRX_get_cmd(RC_channels);
    }
}

void check_GPS_events(void) {
#ifdef NEO_GPS
    if (GPS_is_msg_avail() == TRUE) {
        GPS_parse_stream();
    }
    if (GPS_is_data_avail() == TRUE) {
        GPS_get_data(&GPS_data);
    }
#endif

#ifdef SAM_M8Q_GPS
    if (nmea_is_msg_available() == TRUE) {
        // CHECK FOR XXRMC
        if ((nmea_read_head_address().talk_id[0] == 'G') &&
                (nmea_read_head_address().talk_id[1] == 'N') &&
                (nmea_read_head_address().s_frmtr[0] == 'R') &&
                (nmea_read_head_address().s_frmtr[1] == 'M') &&
                (nmea_read_head_address().s_frmtr[2] == 'C')) {
            if (nmea_get_data(&(cur_msg.data)) == SUCCESS) {
                new_measurement = TRUE;
                nmea_rmc_parse2(&cur_msg);

                rmc_lat_int = nmea_get_rmc_lat_int();
                rmc_lat = nmea_get_rmc_lat();
                rmc_long_int = -nmea_get_rmc_long_int(); //@TODO: Fix sign
                rmc_long = -nmea_get_rmc_long(); //@TODO: Fix sign
                rmc_cog = nmea_get_rmc_cog();
                rmc_vel = nmea_get_rmc_spd();

                rmc_position[0] = (float) rmc_lat;
                rmc_position[1] = (float) rmc_long;
            }
            // CHECK FOR XXGSA for HDOP
        } else if ((nmea_read_head_address().talk_id[0] == 'G') &&
                (nmea_read_head_address().talk_id[1] == 'N') &&
                (nmea_read_head_address().s_frmtr[0] == 'G') &&
                (nmea_read_head_address().s_frmtr[1] == 'S') &&
                (nmea_read_head_address().s_frmtr[2] == 'A')) {
            if (nmea_get_data(&(cur_msg.data)) == SUCCESS) {
                gsa_hdop = nmea_get_gsa_hdop(&cur_msg);
            }
        } else {
            nmea_get_data(&(cur_msg.data)); // Call to remove msgs from buf
        }
    }
#endif
}

void publish_IMU_data(uint8_t data_type) {
    mavlink_message_t msg_tx;
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

    MavSerial_sendMavPacket(&msg_tx);
}

void publish_RC_signals(void) {
    mavlink_message_t msg_tx;
    uint16_t index = 0;
    uint8_t RC_port = 0; //first 8 channels 
    int16_t scaled_channels[CHANNELS];
    uint8_t rssi = 255; //unknown--may be able to extract from receiver

    for (index = 0; index < CHANNELS; index++) {
        scaled_channels[index] = (RC_channels[index] - RC_RX_MID_COUNTS) * RC_RAW_TO_FS;
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

    MavSerial_sendMavPacket(&msg_tx);
}

void publish_RC_signals_raw(void) {
    mavlink_message_t msg_tx;
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

    MavSerial_sendMavPacket(&msg_tx);
}

char check_mavlink_serial_events(union lat_lon_point* wp) {
    char status = FALSE;
    uint16_t command = 0;
    
    wp->latitude = 0.0;
    wp->longitude = 0.0;
    
    status = MavSerial_getMavMsg(&rec_msg);
    
    if (status == TRUE) {
        command = mavlink_msg_command_long_get_command(&rec_msg);
        switch (command) {
            case MAV_CMD_COMPONENT_ARM_DISARM:
                break;
            case MAV_CMD_NAV_WAYPOINT:
                wp->latitude = mavlink_msg_command_long_get_param4(&rec_msg);
                wp->longitude = mavlink_msg_command_long_get_param5(&rec_msg);
                break;
        }
    }
    return status;
}

void publish_GPS(void) {
    mavlink_message_t msg_tx;
    static uint8_t gps_fix = GPS_FIX_TYPE_NO_FIX;

#ifdef NEO_GPS
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
#endif

#ifdef SAM_M8Q_GPS
    /* Send GPS Position for QGC Visualization*/
    mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid,
            &msg_tx,
            Sys_timer_get_usec(),
            GPS_FIX_TYPE_3D_FIX,
            rmc_lat_int,
            rmc_long_int,
            (int) (heading_angle * 1000000.0), // Replacing altitude with heading_angle
            (uint16_t) (gsa_hdop * 1000000.0),
            (uint16_t) (rmc_vel * 1000), // Replacing epv/VDOP velocity - TODO: find why vel doesn't show up in .csv log file
            (uint16_t) (rmc_vel * 1000),
            (uint16_t) (rmc_cog),
            255, //satellites visible again, currently don't care
            0, //alt ellipsoid
            0, //h position uncertainty
            0, //v position uncertainty
            0, //velocity uncertainty
            0, //heading uncertainty
            0); // yaw--GPS doesn't provide
#endif

    MavSerial_sendMavPacket(&msg_tx);
}

void publish_heartbeat(void) {
    mavlink_message_t msg_tx;
    uint32_t custom = 0;
    mavlink_msg_heartbeat_pack(mavlink_system.sysid,
            mavlink_system.compid,
            &msg_tx,
            MAV_TYPE_GENERIC,
            autopilot,
            mode,
            custom,
            state);

    MavSerial_sendMavPacket(&msg_tx);
}

void publish_parameter(const char *param_id) {
    mavlink_message_t msg_tx;
    float param_value = 320.0; // value of the requested parameter
    char param_type = MAV_PARAM_TYPE_INT16; // onboard mavlink parameter type
    uint16_t param_count = 1; // total number of onboard parameters
    uint16_t param_index = 1; //index of this value
    mavlink_msg_param_value_pack(mavlink_system.sysid,
            mavlink_system.compid,
            &msg_tx,
            param_id,
            param_value,
            param_type,
            param_count,
            param_index);

    MavSerial_sendMavPacket(&msg_tx);
}

int publish_waypoint(float wp[DIM]) {
    mavlink_message_t msg_tx;
    mavlink_msg_local_position_ned_pack(mavlink_system.sysid,
            mavlink_system.compid,
            &msg_tx,
            Sys_timer_get_usec(),
            wp[1], /* x: Latitude [meters] EN LTP -> y_north -> wp[1]*/
            wp[0], /* y: Longitude[meters] EN LTP -> x_east -> wp[0]*/
            0.0, /* z: Altitude [meters] */
            0.0, /* vx [meters/second] */
            0.0, /* vy [meters/second] */
            0.0); /* vz [meters/second] */

    MavSerial_sendMavPacket(&msg_tx);
}

int publish_ack(uint8_t result) {
    mavlink_message_t msg_tx;
    /* Send a command acknowledgment */
    mavlink_msg_command_ack_pack(mavlink_system.sysid,
            mavlink_system.compid,
            &msg_tx, 
            MAV_CMD_ACK_OK, 
            result);

    MavSerial_sendMavPacket(&msg_tx);
    return SUCCESS;
}

uint16_t calc_pw(uint16_t raw_counts) {
    int16_t normalized_pulse; //converted to microseconds and centered at 0
    uint16_t pulse_width; //servo output in microseconds
    normalized_pulse = (raw_counts - RC_RX_MID_COUNTS) >> CTS_2_USEC;
    pulse_width = normalized_pulse + RC_SERVO_CENTER_PULSE;
    return pulse_width;
}

void set_control_output(void) {
    int tol = 100;
    if (RC_channels[SWITCH_D] > RC_RX_MAX_COUNTS - tol) {
        // update pulsewidths for each servo output
        RC_servo_set_pulse(calc_pw(RC_channels[ACCELERATOR]), RC_LEFT_WHEEL);
        RC_servo_set_pulse(calc_pw(RC_channels[ACCELERATOR]), RC_RIGHT_WHEEL);
        RC_servo_set_pulse(calc_pw(RC_channels[STEERING]), RC_STEERING);
    } else {
        RC_servo_set_pulse(calc_pw(RC_RX_MID_COUNTS), RC_LEFT_WHEEL);
        RC_servo_set_pulse(calc_pw(RC_RX_MID_COUNTS), RC_RIGHT_WHEEL);
        RC_servo_set_pulse(calc_pw(RC_RX_MID_COUNTS), RC_STEERING);
    }
}
/******************************************************************************
 * UNIT TEST(S)                                                               *
 *****************************************************************************/
#ifdef PUBLISHER_TEST

#include "xc.h"
#include "Board.h"
//#include "Radio_serial.h"
#include "common/mavlink.h"
//#include "NEO_M8N.h"
//#include "AS5047D.h"

#ifdef USB_DEBUG
#include "SerialM32.h"
#else
#include "MavSerial.h"
#endif

int main(void) {
    uint32_t cur_time = 0;
    uint32_t gps_start_time = 0;
    uint32_t control_start_time = 0;
    uint32_t heartbeat_start_time = 0;
    //    uint8_t index;
    int8_t IMU_state = ERROR;
    int8_t IMU_retry = 5;
    uint32_t IMU_error = 0;
    uint8_t error_report = 50;

    //Initialization routines
    Board_init(); //board configuration

    TRISAbits.TRISA3 = 0; /* Set pin as output. This is also LED4 on Max32 */
    TRISCbits.TRISC1 = 0; /* LED5 */

    LATCbits.LATC1 = 1; /* Set LED5 */
    LATAbits.LATA3 = 1; /* Set LED4 */

#ifdef USB_DEBUG
    Serial_init(); //start debug terminal (USB)
#else
    MavSerial_Init();
#endif

#ifdef NEO_GPS
    GPS_init();
#endif

#ifdef SAM_M8Q_GPS
    nmea_serial_init();
#endif

    Sys_timer_init(); //start the system timer

#ifdef USING_RC
    RCRX_init(); //initialize the radio control system
    RC_channels_init(); //set channels to midpoint of RC system
    RC_servo_init(); // start the servo subsystem
#endif

#ifdef USING_IMU
    IMU_state = IMU_init(IMU_SPI_MODE);

    if (IMU_state == ERROR && IMU_retry > 0) {
        IMU_state = IMU_init(IMU_SPI_MODE);
#ifdef USB_DEBUG
        printf("IMU failed init, retrying %f \r\n", IMU_retry);
#endif
        IMU_retry--;
    }
#endif

    LATCbits.LATC1 = 0; /* Set LED5 low */
    LATAbits.LATA3 = 0; /* Set LED4 low */

#ifdef USB_DEBUG
    printf("\r\nMinimal Mavlink application %s, %s \r\n", __DATE__, __TIME__);
#endif

    unsigned int control_loop_count = 0;

    /**************************************************************************
     * Primary Loop                                                           *
     *************************************************************************/
    while (1) {
        cur_time = Sys_timer_get_msec();

        /**********************************************************************
         * Check for all events                                               *
         *********************************************************************/

#ifdef USING_IMU
        //        check_IMU_events(RAW); //check for IMU data ready and publish when available
        check_IMU_events(SCALED);
#endif
        check_mavlink_serial_events();

#ifdef USING_RC
        check_RC_events(); //check incoming RC commands
#endif

#ifdef USING_GPS
        check_GPS_events(); //check and process incoming GPS messages
#endif

        /**********************************************************************
         * Publish control and sensor signals                                 *
         *********************************************************************/
        if ((cur_time - control_start_time) > CONTROL_PERIOD) {
            control_start_time = cur_time; //reset control loop timer

            control_loop_count++;

#ifdef USING_RC
            set_control_output(); // set actuator outputs
            publish_RC_signals_raw();
#endif

#ifdef USING_IMU
            IMU_state = IMU_start_data_acq(); //initiate IMU measurement with SPI

            if (IMU_state == ERROR) {
                IMU_error++;
                if (IMU_error % error_report == 0) {

#ifdef USB_DEBUG
                    printf("IMU error count %d\r\n", IMU_error);
#endif

                }
            }

            publish_IMU_data(SCALED);
#endif

        }

        // Publish GPS
#ifdef USING_GPS
        if (cur_time - gps_start_time >= GPS_PERIOD) {
            gps_start_time = cur_time; //reset GPS timer
            publish_GPS();
        }
#endif

        // Publish heartbeat
        if (cur_time - heartbeat_start_time >= HEARTBEAT_PERIOD) {
            heartbeat_start_time = cur_time; //reset the timer
            publish_heartbeat();

            LATAbits.LATA3 ^= 1; /* Set LED4 */

        }
    }
    /******************* END OF PRIMARY LOOP *********************************/

    while (1); /* Wait and spin if we somehow exit the while(1){} above */

    return 0;
}
#endif


#include <stdio.h>
#include "D:/GSOC/CROSS/MAVlink/c_library_v2-master/c_library_v2-master/common/mavlink.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "mavlink.h"

mavlink_system_t mavlink_system = {
    1,                      // System ID (1-255)
    MAV_COMP_ID_AUTOPILOT1  // Component ID (a MAV_COMPONENT value)
};

int BUFFER_SIZE = 1024;

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
        // putchar_raw(msg_buffer[index]);
        
        //printf("\n%c", msg_buffer[index]);

        //char *str = msg_buffer[index];
        //uart_puts(uart0, str);
        
        
        // char *str = (char)msg_buffer[index];
        //uart_puts(uart0, str);

        //puts_raw(str);

        uart_putc_raw(uart0, msg_buffer[index]);
        
        }
}


void publish_lidar_data(uint16_t distances[], float offset, uint8_t user_increment)
{
    mavlink_message_t msg_tx;
    uint16_t msg_length;
    uint8_t sensor_type = 0;
    uint8_t msg_buffer[BUFFER_SIZE];
    uint16_t index = 0;
    uint8_t increment = user_increment;
    uint16_t min_distance = 5;
    uint16_t max_distnace = 4000;
    float angle_offset = offset;
    mavlink_msg_obstacle_distance_pack(mavlink_system.sysid,
        mavlink_system.compid,
        &msg_tx,
        time_us_64(),
        sensor_type,
        distances,
        increment,
        min_distance,
        max_distnace,
        0,
        angle_offset,
        MAV_FRAME_BODY_FRD
    );
    msg_length = mavlink_msg_to_send_buffer(msg_buffer, &msg_tx);
    for (index = 0; index < msg_length; index++) 
    {
        uart_putc_raw(uart0, msg_buffer[index]);
    }
}


void set_mavlink_communication()
{
    uart_init(uart0, 115200);

    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
}


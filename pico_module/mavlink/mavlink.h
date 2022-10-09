#ifndef MAVLINK_H
#define MAVLINK_H

#include "D:/GSOC/CROSS/MAVlink/c_library_v2-master/c_library_v2-master/common/mavlink.h"

void publish_heartbeat(void);

void publish_lidar_data(uint16_t distances[], float offset, uint8_t increment);

void set_mavlink_communication();

#endif
/** @file
 *  @brief MAVLink comm protocol generated from autorobot.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_AUTOROBOT_H
#define MAVLINK_AUTOROBOT_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_AUTOROBOT.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{1, 119, 12, 0, 0, 0}, {2, 87, 12, 0, 0, 0}, {3, 107, 8, 0, 0, 0}, {31, 154, 8, 0, 0, 0}, {32, 154, 8, 0, 0, 0}, {101, 35, 9, 0, 0, 0}, {131, 90, 12, 0, 0, 0}, {132, 3, 12, 0, 0, 0}, {133, 28, 10, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_AUTOROBOT

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 1
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 1
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_left_wheel_pid_gains.h"
#include "./mavlink_msg_right_wheel_pid_gains.h"
#include "./mavlink_msg_robot_dimensions.h"
#include "./mavlink_msg_desire_wheel_rpm.h"
#include "./mavlink_msg_desire_cmd_val.h"
#include "./mavlink_msg_initial_status.h"
#include "./mavlink_msg_robot_position_change.h"
#include "./mavlink_msg_robot_sensor_readings.h"
#include "./mavlink_msg_robot_distance_sensor_readings.h"

// base include


#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_LEFT_WHEEL_PID_GAINS, MAVLINK_MESSAGE_INFO_RIGHT_WHEEL_PID_GAINS, MAVLINK_MESSAGE_INFO_ROBOT_DIMENSIONS, MAVLINK_MESSAGE_INFO_DESIRE_WHEEL_RPM, MAVLINK_MESSAGE_INFO_DESIRE_CMD_VAL, MAVLINK_MESSAGE_INFO_INITIAL_STATUS, MAVLINK_MESSAGE_INFO_ROBOT_POSITION_CHANGE, MAVLINK_MESSAGE_INFO_ROBOT_SENSOR_READINGS, MAVLINK_MESSAGE_INFO_ROBOT_DISTANCE_SENSOR_READINGS}
# define MAVLINK_MESSAGE_NAMES {{ "DESIRE_CMD_VAL", 32 }, { "DESIRE_WHEEL_RPM", 31 }, { "INITIAL_STATUS", 101 }, { "LEFT_WHEEL_PID_GAINS", 1 }, { "RIGHT_WHEEL_PID_GAINS", 2 }, { "ROBOT_DIMENSIONS", 3 }, { "ROBOT_DISTANCE_SENSOR_READINGS", 133 }, { "ROBOT_POSITION_CHANGE", 131 }, { "ROBOT_SENSOR_READINGS", 132 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_AUTOROBOT_H

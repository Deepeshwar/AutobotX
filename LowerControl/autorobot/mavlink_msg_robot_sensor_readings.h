#pragma once
// MESSAGE ROBOT_SENSOR_READINGS PACKING

#define MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS 132

MAVPACKED(
typedef struct __mavlink_robot_sensor_readings_t {
 uint32_t left_wheel_ticks; /*<  left wheel encoder ticks*/
 uint32_t right_wheel_ticks; /*<  right wheel encoder ticks*/
 float curr_heading; /*<  current heading of robot*/
}) mavlink_robot_sensor_readings_t;

#define MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN 12
#define MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_MIN_LEN 12
#define MAVLINK_MSG_ID_132_LEN 12
#define MAVLINK_MSG_ID_132_MIN_LEN 12

#define MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_CRC 3
#define MAVLINK_MSG_ID_132_CRC 3



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROBOT_SENSOR_READINGS { \
    132, \
    "ROBOT_SENSOR_READINGS", \
    3, \
    {  { "left_wheel_ticks", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_robot_sensor_readings_t, left_wheel_ticks) }, \
         { "right_wheel_ticks", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_robot_sensor_readings_t, right_wheel_ticks) }, \
         { "curr_heading", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_robot_sensor_readings_t, curr_heading) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROBOT_SENSOR_READINGS { \
    "ROBOT_SENSOR_READINGS", \
    3, \
    {  { "left_wheel_ticks", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_robot_sensor_readings_t, left_wheel_ticks) }, \
         { "right_wheel_ticks", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_robot_sensor_readings_t, right_wheel_ticks) }, \
         { "curr_heading", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_robot_sensor_readings_t, curr_heading) }, \
         } \
}
#endif

/**
 * @brief Pack a robot_sensor_readings message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param left_wheel_ticks  left wheel encoder ticks
 * @param right_wheel_ticks  right wheel encoder ticks
 * @param curr_heading  current heading of robot
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robot_sensor_readings_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t left_wheel_ticks, uint32_t right_wheel_ticks, float curr_heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN];
    _mav_put_uint32_t(buf, 0, left_wheel_ticks);
    _mav_put_uint32_t(buf, 4, right_wheel_ticks);
    _mav_put_float(buf, 8, curr_heading);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN);
#else
    mavlink_robot_sensor_readings_t packet;
    packet.left_wheel_ticks = left_wheel_ticks;
    packet.right_wheel_ticks = right_wheel_ticks;
    packet.curr_heading = curr_heading;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_CRC);
}

/**
 * @brief Pack a robot_sensor_readings message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param left_wheel_ticks  left wheel encoder ticks
 * @param right_wheel_ticks  right wheel encoder ticks
 * @param curr_heading  current heading of robot
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robot_sensor_readings_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t left_wheel_ticks,uint32_t right_wheel_ticks,float curr_heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN];
    _mav_put_uint32_t(buf, 0, left_wheel_ticks);
    _mav_put_uint32_t(buf, 4, right_wheel_ticks);
    _mav_put_float(buf, 8, curr_heading);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN);
#else
    mavlink_robot_sensor_readings_t packet;
    packet.left_wheel_ticks = left_wheel_ticks;
    packet.right_wheel_ticks = right_wheel_ticks;
    packet.curr_heading = curr_heading;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_CRC);
}

/**
 * @brief Encode a robot_sensor_readings struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param robot_sensor_readings C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robot_sensor_readings_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_robot_sensor_readings_t* robot_sensor_readings)
{
    return mavlink_msg_robot_sensor_readings_pack(system_id, component_id, msg, robot_sensor_readings->left_wheel_ticks, robot_sensor_readings->right_wheel_ticks, robot_sensor_readings->curr_heading);
}

/**
 * @brief Encode a robot_sensor_readings struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param robot_sensor_readings C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robot_sensor_readings_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_robot_sensor_readings_t* robot_sensor_readings)
{
    return mavlink_msg_robot_sensor_readings_pack_chan(system_id, component_id, chan, msg, robot_sensor_readings->left_wheel_ticks, robot_sensor_readings->right_wheel_ticks, robot_sensor_readings->curr_heading);
}

/**
 * @brief Send a robot_sensor_readings message
 * @param chan MAVLink channel to send the message
 *
 * @param left_wheel_ticks  left wheel encoder ticks
 * @param right_wheel_ticks  right wheel encoder ticks
 * @param curr_heading  current heading of robot
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_robot_sensor_readings_send(mavlink_channel_t chan, uint32_t left_wheel_ticks, uint32_t right_wheel_ticks, float curr_heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN];
    _mav_put_uint32_t(buf, 0, left_wheel_ticks);
    _mav_put_uint32_t(buf, 4, right_wheel_ticks);
    _mav_put_float(buf, 8, curr_heading);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS, buf, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_CRC);
#else
    mavlink_robot_sensor_readings_t packet;
    packet.left_wheel_ticks = left_wheel_ticks;
    packet.right_wheel_ticks = right_wheel_ticks;
    packet.curr_heading = curr_heading;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS, (const char *)&packet, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_CRC);
#endif
}

/**
 * @brief Send a robot_sensor_readings message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_robot_sensor_readings_send_struct(mavlink_channel_t chan, const mavlink_robot_sensor_readings_t* robot_sensor_readings)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_robot_sensor_readings_send(chan, robot_sensor_readings->left_wheel_ticks, robot_sensor_readings->right_wheel_ticks, robot_sensor_readings->curr_heading);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS, (const char *)robot_sensor_readings, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_robot_sensor_readings_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t left_wheel_ticks, uint32_t right_wheel_ticks, float curr_heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, left_wheel_ticks);
    _mav_put_uint32_t(buf, 4, right_wheel_ticks);
    _mav_put_float(buf, 8, curr_heading);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS, buf, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_CRC);
#else
    mavlink_robot_sensor_readings_t *packet = (mavlink_robot_sensor_readings_t *)msgbuf;
    packet->left_wheel_ticks = left_wheel_ticks;
    packet->right_wheel_ticks = right_wheel_ticks;
    packet->curr_heading = curr_heading;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS, (const char *)packet, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_CRC);
#endif
}
#endif

#endif

// MESSAGE ROBOT_SENSOR_READINGS UNPACKING


/**
 * @brief Get field left_wheel_ticks from robot_sensor_readings message
 *
 * @return  left wheel encoder ticks
 */
static inline uint32_t mavlink_msg_robot_sensor_readings_get_left_wheel_ticks(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field right_wheel_ticks from robot_sensor_readings message
 *
 * @return  right wheel encoder ticks
 */
static inline uint32_t mavlink_msg_robot_sensor_readings_get_right_wheel_ticks(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field curr_heading from robot_sensor_readings message
 *
 * @return  current heading of robot
 */
static inline float mavlink_msg_robot_sensor_readings_get_curr_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a robot_sensor_readings message into a struct
 *
 * @param msg The message to decode
 * @param robot_sensor_readings C-struct to decode the message contents into
 */
static inline void mavlink_msg_robot_sensor_readings_decode(const mavlink_message_t* msg, mavlink_robot_sensor_readings_t* robot_sensor_readings)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    robot_sensor_readings->left_wheel_ticks = mavlink_msg_robot_sensor_readings_get_left_wheel_ticks(msg);
    robot_sensor_readings->right_wheel_ticks = mavlink_msg_robot_sensor_readings_get_right_wheel_ticks(msg);
    robot_sensor_readings->curr_heading = mavlink_msg_robot_sensor_readings_get_curr_heading(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN? msg->len : MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN;
        memset(robot_sensor_readings, 0, MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_LEN);
    memcpy(robot_sensor_readings, _MAV_PAYLOAD(msg), len);
#endif
}

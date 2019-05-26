#pragma once
// MESSAGE ROBOT_POSITION_CHANGE PACKING

#define MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE 131

MAVPACKED(
typedef struct __mavlink_robot_position_change_t {
 float delta_x; /*<  change in x*/
 float delta_y; /*<  change in y*/
 float delta_theta; /*<  change in theta*/
}) mavlink_robot_position_change_t;

#define MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN 12
#define MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_MIN_LEN 12
#define MAVLINK_MSG_ID_131_LEN 12
#define MAVLINK_MSG_ID_131_MIN_LEN 12

#define MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_CRC 90
#define MAVLINK_MSG_ID_131_CRC 90



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROBOT_POSITION_CHANGE { \
    131, \
    "ROBOT_POSITION_CHANGE", \
    3, \
    {  { "delta_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_robot_position_change_t, delta_x) }, \
         { "delta_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_robot_position_change_t, delta_y) }, \
         { "delta_theta", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_robot_position_change_t, delta_theta) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROBOT_POSITION_CHANGE { \
    "ROBOT_POSITION_CHANGE", \
    3, \
    {  { "delta_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_robot_position_change_t, delta_x) }, \
         { "delta_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_robot_position_change_t, delta_y) }, \
         { "delta_theta", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_robot_position_change_t, delta_theta) }, \
         } \
}
#endif

/**
 * @brief Pack a robot_position_change message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param delta_x  change in x
 * @param delta_y  change in y
 * @param delta_theta  change in theta
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robot_position_change_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float delta_x, float delta_y, float delta_theta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN];
    _mav_put_float(buf, 0, delta_x);
    _mav_put_float(buf, 4, delta_y);
    _mav_put_float(buf, 8, delta_theta);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN);
#else
    mavlink_robot_position_change_t packet;
    packet.delta_x = delta_x;
    packet.delta_y = delta_y;
    packet.delta_theta = delta_theta;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_MIN_LEN, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_CRC);
}

/**
 * @brief Pack a robot_position_change message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param delta_x  change in x
 * @param delta_y  change in y
 * @param delta_theta  change in theta
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robot_position_change_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float delta_x,float delta_y,float delta_theta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN];
    _mav_put_float(buf, 0, delta_x);
    _mav_put_float(buf, 4, delta_y);
    _mav_put_float(buf, 8, delta_theta);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN);
#else
    mavlink_robot_position_change_t packet;
    packet.delta_x = delta_x;
    packet.delta_y = delta_y;
    packet.delta_theta = delta_theta;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_MIN_LEN, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_CRC);
}

/**
 * @brief Encode a robot_position_change struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param robot_position_change C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robot_position_change_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_robot_position_change_t* robot_position_change)
{
    return mavlink_msg_robot_position_change_pack(system_id, component_id, msg, robot_position_change->delta_x, robot_position_change->delta_y, robot_position_change->delta_theta);
}

/**
 * @brief Encode a robot_position_change struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param robot_position_change C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robot_position_change_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_robot_position_change_t* robot_position_change)
{
    return mavlink_msg_robot_position_change_pack_chan(system_id, component_id, chan, msg, robot_position_change->delta_x, robot_position_change->delta_y, robot_position_change->delta_theta);
}

/**
 * @brief Send a robot_position_change message
 * @param chan MAVLink channel to send the message
 *
 * @param delta_x  change in x
 * @param delta_y  change in y
 * @param delta_theta  change in theta
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_robot_position_change_send(mavlink_channel_t chan, float delta_x, float delta_y, float delta_theta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN];
    _mav_put_float(buf, 0, delta_x);
    _mav_put_float(buf, 4, delta_y);
    _mav_put_float(buf, 8, delta_theta);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE, buf, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_MIN_LEN, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_CRC);
#else
    mavlink_robot_position_change_t packet;
    packet.delta_x = delta_x;
    packet.delta_y = delta_y;
    packet.delta_theta = delta_theta;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE, (const char *)&packet, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_MIN_LEN, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_CRC);
#endif
}

/**
 * @brief Send a robot_position_change message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_robot_position_change_send_struct(mavlink_channel_t chan, const mavlink_robot_position_change_t* robot_position_change)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_robot_position_change_send(chan, robot_position_change->delta_x, robot_position_change->delta_y, robot_position_change->delta_theta);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE, (const char *)robot_position_change, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_MIN_LEN, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_robot_position_change_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float delta_x, float delta_y, float delta_theta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, delta_x);
    _mav_put_float(buf, 4, delta_y);
    _mav_put_float(buf, 8, delta_theta);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE, buf, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_MIN_LEN, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_CRC);
#else
    mavlink_robot_position_change_t *packet = (mavlink_robot_position_change_t *)msgbuf;
    packet->delta_x = delta_x;
    packet->delta_y = delta_y;
    packet->delta_theta = delta_theta;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE, (const char *)packet, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_MIN_LEN, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_CRC);
#endif
}
#endif

#endif

// MESSAGE ROBOT_POSITION_CHANGE UNPACKING


/**
 * @brief Get field delta_x from robot_position_change message
 *
 * @return  change in x
 */
static inline float mavlink_msg_robot_position_change_get_delta_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field delta_y from robot_position_change message
 *
 * @return  change in y
 */
static inline float mavlink_msg_robot_position_change_get_delta_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field delta_theta from robot_position_change message
 *
 * @return  change in theta
 */
static inline float mavlink_msg_robot_position_change_get_delta_theta(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a robot_position_change message into a struct
 *
 * @param msg The message to decode
 * @param robot_position_change C-struct to decode the message contents into
 */
static inline void mavlink_msg_robot_position_change_decode(const mavlink_message_t* msg, mavlink_robot_position_change_t* robot_position_change)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    robot_position_change->delta_x = mavlink_msg_robot_position_change_get_delta_x(msg);
    robot_position_change->delta_y = mavlink_msg_robot_position_change_get_delta_y(msg);
    robot_position_change->delta_theta = mavlink_msg_robot_position_change_get_delta_theta(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN? msg->len : MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN;
        memset(robot_position_change, 0, MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_LEN);
    memcpy(robot_position_change, _MAV_PAYLOAD(msg), len);
#endif
}

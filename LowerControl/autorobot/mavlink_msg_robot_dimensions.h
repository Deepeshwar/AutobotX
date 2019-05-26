#pragma once
// MESSAGE ROBOT_DIMENSIONS PACKING

#define MAVLINK_MSG_ID_ROBOT_DIMENSIONS 3

MAVPACKED(
typedef struct __mavlink_robot_dimensions_t {
 float wheel_radius; /*<  Raduis of wheels*/
 float distance_bw_wheels; /*<  Distance between wheels*/
}) mavlink_robot_dimensions_t;

#define MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN 8
#define MAVLINK_MSG_ID_ROBOT_DIMENSIONS_MIN_LEN 8
#define MAVLINK_MSG_ID_3_LEN 8
#define MAVLINK_MSG_ID_3_MIN_LEN 8

#define MAVLINK_MSG_ID_ROBOT_DIMENSIONS_CRC 107
#define MAVLINK_MSG_ID_3_CRC 107



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROBOT_DIMENSIONS { \
    3, \
    "ROBOT_DIMENSIONS", \
    2, \
    {  { "wheel_radius", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_robot_dimensions_t, wheel_radius) }, \
         { "distance_bw_wheels", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_robot_dimensions_t, distance_bw_wheels) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROBOT_DIMENSIONS { \
    "ROBOT_DIMENSIONS", \
    2, \
    {  { "wheel_radius", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_robot_dimensions_t, wheel_radius) }, \
         { "distance_bw_wheels", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_robot_dimensions_t, distance_bw_wheels) }, \
         } \
}
#endif

/**
 * @brief Pack a robot_dimensions message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param wheel_radius  Raduis of wheels
 * @param distance_bw_wheels  Distance between wheels
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robot_dimensions_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float wheel_radius, float distance_bw_wheels)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN];
    _mav_put_float(buf, 0, wheel_radius);
    _mav_put_float(buf, 4, distance_bw_wheels);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN);
#else
    mavlink_robot_dimensions_t packet;
    packet.wheel_radius = wheel_radius;
    packet.distance_bw_wheels = distance_bw_wheels;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOT_DIMENSIONS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_CRC);
}

/**
 * @brief Pack a robot_dimensions message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param wheel_radius  Raduis of wheels
 * @param distance_bw_wheels  Distance between wheels
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robot_dimensions_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float wheel_radius,float distance_bw_wheels)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN];
    _mav_put_float(buf, 0, wheel_radius);
    _mav_put_float(buf, 4, distance_bw_wheels);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN);
#else
    mavlink_robot_dimensions_t packet;
    packet.wheel_radius = wheel_radius;
    packet.distance_bw_wheels = distance_bw_wheels;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOT_DIMENSIONS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_CRC);
}

/**
 * @brief Encode a robot_dimensions struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param robot_dimensions C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robot_dimensions_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_robot_dimensions_t* robot_dimensions)
{
    return mavlink_msg_robot_dimensions_pack(system_id, component_id, msg, robot_dimensions->wheel_radius, robot_dimensions->distance_bw_wheels);
}

/**
 * @brief Encode a robot_dimensions struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param robot_dimensions C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robot_dimensions_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_robot_dimensions_t* robot_dimensions)
{
    return mavlink_msg_robot_dimensions_pack_chan(system_id, component_id, chan, msg, robot_dimensions->wheel_radius, robot_dimensions->distance_bw_wheels);
}

/**
 * @brief Send a robot_dimensions message
 * @param chan MAVLink channel to send the message
 *
 * @param wheel_radius  Raduis of wheels
 * @param distance_bw_wheels  Distance between wheels
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_robot_dimensions_send(mavlink_channel_t chan, float wheel_radius, float distance_bw_wheels)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN];
    _mav_put_float(buf, 0, wheel_radius);
    _mav_put_float(buf, 4, distance_bw_wheels);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_DIMENSIONS, buf, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_CRC);
#else
    mavlink_robot_dimensions_t packet;
    packet.wheel_radius = wheel_radius;
    packet.distance_bw_wheels = distance_bw_wheels;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_DIMENSIONS, (const char *)&packet, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_CRC);
#endif
}

/**
 * @brief Send a robot_dimensions message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_robot_dimensions_send_struct(mavlink_channel_t chan, const mavlink_robot_dimensions_t* robot_dimensions)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_robot_dimensions_send(chan, robot_dimensions->wheel_radius, robot_dimensions->distance_bw_wheels);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_DIMENSIONS, (const char *)robot_dimensions, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_robot_dimensions_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float wheel_radius, float distance_bw_wheels)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, wheel_radius);
    _mav_put_float(buf, 4, distance_bw_wheels);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_DIMENSIONS, buf, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_CRC);
#else
    mavlink_robot_dimensions_t *packet = (mavlink_robot_dimensions_t *)msgbuf;
    packet->wheel_radius = wheel_radius;
    packet->distance_bw_wheels = distance_bw_wheels;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_DIMENSIONS, (const char *)packet, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_CRC);
#endif
}
#endif

#endif

// MESSAGE ROBOT_DIMENSIONS UNPACKING


/**
 * @brief Get field wheel_radius from robot_dimensions message
 *
 * @return  Raduis of wheels
 */
static inline float mavlink_msg_robot_dimensions_get_wheel_radius(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field distance_bw_wheels from robot_dimensions message
 *
 * @return  Distance between wheels
 */
static inline float mavlink_msg_robot_dimensions_get_distance_bw_wheels(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a robot_dimensions message into a struct
 *
 * @param msg The message to decode
 * @param robot_dimensions C-struct to decode the message contents into
 */
static inline void mavlink_msg_robot_dimensions_decode(const mavlink_message_t* msg, mavlink_robot_dimensions_t* robot_dimensions)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    robot_dimensions->wheel_radius = mavlink_msg_robot_dimensions_get_wheel_radius(msg);
    robot_dimensions->distance_bw_wheels = mavlink_msg_robot_dimensions_get_distance_bw_wheels(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN? msg->len : MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN;
        memset(robot_dimensions, 0, MAVLINK_MSG_ID_ROBOT_DIMENSIONS_LEN);
    memcpy(robot_dimensions, _MAV_PAYLOAD(msg), len);
#endif
}

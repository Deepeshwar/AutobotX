#pragma once
// MESSAGE DESIRE_WHEEL_RPM PACKING

#define MAVLINK_MSG_ID_DESIRE_WHEEL_RPM 31

MAVPACKED(
typedef struct __mavlink_desire_wheel_rpm_t {
 float left_wheel_rpm; /*<  Desire rpm of left wheel*/
 float right_wheel_rpm; /*<  Desire rpm of right wheel*/
}) mavlink_desire_wheel_rpm_t;

#define MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN 8
#define MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_MIN_LEN 8
#define MAVLINK_MSG_ID_31_LEN 8
#define MAVLINK_MSG_ID_31_MIN_LEN 8

#define MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_CRC 154
#define MAVLINK_MSG_ID_31_CRC 154



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DESIRE_WHEEL_RPM { \
    31, \
    "DESIRE_WHEEL_RPM", \
    2, \
    {  { "left_wheel_rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_desire_wheel_rpm_t, left_wheel_rpm) }, \
         { "right_wheel_rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_desire_wheel_rpm_t, right_wheel_rpm) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DESIRE_WHEEL_RPM { \
    "DESIRE_WHEEL_RPM", \
    2, \
    {  { "left_wheel_rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_desire_wheel_rpm_t, left_wheel_rpm) }, \
         { "right_wheel_rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_desire_wheel_rpm_t, right_wheel_rpm) }, \
         } \
}
#endif

/**
 * @brief Pack a desire_wheel_rpm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param left_wheel_rpm  Desire rpm of left wheel
 * @param right_wheel_rpm  Desire rpm of right wheel
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_desire_wheel_rpm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float left_wheel_rpm, float right_wheel_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN];
    _mav_put_float(buf, 0, left_wheel_rpm);
    _mav_put_float(buf, 4, right_wheel_rpm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN);
#else
    mavlink_desire_wheel_rpm_t packet;
    packet.left_wheel_rpm = left_wheel_rpm;
    packet.right_wheel_rpm = right_wheel_rpm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DESIRE_WHEEL_RPM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_MIN_LEN, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_CRC);
}

/**
 * @brief Pack a desire_wheel_rpm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param left_wheel_rpm  Desire rpm of left wheel
 * @param right_wheel_rpm  Desire rpm of right wheel
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_desire_wheel_rpm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float left_wheel_rpm,float right_wheel_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN];
    _mav_put_float(buf, 0, left_wheel_rpm);
    _mav_put_float(buf, 4, right_wheel_rpm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN);
#else
    mavlink_desire_wheel_rpm_t packet;
    packet.left_wheel_rpm = left_wheel_rpm;
    packet.right_wheel_rpm = right_wheel_rpm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DESIRE_WHEEL_RPM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_MIN_LEN, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_CRC);
}

/**
 * @brief Encode a desire_wheel_rpm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param desire_wheel_rpm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_desire_wheel_rpm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_desire_wheel_rpm_t* desire_wheel_rpm)
{
    return mavlink_msg_desire_wheel_rpm_pack(system_id, component_id, msg, desire_wheel_rpm->left_wheel_rpm, desire_wheel_rpm->right_wheel_rpm);
}

/**
 * @brief Encode a desire_wheel_rpm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param desire_wheel_rpm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_desire_wheel_rpm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_desire_wheel_rpm_t* desire_wheel_rpm)
{
    return mavlink_msg_desire_wheel_rpm_pack_chan(system_id, component_id, chan, msg, desire_wheel_rpm->left_wheel_rpm, desire_wheel_rpm->right_wheel_rpm);
}

/**
 * @brief Send a desire_wheel_rpm message
 * @param chan MAVLink channel to send the message
 *
 * @param left_wheel_rpm  Desire rpm of left wheel
 * @param right_wheel_rpm  Desire rpm of right wheel
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_desire_wheel_rpm_send(mavlink_channel_t chan, float left_wheel_rpm, float right_wheel_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN];
    _mav_put_float(buf, 0, left_wheel_rpm);
    _mav_put_float(buf, 4, right_wheel_rpm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM, buf, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_MIN_LEN, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_CRC);
#else
    mavlink_desire_wheel_rpm_t packet;
    packet.left_wheel_rpm = left_wheel_rpm;
    packet.right_wheel_rpm = right_wheel_rpm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM, (const char *)&packet, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_MIN_LEN, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_CRC);
#endif
}

/**
 * @brief Send a desire_wheel_rpm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_desire_wheel_rpm_send_struct(mavlink_channel_t chan, const mavlink_desire_wheel_rpm_t* desire_wheel_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_desire_wheel_rpm_send(chan, desire_wheel_rpm->left_wheel_rpm, desire_wheel_rpm->right_wheel_rpm);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM, (const char *)desire_wheel_rpm, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_MIN_LEN, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_CRC);
#endif
}

#if MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_desire_wheel_rpm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float left_wheel_rpm, float right_wheel_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, left_wheel_rpm);
    _mav_put_float(buf, 4, right_wheel_rpm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM, buf, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_MIN_LEN, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_CRC);
#else
    mavlink_desire_wheel_rpm_t *packet = (mavlink_desire_wheel_rpm_t *)msgbuf;
    packet->left_wheel_rpm = left_wheel_rpm;
    packet->right_wheel_rpm = right_wheel_rpm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM, (const char *)packet, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_MIN_LEN, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_CRC);
#endif
}
#endif

#endif

// MESSAGE DESIRE_WHEEL_RPM UNPACKING


/**
 * @brief Get field left_wheel_rpm from desire_wheel_rpm message
 *
 * @return  Desire rpm of left wheel
 */
static inline float mavlink_msg_desire_wheel_rpm_get_left_wheel_rpm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field right_wheel_rpm from desire_wheel_rpm message
 *
 * @return  Desire rpm of right wheel
 */
static inline float mavlink_msg_desire_wheel_rpm_get_right_wheel_rpm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a desire_wheel_rpm message into a struct
 *
 * @param msg The message to decode
 * @param desire_wheel_rpm C-struct to decode the message contents into
 */
static inline void mavlink_msg_desire_wheel_rpm_decode(const mavlink_message_t* msg, mavlink_desire_wheel_rpm_t* desire_wheel_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    desire_wheel_rpm->left_wheel_rpm = mavlink_msg_desire_wheel_rpm_get_left_wheel_rpm(msg);
    desire_wheel_rpm->right_wheel_rpm = mavlink_msg_desire_wheel_rpm_get_right_wheel_rpm(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN? msg->len : MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN;
        memset(desire_wheel_rpm, 0, MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_LEN);
    memcpy(desire_wheel_rpm, _MAV_PAYLOAD(msg), len);
#endif
}

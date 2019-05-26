#pragma once
// MESSAGE DESIRE_CMD_VAL PACKING
#include <stdio.h>
#define MAVLINK_MSG_ID_DESIRE_CMD_VAL 32

MAVPACKED(
typedef struct __mavlink_desire_cmd_val_t {
 float v; /*<  linear velocity*/
 float w; /*<  angular velocity*/
}) mavlink_desire_cmd_val_t;

#define MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN 8
#define MAVLINK_MSG_ID_DESIRE_CMD_VAL_MIN_LEN 8
#define MAVLINK_MSG_ID_32_LEN 8
#define MAVLINK_MSG_ID_32_MIN_LEN 8

#define MAVLINK_MSG_ID_DESIRE_CMD_VAL_CRC 154
#define MAVLINK_MSG_ID_32_CRC 154



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DESIRE_CMD_VAL { \
    32, \
    "DESIRE_CMD_VAL", \
    2, \
    {  { "v", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_desire_cmd_val_t, v) }, \
         { "w", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_desire_cmd_val_t, w) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DESIRE_CMD_VAL { \
    "DESIRE_CMD_VAL", \
    2, \
    {  { "v", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_desire_cmd_val_t, v) }, \
         { "w", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_desire_cmd_val_t, w) }, \
         } \
}
#endif

/**
 * @brief Pack a desire_cmd_val message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param v  linear velocity
 * @param w  angular velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_desire_cmd_val_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float v, float w)
{


#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN];
    _mav_put_float(buf, 0, v);
    _mav_put_float(buf, 4, w);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN);
#else
    mavlink_desire_cmd_val_t packet;
    packet.v = v;
    packet.w = w;


        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DESIRE_CMD_VAL;
  
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DESIRE_CMD_VAL_MIN_LEN, MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN, MAVLINK_MSG_ID_DESIRE_CMD_VAL_CRC);
}

/**
 * @brief Pack a desire_cmd_val message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param v  linear velocity
 * @param w  angular velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_desire_cmd_val_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float v,float w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN];
    _mav_put_float(buf, 0, v);
    _mav_put_float(buf, 4, w);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN);
#else
    mavlink_desire_cmd_val_t packet;
    packet.v = v;
    packet.w = w;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DESIRE_CMD_VAL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DESIRE_CMD_VAL_MIN_LEN, MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN, MAVLINK_MSG_ID_DESIRE_CMD_VAL_CRC);
}

/**
 * @brief Encode a desire_cmd_val struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param desire_cmd_val C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_desire_cmd_val_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_desire_cmd_val_t* desire_cmd_val)
{
    return mavlink_msg_desire_cmd_val_pack(system_id, component_id, msg, desire_cmd_val->v, desire_cmd_val->w);
}

/**
 * @brief Encode a desire_cmd_val struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param desire_cmd_val C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_desire_cmd_val_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_desire_cmd_val_t* desire_cmd_val)
{
    return mavlink_msg_desire_cmd_val_pack_chan(system_id, component_id, chan, msg, desire_cmd_val->v, desire_cmd_val->w);
}

/**
 * @brief Send a desire_cmd_val message
 * @param chan MAVLink channel to send the message
 *
 * @param v  linear velocity
 * @param w  angular velocity
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_desire_cmd_val_send(mavlink_channel_t chan, float v, float w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN];
    _mav_put_float(buf, 0, v);
    _mav_put_float(buf, 4, w);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DESIRE_CMD_VAL, buf, MAVLINK_MSG_ID_DESIRE_CMD_VAL_MIN_LEN, MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN, MAVLINK_MSG_ID_DESIRE_CMD_VAL_CRC);
#else
    mavlink_desire_cmd_val_t packet;
    packet.v = v;
    packet.w = w;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DESIRE_CMD_VAL, (const char *)&packet, MAVLINK_MSG_ID_DESIRE_CMD_VAL_MIN_LEN, MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN, MAVLINK_MSG_ID_DESIRE_CMD_VAL_CRC);
#endif
}

/**
 * @brief Send a desire_cmd_val message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_desire_cmd_val_send_struct(mavlink_channel_t chan, const mavlink_desire_cmd_val_t* desire_cmd_val)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_desire_cmd_val_send(chan, desire_cmd_val->v, desire_cmd_val->w);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DESIRE_CMD_VAL, (const char *)desire_cmd_val, MAVLINK_MSG_ID_DESIRE_CMD_VAL_MIN_LEN, MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN, MAVLINK_MSG_ID_DESIRE_CMD_VAL_CRC);
#endif
}

#if MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_desire_cmd_val_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float v, float w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, v);
    _mav_put_float(buf, 4, w);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DESIRE_CMD_VAL, buf, MAVLINK_MSG_ID_DESIRE_CMD_VAL_MIN_LEN, MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN, MAVLINK_MSG_ID_DESIRE_CMD_VAL_CRC);
#else
    mavlink_desire_cmd_val_t *packet = (mavlink_desire_cmd_val_t *)msgbuf;
    packet->v = v;
    packet->w = w;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DESIRE_CMD_VAL, (const char *)packet, MAVLINK_MSG_ID_DESIRE_CMD_VAL_MIN_LEN, MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN, MAVLINK_MSG_ID_DESIRE_CMD_VAL_CRC);
#endif
}
#endif

#endif

// MESSAGE DESIRE_CMD_VAL UNPACKING


/**
 * @brief Get field v from desire_cmd_val message
 *
 * @return  linear velocity
 */
static inline float mavlink_msg_desire_cmd_val_get_v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field w from desire_cmd_val message
 *
 * @return  angular velocity
 */
static inline float mavlink_msg_desire_cmd_val_get_w(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a desire_cmd_val message into a struct
 *
 * @param msg The message to decode
 * @param desire_cmd_val C-struct to decode the message contents into
 */
static inline void mavlink_msg_desire_cmd_val_decode(const mavlink_message_t* msg, mavlink_desire_cmd_val_t* desire_cmd_val)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    desire_cmd_val->v = mavlink_msg_desire_cmd_val_get_v(msg);
    desire_cmd_val->w = mavlink_msg_desire_cmd_val_get_w(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN? msg->len : MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN;
        memset(desire_cmd_val, 0, MAVLINK_MSG_ID_DESIRE_CMD_VAL_LEN);
    memcpy(desire_cmd_val, _MAV_PAYLOAD(msg), len);
#endif
}

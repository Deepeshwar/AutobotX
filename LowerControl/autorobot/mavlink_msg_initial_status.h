#pragma once
// MESSAGE INITIAL_STATUS PACKING

#define MAVLINK_MSG_ID_INITIAL_STATUS 101

MAVPACKED(
typedef struct __mavlink_initial_status_t {
 float initial_heading; /*<  Initial heading of robot*/
 uint16_t update_freq; /*<  Freuency at which data is sent to Ros*/
 uint16_t encoder_ppr; /*<  PPR of encoders*/
 uint8_t ultrasonic_config; /*<  Configuration of ultrasonic sensors*/
}) mavlink_initial_status_t;

#define MAVLINK_MSG_ID_INITIAL_STATUS_LEN 9
#define MAVLINK_MSG_ID_INITIAL_STATUS_MIN_LEN 9
#define MAVLINK_MSG_ID_101_LEN 9
#define MAVLINK_MSG_ID_101_MIN_LEN 9

#define MAVLINK_MSG_ID_INITIAL_STATUS_CRC 35
#define MAVLINK_MSG_ID_101_CRC 35



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_INITIAL_STATUS { \
    101, \
    "INITIAL_STATUS", \
    4, \
    {  { "initial_heading", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_initial_status_t, initial_heading) }, \
         { "ultrasonic_config", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_initial_status_t, ultrasonic_config) }, \
         { "update_freq", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_initial_status_t, update_freq) }, \
         { "encoder_ppr", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_initial_status_t, encoder_ppr) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_INITIAL_STATUS { \
    "INITIAL_STATUS", \
    4, \
    {  { "initial_heading", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_initial_status_t, initial_heading) }, \
         { "ultrasonic_config", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_initial_status_t, ultrasonic_config) }, \
         { "update_freq", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_initial_status_t, update_freq) }, \
         { "encoder_ppr", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_initial_status_t, encoder_ppr) }, \
         } \
}
#endif

/**
 * @brief Pack a initial_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param initial_heading  Initial heading of robot
 * @param ultrasonic_config  Configuration of ultrasonic sensors
 * @param update_freq  Freuency at which data is sent to Ros
 * @param encoder_ppr  PPR of encoders
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_initial_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float initial_heading, uint8_t ultrasonic_config, uint16_t update_freq, uint16_t encoder_ppr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INITIAL_STATUS_LEN];
    _mav_put_float(buf, 0, initial_heading);
    _mav_put_uint16_t(buf, 4, update_freq);
    _mav_put_uint16_t(buf, 6, encoder_ppr);
    _mav_put_uint8_t(buf, 8, ultrasonic_config);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INITIAL_STATUS_LEN);
#else
    mavlink_initial_status_t packet;
    packet.initial_heading = initial_heading;
    packet.update_freq = update_freq;
    packet.encoder_ppr = encoder_ppr;
    packet.ultrasonic_config = ultrasonic_config;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INITIAL_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INITIAL_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_INITIAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_INITIAL_STATUS_LEN, MAVLINK_MSG_ID_INITIAL_STATUS_CRC);
}

/**
 * @brief Pack a initial_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param initial_heading  Initial heading of robot
 * @param ultrasonic_config  Configuration of ultrasonic sensors
 * @param update_freq  Freuency at which data is sent to Ros
 * @param encoder_ppr  PPR of encoders
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_initial_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float initial_heading,uint8_t ultrasonic_config,uint16_t update_freq,uint16_t encoder_ppr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INITIAL_STATUS_LEN];
    _mav_put_float(buf, 0, initial_heading);
    _mav_put_uint16_t(buf, 4, update_freq);
    _mav_put_uint16_t(buf, 6, encoder_ppr);
    _mav_put_uint8_t(buf, 8, ultrasonic_config);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INITIAL_STATUS_LEN);
#else
    mavlink_initial_status_t packet;
    packet.initial_heading = initial_heading;
    packet.update_freq = update_freq;
    packet.encoder_ppr = encoder_ppr;
    packet.ultrasonic_config = ultrasonic_config;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INITIAL_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INITIAL_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_INITIAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_INITIAL_STATUS_LEN, MAVLINK_MSG_ID_INITIAL_STATUS_CRC);
}

/**
 * @brief Encode a initial_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param initial_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_initial_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_initial_status_t* initial_status)
{
    return mavlink_msg_initial_status_pack(system_id, component_id, msg, initial_status->initial_heading, initial_status->ultrasonic_config, initial_status->update_freq, initial_status->encoder_ppr);
}

/**
 * @brief Encode a initial_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param initial_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_initial_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_initial_status_t* initial_status)
{
    return mavlink_msg_initial_status_pack_chan(system_id, component_id, chan, msg, initial_status->initial_heading, initial_status->ultrasonic_config, initial_status->update_freq, initial_status->encoder_ppr);
}

/**
 * @brief Send a initial_status message
 * @param chan MAVLink channel to send the message
 *
 * @param initial_heading  Initial heading of robot
 * @param ultrasonic_config  Configuration of ultrasonic sensors
 * @param update_freq  Freuency at which data is sent to Ros
 * @param encoder_ppr  PPR of encoders
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_initial_status_send(mavlink_channel_t chan, float initial_heading, uint8_t ultrasonic_config, uint16_t update_freq, uint16_t encoder_ppr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INITIAL_STATUS_LEN];
    _mav_put_float(buf, 0, initial_heading);
    _mav_put_uint16_t(buf, 4, update_freq);
    _mav_put_uint16_t(buf, 6, encoder_ppr);
    _mav_put_uint8_t(buf, 8, ultrasonic_config);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INITIAL_STATUS, buf, MAVLINK_MSG_ID_INITIAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_INITIAL_STATUS_LEN, MAVLINK_MSG_ID_INITIAL_STATUS_CRC);
#else
    mavlink_initial_status_t packet;
    packet.initial_heading = initial_heading;
    packet.update_freq = update_freq;
    packet.encoder_ppr = encoder_ppr;
    packet.ultrasonic_config = ultrasonic_config;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INITIAL_STATUS, (const char *)&packet, MAVLINK_MSG_ID_INITIAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_INITIAL_STATUS_LEN, MAVLINK_MSG_ID_INITIAL_STATUS_CRC);
#endif
}

/**
 * @brief Send a initial_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_initial_status_send_struct(mavlink_channel_t chan, const mavlink_initial_status_t* initial_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_initial_status_send(chan, initial_status->initial_heading, initial_status->ultrasonic_config, initial_status->update_freq, initial_status->encoder_ppr);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INITIAL_STATUS, (const char *)initial_status, MAVLINK_MSG_ID_INITIAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_INITIAL_STATUS_LEN, MAVLINK_MSG_ID_INITIAL_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_INITIAL_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_initial_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float initial_heading, uint8_t ultrasonic_config, uint16_t update_freq, uint16_t encoder_ppr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, initial_heading);
    _mav_put_uint16_t(buf, 4, update_freq);
    _mav_put_uint16_t(buf, 6, encoder_ppr);
    _mav_put_uint8_t(buf, 8, ultrasonic_config);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INITIAL_STATUS, buf, MAVLINK_MSG_ID_INITIAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_INITIAL_STATUS_LEN, MAVLINK_MSG_ID_INITIAL_STATUS_CRC);
#else
    mavlink_initial_status_t *packet = (mavlink_initial_status_t *)msgbuf;
    packet->initial_heading = initial_heading;
    packet->update_freq = update_freq;
    packet->encoder_ppr = encoder_ppr;
    packet->ultrasonic_config = ultrasonic_config;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INITIAL_STATUS, (const char *)packet, MAVLINK_MSG_ID_INITIAL_STATUS_MIN_LEN, MAVLINK_MSG_ID_INITIAL_STATUS_LEN, MAVLINK_MSG_ID_INITIAL_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE INITIAL_STATUS UNPACKING


/**
 * @brief Get field initial_heading from initial_status message
 *
 * @return  Initial heading of robot
 */
static inline float mavlink_msg_initial_status_get_initial_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ultrasonic_config from initial_status message
 *
 * @return  Configuration of ultrasonic sensors
 */
static inline uint8_t mavlink_msg_initial_status_get_ultrasonic_config(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field update_freq from initial_status message
 *
 * @return  Freuency at which data is sent to Ros
 */
static inline uint16_t mavlink_msg_initial_status_get_update_freq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field encoder_ppr from initial_status message
 *
 * @return  PPR of encoders
 */
static inline uint16_t mavlink_msg_initial_status_get_encoder_ppr(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Decode a initial_status message into a struct
 *
 * @param msg The message to decode
 * @param initial_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_initial_status_decode(const mavlink_message_t* msg, mavlink_initial_status_t* initial_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    initial_status->initial_heading = mavlink_msg_initial_status_get_initial_heading(msg);
    initial_status->update_freq = mavlink_msg_initial_status_get_update_freq(msg);
    initial_status->encoder_ppr = mavlink_msg_initial_status_get_encoder_ppr(msg);
    initial_status->ultrasonic_config = mavlink_msg_initial_status_get_ultrasonic_config(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_INITIAL_STATUS_LEN? msg->len : MAVLINK_MSG_ID_INITIAL_STATUS_LEN;
        memset(initial_status, 0, MAVLINK_MSG_ID_INITIAL_STATUS_LEN);
    memcpy(initial_status, _MAV_PAYLOAD(msg), len);
#endif
}

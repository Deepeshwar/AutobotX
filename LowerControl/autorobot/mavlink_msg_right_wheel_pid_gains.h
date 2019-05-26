#pragma once
// MESSAGE RIGHT_WHEEL_PID_GAINS PACKING

#define MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS 2

MAVPACKED(
typedef struct __mavlink_right_wheel_pid_gains_t {
 float Kp; /*<  Proportional Gain*/
 float Ki; /*<  Integral Gain*/
 float Kd; /*<  Derivative Gain*/
}) mavlink_right_wheel_pid_gains_t;

#define MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN 12
#define MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_MIN_LEN 12
#define MAVLINK_MSG_ID_2_LEN 12
#define MAVLINK_MSG_ID_2_MIN_LEN 12

#define MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_CRC 87
#define MAVLINK_MSG_ID_2_CRC 87



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RIGHT_WHEEL_PID_GAINS { \
    2, \
    "RIGHT_WHEEL_PID_GAINS", \
    3, \
    {  { "Kp", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_right_wheel_pid_gains_t, Kp) }, \
         { "Ki", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_right_wheel_pid_gains_t, Ki) }, \
         { "Kd", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_right_wheel_pid_gains_t, Kd) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RIGHT_WHEEL_PID_GAINS { \
    "RIGHT_WHEEL_PID_GAINS", \
    3, \
    {  { "Kp", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_right_wheel_pid_gains_t, Kp) }, \
         { "Ki", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_right_wheel_pid_gains_t, Ki) }, \
         { "Kd", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_right_wheel_pid_gains_t, Kd) }, \
         } \
}
#endif

/**
 * @brief Pack a right_wheel_pid_gains message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Kp  Proportional Gain
 * @param Ki  Integral Gain
 * @param Kd  Derivative Gain
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_right_wheel_pid_gains_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float Kp, float Ki, float Kd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN];
    _mav_put_float(buf, 0, Kp);
    _mav_put_float(buf, 4, Ki);
    _mav_put_float(buf, 8, Kd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN);
#else
    mavlink_right_wheel_pid_gains_t packet;
    packet.Kp = Kp;
    packet.Ki = Ki;
    packet.Kd = Kd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_MIN_LEN, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_CRC);
}

/**
 * @brief Pack a right_wheel_pid_gains message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param Kp  Proportional Gain
 * @param Ki  Integral Gain
 * @param Kd  Derivative Gain
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_right_wheel_pid_gains_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float Kp,float Ki,float Kd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN];
    _mav_put_float(buf, 0, Kp);
    _mav_put_float(buf, 4, Ki);
    _mav_put_float(buf, 8, Kd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN);
#else
    mavlink_right_wheel_pid_gains_t packet;
    packet.Kp = Kp;
    packet.Ki = Ki;
    packet.Kd = Kd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_MIN_LEN, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_CRC);
}

/**
 * @brief Encode a right_wheel_pid_gains struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param right_wheel_pid_gains C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_right_wheel_pid_gains_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_right_wheel_pid_gains_t* right_wheel_pid_gains)
{
    return mavlink_msg_right_wheel_pid_gains_pack(system_id, component_id, msg, right_wheel_pid_gains->Kp, right_wheel_pid_gains->Ki, right_wheel_pid_gains->Kd);
}

/**
 * @brief Encode a right_wheel_pid_gains struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param right_wheel_pid_gains C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_right_wheel_pid_gains_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_right_wheel_pid_gains_t* right_wheel_pid_gains)
{
    return mavlink_msg_right_wheel_pid_gains_pack_chan(system_id, component_id, chan, msg, right_wheel_pid_gains->Kp, right_wheel_pid_gains->Ki, right_wheel_pid_gains->Kd);
}

/**
 * @brief Send a right_wheel_pid_gains message
 * @param chan MAVLink channel to send the message
 *
 * @param Kp  Proportional Gain
 * @param Ki  Integral Gain
 * @param Kd  Derivative Gain
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_right_wheel_pid_gains_send(mavlink_channel_t chan, float Kp, float Ki, float Kd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN];
    _mav_put_float(buf, 0, Kp);
    _mav_put_float(buf, 4, Ki);
    _mav_put_float(buf, 8, Kd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS, buf, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_MIN_LEN, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_CRC);
#else
    mavlink_right_wheel_pid_gains_t packet;
    packet.Kp = Kp;
    packet.Ki = Ki;
    packet.Kd = Kd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS, (const char *)&packet, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_MIN_LEN, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_CRC);
#endif
}

/**
 * @brief Send a right_wheel_pid_gains message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_right_wheel_pid_gains_send_struct(mavlink_channel_t chan, const mavlink_right_wheel_pid_gains_t* right_wheel_pid_gains)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_right_wheel_pid_gains_send(chan, right_wheel_pid_gains->Kp, right_wheel_pid_gains->Ki, right_wheel_pid_gains->Kd);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS, (const char *)right_wheel_pid_gains, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_MIN_LEN, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_CRC);
#endif
}

#if MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_right_wheel_pid_gains_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float Kp, float Ki, float Kd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, Kp);
    _mav_put_float(buf, 4, Ki);
    _mav_put_float(buf, 8, Kd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS, buf, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_MIN_LEN, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_CRC);
#else
    mavlink_right_wheel_pid_gains_t *packet = (mavlink_right_wheel_pid_gains_t *)msgbuf;
    packet->Kp = Kp;
    packet->Ki = Ki;
    packet->Kd = Kd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS, (const char *)packet, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_MIN_LEN, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_CRC);
#endif
}
#endif

#endif

// MESSAGE RIGHT_WHEEL_PID_GAINS UNPACKING


/**
 * @brief Get field Kp from right_wheel_pid_gains message
 *
 * @return  Proportional Gain
 */
static inline float mavlink_msg_right_wheel_pid_gains_get_Kp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field Ki from right_wheel_pid_gains message
 *
 * @return  Integral Gain
 */
static inline float mavlink_msg_right_wheel_pid_gains_get_Ki(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field Kd from right_wheel_pid_gains message
 *
 * @return  Derivative Gain
 */
static inline float mavlink_msg_right_wheel_pid_gains_get_Kd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a right_wheel_pid_gains message into a struct
 *
 * @param msg The message to decode
 * @param right_wheel_pid_gains C-struct to decode the message contents into
 */
static inline void mavlink_msg_right_wheel_pid_gains_decode(const mavlink_message_t* msg, mavlink_right_wheel_pid_gains_t* right_wheel_pid_gains)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    right_wheel_pid_gains->Kp = mavlink_msg_right_wheel_pid_gains_get_Kp(msg);
    right_wheel_pid_gains->Ki = mavlink_msg_right_wheel_pid_gains_get_Ki(msg);
    right_wheel_pid_gains->Kd = mavlink_msg_right_wheel_pid_gains_get_Kd(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN? msg->len : MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN;
        memset(right_wheel_pid_gains, 0, MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_LEN);
    memcpy(right_wheel_pid_gains, _MAV_PAYLOAD(msg), len);
#endif
}

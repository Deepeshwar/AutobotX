#pragma once
// MESSAGE ROBOT_DISTANCE_SENSOR_READINGS PACKING

#define MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS 133

MAVPACKED(
typedef struct __mavlink_robot_distance_sensor_readings_t {
 uint16_t ultrasonic_readings[5]; /*<  reading of five ultrasonic sensors, from right to left*/
}) mavlink_robot_distance_sensor_readings_t;

#define MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN 10
#define MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_MIN_LEN 10
#define MAVLINK_MSG_ID_133_LEN 10
#define MAVLINK_MSG_ID_133_MIN_LEN 10

#define MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_CRC 28
#define MAVLINK_MSG_ID_133_CRC 28

#define MAVLINK_MSG_ROBOT_DISTANCE_SENSOR_READINGS_FIELD_ULTRASONIC_READINGS_LEN 5

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROBOT_DISTANCE_SENSOR_READINGS { \
    133, \
    "ROBOT_DISTANCE_SENSOR_READINGS", \
    1, \
    {  { "ultrasonic_readings", NULL, MAVLINK_TYPE_UINT16_T, 5, 0, offsetof(mavlink_robot_distance_sensor_readings_t, ultrasonic_readings) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROBOT_DISTANCE_SENSOR_READINGS { \
    "ROBOT_DISTANCE_SENSOR_READINGS", \
    1, \
    {  { "ultrasonic_readings", NULL, MAVLINK_TYPE_UINT16_T, 5, 0, offsetof(mavlink_robot_distance_sensor_readings_t, ultrasonic_readings) }, \
         } \
}
#endif

/**
 * @brief Pack a robot_distance_sensor_readings message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ultrasonic_readings  reading of five ultrasonic sensors, from right to left
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robot_distance_sensor_readings_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint16_t *ultrasonic_readings)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN];

    _mav_put_uint16_t_array(buf, 0, ultrasonic_readings, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN);
#else
    mavlink_robot_distance_sensor_readings_t packet;

    mav_array_memcpy(packet.ultrasonic_readings, ultrasonic_readings, sizeof(uint16_t)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_CRC);
}

/**
 * @brief Pack a robot_distance_sensor_readings message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ultrasonic_readings  reading of five ultrasonic sensors, from right to left
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robot_distance_sensor_readings_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint16_t *ultrasonic_readings)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN];

    _mav_put_uint16_t_array(buf, 0, ultrasonic_readings, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN);
#else
    mavlink_robot_distance_sensor_readings_t packet;

    mav_array_memcpy(packet.ultrasonic_readings, ultrasonic_readings, sizeof(uint16_t)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_CRC);
}

/**
 * @brief Encode a robot_distance_sensor_readings struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param robot_distance_sensor_readings C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robot_distance_sensor_readings_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_robot_distance_sensor_readings_t* robot_distance_sensor_readings)
{
    return mavlink_msg_robot_distance_sensor_readings_pack(system_id, component_id, msg, robot_distance_sensor_readings->ultrasonic_readings);
}

/**
 * @brief Encode a robot_distance_sensor_readings struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param robot_distance_sensor_readings C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robot_distance_sensor_readings_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_robot_distance_sensor_readings_t* robot_distance_sensor_readings)
{
    return mavlink_msg_robot_distance_sensor_readings_pack_chan(system_id, component_id, chan, msg, robot_distance_sensor_readings->ultrasonic_readings);
}

/**
 * @brief Send a robot_distance_sensor_readings message
 * @param chan MAVLink channel to send the message
 *
 * @param ultrasonic_readings  reading of five ultrasonic sensors, from right to left
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_robot_distance_sensor_readings_send(mavlink_channel_t chan, const uint16_t *ultrasonic_readings)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN];

    _mav_put_uint16_t_array(buf, 0, ultrasonic_readings, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS, buf, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_CRC);
#else
    mavlink_robot_distance_sensor_readings_t packet;

    mav_array_memcpy(packet.ultrasonic_readings, ultrasonic_readings, sizeof(uint16_t)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS, (const char *)&packet, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_CRC);
#endif
}

/**
 * @brief Send a robot_distance_sensor_readings message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_robot_distance_sensor_readings_send_struct(mavlink_channel_t chan, const mavlink_robot_distance_sensor_readings_t* robot_distance_sensor_readings)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_robot_distance_sensor_readings_send(chan, robot_distance_sensor_readings->ultrasonic_readings);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS, (const char *)robot_distance_sensor_readings, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_robot_distance_sensor_readings_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint16_t *ultrasonic_readings)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_uint16_t_array(buf, 0, ultrasonic_readings, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS, buf, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_CRC);
#else
    mavlink_robot_distance_sensor_readings_t *packet = (mavlink_robot_distance_sensor_readings_t *)msgbuf;

    mav_array_memcpy(packet->ultrasonic_readings, ultrasonic_readings, sizeof(uint16_t)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS, (const char *)packet, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_MIN_LEN, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_CRC);
#endif
}
#endif

#endif

// MESSAGE ROBOT_DISTANCE_SENSOR_READINGS UNPACKING


/**
 * @brief Get field ultrasonic_readings from robot_distance_sensor_readings message
 *
 * @return  reading of five ultrasonic sensors, from right to left
 */
static inline uint16_t mavlink_msg_robot_distance_sensor_readings_get_ultrasonic_readings(const mavlink_message_t* msg, uint16_t *ultrasonic_readings)
{
    return _MAV_RETURN_uint16_t_array(msg, ultrasonic_readings, 5,  0);
}

/**
 * @brief Decode a robot_distance_sensor_readings message into a struct
 *
 * @param msg The message to decode
 * @param robot_distance_sensor_readings C-struct to decode the message contents into
 */
static inline void mavlink_msg_robot_distance_sensor_readings_decode(const mavlink_message_t* msg, mavlink_robot_distance_sensor_readings_t* robot_distance_sensor_readings)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_robot_distance_sensor_readings_get_ultrasonic_readings(msg, robot_distance_sensor_readings->ultrasonic_readings);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN? msg->len : MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN;
        memset(robot_distance_sensor_readings, 0, MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_LEN);
    memcpy(robot_distance_sensor_readings, _MAV_PAYLOAD(msg), len);
#endif
}

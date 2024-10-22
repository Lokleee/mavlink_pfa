#pragma once
// MESSAGE HOVER_THRUST_ESTIMATE PACKING

#define MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE 98


typedef struct __mavlink_hover_thrust_estimate_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float thrust; /*<  Estimate hover thrust*/
} mavlink_hover_thrust_estimate_t;

#define MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN 8
#define MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN 8
#define MAVLINK_MSG_ID_98_LEN 8
#define MAVLINK_MSG_ID_98_MIN_LEN 8

#define MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_CRC 188
#define MAVLINK_MSG_ID_98_CRC 188



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HOVER_THRUST_ESTIMATE { \
    98, \
    "HOVER_THRUST_ESTIMATE", \
    2, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_hover_thrust_estimate_t, time_boot_ms) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_hover_thrust_estimate_t, thrust) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HOVER_THRUST_ESTIMATE { \
    "HOVER_THRUST_ESTIMATE", \
    2, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_hover_thrust_estimate_t, time_boot_ms) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_hover_thrust_estimate_t, thrust) }, \
         } \
}
#endif

/**
 * @brief Pack a hover_thrust_estimate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param thrust  Estimate hover thrust
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hover_thrust_estimate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, thrust);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN);
#else
    mavlink_hover_thrust_estimate_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.thrust = thrust;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_CRC);
}

/**
 * @brief Pack a hover_thrust_estimate message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param thrust  Estimate hover thrust
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hover_thrust_estimate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, thrust);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN);
#else
    mavlink_hover_thrust_estimate_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.thrust = thrust;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_CRC);
}

/**
 * @brief Encode a hover_thrust_estimate struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hover_thrust_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hover_thrust_estimate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hover_thrust_estimate_t* hover_thrust_estimate)
{
    return mavlink_msg_hover_thrust_estimate_pack(system_id, component_id, msg, hover_thrust_estimate->time_boot_ms, hover_thrust_estimate->thrust);
}

/**
 * @brief Encode a hover_thrust_estimate struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hover_thrust_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hover_thrust_estimate_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hover_thrust_estimate_t* hover_thrust_estimate)
{
    return mavlink_msg_hover_thrust_estimate_pack_chan(system_id, component_id, chan, msg, hover_thrust_estimate->time_boot_ms, hover_thrust_estimate->thrust);
}

/**
 * @brief Send a hover_thrust_estimate message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param thrust  Estimate hover thrust
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hover_thrust_estimate_send(mavlink_channel_t chan, uint32_t time_boot_ms, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, thrust);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE, buf, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_CRC);
#else
    mavlink_hover_thrust_estimate_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.thrust = thrust;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE, (const char *)&packet, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_CRC);
#endif
}

/**
 * @brief Send a hover_thrust_estimate message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hover_thrust_estimate_send_struct(mavlink_channel_t chan, const mavlink_hover_thrust_estimate_t* hover_thrust_estimate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hover_thrust_estimate_send(chan, hover_thrust_estimate->time_boot_ms, hover_thrust_estimate->thrust);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE, (const char *)hover_thrust_estimate, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hover_thrust_estimate_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, thrust);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE, buf, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_CRC);
#else
    mavlink_hover_thrust_estimate_t *packet = (mavlink_hover_thrust_estimate_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->thrust = thrust;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE, (const char *)packet, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_CRC);
#endif
}
#endif

#endif

// MESSAGE HOVER_THRUST_ESTIMATE UNPACKING


/**
 * @brief Get field time_boot_ms from hover_thrust_estimate message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_hover_thrust_estimate_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field thrust from hover_thrust_estimate message
 *
 * @return  Estimate hover thrust
 */
static inline float mavlink_msg_hover_thrust_estimate_get_thrust(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a hover_thrust_estimate message into a struct
 *
 * @param msg The message to decode
 * @param hover_thrust_estimate C-struct to decode the message contents into
 */
static inline void mavlink_msg_hover_thrust_estimate_decode(const mavlink_message_t* msg, mavlink_hover_thrust_estimate_t* hover_thrust_estimate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    hover_thrust_estimate->time_boot_ms = mavlink_msg_hover_thrust_estimate_get_time_boot_ms(msg);
    hover_thrust_estimate->thrust = mavlink_msg_hover_thrust_estimate_get_thrust(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN? msg->len : MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN;
        memset(hover_thrust_estimate, 0, MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN);
    memcpy(hover_thrust_estimate, _MAV_PAYLOAD(msg), len);
#endif
}

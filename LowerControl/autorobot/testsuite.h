/** @file
 *    @brief MAVLink comm protocol testsuite generated from autorobot.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef AUTOROBOT_TESTSUITE_H
#define AUTOROBOT_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_autorobot(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_autorobot(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_left_wheel_pid_gains(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LEFT_WHEEL_PID_GAINS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_left_wheel_pid_gains_t packet_in = {
        17.0,45.0,73.0
    };
    mavlink_left_wheel_pid_gains_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.Kp = packet_in.Kp;
        packet1.Ki = packet_in.Ki;
        packet1.Kd = packet_in.Kd;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LEFT_WHEEL_PID_GAINS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LEFT_WHEEL_PID_GAINS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_left_wheel_pid_gains_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_left_wheel_pid_gains_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_left_wheel_pid_gains_pack(system_id, component_id, &msg , packet1.Kp , packet1.Ki , packet1.Kd );
    mavlink_msg_left_wheel_pid_gains_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_left_wheel_pid_gains_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.Kp , packet1.Ki , packet1.Kd );
    mavlink_msg_left_wheel_pid_gains_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_left_wheel_pid_gains_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_left_wheel_pid_gains_send(MAVLINK_COMM_1 , packet1.Kp , packet1.Ki , packet1.Kd );
    mavlink_msg_left_wheel_pid_gains_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_right_wheel_pid_gains(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_right_wheel_pid_gains_t packet_in = {
        17.0,45.0,73.0
    };
    mavlink_right_wheel_pid_gains_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.Kp = packet_in.Kp;
        packet1.Ki = packet_in.Ki;
        packet1.Kd = packet_in.Kd;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RIGHT_WHEEL_PID_GAINS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_right_wheel_pid_gains_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_right_wheel_pid_gains_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_right_wheel_pid_gains_pack(system_id, component_id, &msg , packet1.Kp , packet1.Ki , packet1.Kd );
    mavlink_msg_right_wheel_pid_gains_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_right_wheel_pid_gains_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.Kp , packet1.Ki , packet1.Kd );
    mavlink_msg_right_wheel_pid_gains_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_right_wheel_pid_gains_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_right_wheel_pid_gains_send(MAVLINK_COMM_1 , packet1.Kp , packet1.Ki , packet1.Kd );
    mavlink_msg_right_wheel_pid_gains_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_robot_dimensions(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROBOT_DIMENSIONS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_robot_dimensions_t packet_in = {
        17.0,45.0
    };
    mavlink_robot_dimensions_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.wheel_radius = packet_in.wheel_radius;
        packet1.distance_bw_wheels = packet_in.distance_bw_wheels;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROBOT_DIMENSIONS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROBOT_DIMENSIONS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_dimensions_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_robot_dimensions_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_dimensions_pack(system_id, component_id, &msg , packet1.wheel_radius , packet1.distance_bw_wheels );
    mavlink_msg_robot_dimensions_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_dimensions_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.wheel_radius , packet1.distance_bw_wheels );
    mavlink_msg_robot_dimensions_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_robot_dimensions_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_dimensions_send(MAVLINK_COMM_1 , packet1.wheel_radius , packet1.distance_bw_wheels );
    mavlink_msg_robot_dimensions_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_desire_wheel_rpm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DESIRE_WHEEL_RPM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_desire_wheel_rpm_t packet_in = {
        17.0,45.0
    };
    mavlink_desire_wheel_rpm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.left_wheel_rpm = packet_in.left_wheel_rpm;
        packet1.right_wheel_rpm = packet_in.right_wheel_rpm;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DESIRE_WHEEL_RPM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_desire_wheel_rpm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_desire_wheel_rpm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_desire_wheel_rpm_pack(system_id, component_id, &msg , packet1.left_wheel_rpm , packet1.right_wheel_rpm );
    mavlink_msg_desire_wheel_rpm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_desire_wheel_rpm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.left_wheel_rpm , packet1.right_wheel_rpm );
    mavlink_msg_desire_wheel_rpm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_desire_wheel_rpm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_desire_wheel_rpm_send(MAVLINK_COMM_1 , packet1.left_wheel_rpm , packet1.right_wheel_rpm );
    mavlink_msg_desire_wheel_rpm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_desire_cmd_val(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DESIRE_CMD_VAL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_desire_cmd_val_t packet_in = {
        17.0,45.0
    };
    mavlink_desire_cmd_val_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.v = packet_in.v;
        packet1.w = packet_in.w;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DESIRE_CMD_VAL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DESIRE_CMD_VAL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_desire_cmd_val_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_desire_cmd_val_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_desire_cmd_val_pack(system_id, component_id, &msg , packet1.v , packet1.w );
    mavlink_msg_desire_cmd_val_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_desire_cmd_val_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.v , packet1.w );
    mavlink_msg_desire_cmd_val_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_desire_cmd_val_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_desire_cmd_val_send(MAVLINK_COMM_1 , packet1.v , packet1.w );
    mavlink_msg_desire_cmd_val_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_initial_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_INITIAL_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_initial_status_t packet_in = {
        17.0,17443,17547,29
    };
    mavlink_initial_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.initial_heading = packet_in.initial_heading;
        packet1.update_freq = packet_in.update_freq;
        packet1.encoder_ppr = packet_in.encoder_ppr;
        packet1.ultrasonic_config = packet_in.ultrasonic_config;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_INITIAL_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_INITIAL_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_initial_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_initial_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_initial_status_pack(system_id, component_id, &msg , packet1.initial_heading , packet1.ultrasonic_config , packet1.update_freq , packet1.encoder_ppr );
    mavlink_msg_initial_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_initial_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.initial_heading , packet1.ultrasonic_config , packet1.update_freq , packet1.encoder_ppr );
    mavlink_msg_initial_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_initial_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_initial_status_send(MAVLINK_COMM_1 , packet1.initial_heading , packet1.ultrasonic_config , packet1.update_freq , packet1.encoder_ppr );
    mavlink_msg_initial_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_robot_position_change(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_robot_position_change_t packet_in = {
        17.0,45.0,73.0
    };
    mavlink_robot_position_change_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.delta_x = packet_in.delta_x;
        packet1.delta_y = packet_in.delta_y;
        packet1.delta_theta = packet_in.delta_theta;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROBOT_POSITION_CHANGE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_position_change_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_robot_position_change_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_position_change_pack(system_id, component_id, &msg , packet1.delta_x , packet1.delta_y , packet1.delta_theta );
    mavlink_msg_robot_position_change_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_position_change_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.delta_x , packet1.delta_y , packet1.delta_theta );
    mavlink_msg_robot_position_change_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_robot_position_change_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_position_change_send(MAVLINK_COMM_1 , packet1.delta_x , packet1.delta_y , packet1.delta_theta );
    mavlink_msg_robot_position_change_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_robot_sensor_readings(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_robot_sensor_readings_t packet_in = {
        963497464,963497672,73.0
    };
    mavlink_robot_sensor_readings_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.left_wheel_ticks = packet_in.left_wheel_ticks;
        packet1.right_wheel_ticks = packet_in.right_wheel_ticks;
        packet1.curr_heading = packet_in.curr_heading;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROBOT_SENSOR_READINGS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_sensor_readings_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_robot_sensor_readings_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_sensor_readings_pack(system_id, component_id, &msg , packet1.left_wheel_ticks , packet1.right_wheel_ticks , packet1.curr_heading );
    mavlink_msg_robot_sensor_readings_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_sensor_readings_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.left_wheel_ticks , packet1.right_wheel_ticks , packet1.curr_heading );
    mavlink_msg_robot_sensor_readings_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_robot_sensor_readings_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_sensor_readings_send(MAVLINK_COMM_1 , packet1.left_wheel_ticks , packet1.right_wheel_ticks , packet1.curr_heading );
    mavlink_msg_robot_sensor_readings_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_robot_distance_sensor_readings(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_robot_distance_sensor_readings_t packet_in = {
        { 17235, 17236, 17237, 17238, 17239 }
    };
    mavlink_robot_distance_sensor_readings_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.ultrasonic_readings, packet_in.ultrasonic_readings, sizeof(uint16_t)*5);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROBOT_DISTANCE_SENSOR_READINGS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_distance_sensor_readings_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_robot_distance_sensor_readings_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_distance_sensor_readings_pack(system_id, component_id, &msg , packet1.ultrasonic_readings );
    mavlink_msg_robot_distance_sensor_readings_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_distance_sensor_readings_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ultrasonic_readings );
    mavlink_msg_robot_distance_sensor_readings_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_robot_distance_sensor_readings_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robot_distance_sensor_readings_send(MAVLINK_COMM_1 , packet1.ultrasonic_readings );
    mavlink_msg_robot_distance_sensor_readings_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_autorobot(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_left_wheel_pid_gains(system_id, component_id, last_msg);
    mavlink_test_right_wheel_pid_gains(system_id, component_id, last_msg);
    mavlink_test_robot_dimensions(system_id, component_id, last_msg);
    mavlink_test_desire_wheel_rpm(system_id, component_id, last_msg);
    mavlink_test_desire_cmd_val(system_id, component_id, last_msg);
    mavlink_test_initial_status(system_id, component_id, last_msg);
    mavlink_test_robot_position_change(system_id, component_id, last_msg);
    mavlink_test_robot_sensor_readings(system_id, component_id, last_msg);
    mavlink_test_robot_distance_sensor_readings(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // AUTOROBOT_TESTSUITE_H

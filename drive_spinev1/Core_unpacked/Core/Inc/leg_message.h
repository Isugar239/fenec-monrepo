/*
 * leg_message.h
 *
 *  Created on: Dec 10, 2022
 *      Author: Alex
 */

#ifndef INC_LEG_MESSAGE_H_
#define INC_LEG_MESSAGE_H_

// 60 bytes
// 30 16-bit words
typedef struct
{
    float q_abad[2];
    float q_hip[2];
    float q_knee[2];
    float qd_abad[2];
    float qd_hip[2];
    float qd_knee[2];
    int32_t flags[2];
    int32_t checksum;
} spi_data_t;

// 132 bytes
// 66 16-bit words
typedef struct
{
    float q_des_abad[2];
    float q_des_hip[2];
    float q_des_knee[2];
    float qd_des_abad[2];
    float qd_des_hip[2];
    float qd_des_knee[2];
    float kp_abad[2];
    float kp_hip[2];
    float kp_knee[2];
    float kd_abad[2];
    float kd_hip[2];
    float kd_knee[2];
    float tau_abad_ff[2];
    float tau_hip_ff[2];
    float tau_knee_ff[2];
    int32_t flags[2];
    int32_t checksum;
} spi_command_t;

typedef struct {
    float p_des, v_des, kp, kd, t_ff;
    } joint_control;

typedef struct {
    float p, v, t;
    } joint_state;

typedef struct {
    joint_state a, h, k;
    } leg_state;

typedef struct {
    joint_control a, h, k;
    } leg_control;

#endif /* INC_LEG_MESSAGE_H_ */

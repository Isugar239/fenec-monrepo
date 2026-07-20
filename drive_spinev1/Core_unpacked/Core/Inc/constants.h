/*
 * constants.h
 *
 *  Created on: Dec 10, 2022
 *      Author: Alex
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

#define INDICATOR 		GPIOC, GPIO_PIN_5  // Enable gate drive pin.
#define CAN_H1			hcan1				// CAN handle
#define CAN_H2			hcan2				// CAN handle

//SPI
// length of receive/transmit buffers
#define RX_LEN 66
#define TX_LEN 66

// length of outgoing/incoming messages
#define DATA_LEN 30
#define CMD_LEN 66

#define ENABLE_CMD 0xFFFF
#define DISABLE_CMD 0x1F1F

//CAN
// Master CAN ID ///
#define CAN_ID 0x0
#define ABAD_ID 0x1
#define HIP_ID 0x2
#define KNEE_ID 0x3

/// Value Limits ///
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -112.5f
#define T_MAX 112.5f

/// Joint Soft Stops ///
#define A_LIM_P 10.0f
#define A_LIM_N -10.0f
#define H_LIM_P 10.0f
#define H_LIM_N -10.0f
#define K_LIM_P 10.0f
#define K_LIM_N -10.0f
#define KP_SOFTSTOP 0.0f //100.0f
#define KD_SOFTSTOP 0.0f //0.4f

#endif /* INC_CONSTANTS_H_ */

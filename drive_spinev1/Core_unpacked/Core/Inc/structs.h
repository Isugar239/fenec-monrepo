/*
 * structs.h
 *
 *  Created on: Dec 10, 2022
 *      Author: Alex
 */

#ifndef INC_STRUCTS_H_
#define INC_STRUCTS_H_

#include "can.h"
#include "leg_message.h"

extern CANRxMessage can1_rx, can2_rx;
extern CANTxMessage a1_can, a2_can, h1_can, h2_can, k1_can, k2_can; //TX Messages

extern spi_data_t spi_data;       // data from spine to up
extern spi_command_t spi_command; // data from up to spine
extern leg_state l1_state, l2_state;
extern leg_control l1_control, l2_control;


#endif /* INC_STRUCTS_H_ */

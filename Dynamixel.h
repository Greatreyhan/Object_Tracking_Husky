/*
 * Dynamixel.h
 *
 *  Created on: Mar 10, 2022
 *      Author: Maulana Reyhan Savero
 */
#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#include "main.h"
#include <string.h>

typedef struct{
	uint8_t tx_buf[15];
	uint8_t rx_buf[20];
	uint8_t id;
}dynamixel_t;

typedef enum{
	TORQUE_OFF = 0x00U,
	TORQUE_ON = 0x01U
}dynamixel_torque_t;

typedef enum{
	LED_OFF = 0x00U,
	LED_ON = 0x01U
}dynamixel_led_t;

typedef enum{
	MOVING_CCW = 0x00U,
	MOVING_CW = 0x01U
}dynamixel_direction_t;

void dyna_set_bautrate(dynamixel_t* dyn, uint8_t speed);
void dyna_set_delay(dynamixel_t* dyn, uint8_t time);
void dyna_set_voltage_limit(dynamixel_t* dyn, uint8_t up, uint8_t down);
void dyna_set_torque_value(dynamixel_t* dyn, uint16_t torq);
void dyna_set_margin_CW(dynamixel_t* dyn, uint8_t margin);
void dyna_set_margin_CCW(dynamixel_t* dyn, uint8_t margin);
void dyna_set_slope_CW(dynamixel_t* dyn, uint8_t margin);
void dyna_set_slope_CCW(dynamixel_t* dyn, uint8_t margin);


// Tested
void dyna_init(UART_HandleTypeDef* addr, dynamixel_t* dyn, uint8_t id_dyn);
void dyna_set_torque_enabler(dynamixel_t* dyn, dynamixel_torque_t condition);
void dyna_ping(dynamixel_t* dyn);
void dyna_reset_mem(dynamixel_t* dyn);
void dyna_set_led(dynamixel_t* dyn, dynamixel_led_t led);
void dyna_set_goal_position(dynamixel_t* dyn, uint16_t goal);
void dyna_set_moving_speed(dynamixel_t* dyn, uint16_t speed, dynamixel_direction_t dir);
static uint8_t checksum_generator(uint8_t* msg, uint8_t len);
void dyna_set_id(dynamixel_t* dyn, uint8_t new_id);
void dyna_set_limit_CW(dynamixel_t* dyn, uint16_t angle);
void dyna_set_limit_CCW(dynamixel_t* dyn, uint16_t angle);
uint16_t dyna_read_posisition(dynamixel_t* dyn);

// Competition
void dyna_endless_turn(dynamixel_t* dyn, uint16_t time, uint16_t speed, dynamixel_direction_t dir);
void dyna_calibrate(dynamixel_t* dyn);
void dyna_scan(dynamixel_t* dyn, uint16_t start, uint16_t speed, dynamixel_direction_t dir);
uint8_t dyna_temperature(dynamixel_t* dyn);

// Testing
uint8_t dyna_read_posisition_L(dynamixel_t* dyn);
uint8_t dyna_read_posisition_H(dynamixel_t* dyn);
#endif
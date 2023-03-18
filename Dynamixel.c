/*
 * Dynamixel.c
 *
 *  Created on: Mar 10, 2022
 *      Author: Maulana Reyhan Savero
 */

#include "Dynamixel.h"

static UART_HandleTypeDef* huart;
static uint8_t rx_buf[10];

void dyna_init(UART_HandleTypeDef* addr, dynamixel_t* dyn, uint8_t id_dyn){
	huart = addr;
	dyn->id = id_dyn;
}

void dyna_ping(dynamixel_t* dyn){
	uint8_t ping[6] = {0xFF, 0xFF, dyn->id, 0x02, 0x01, 0x00};
	uint8_t chk = checksum_generator(ping,6);
	ping[5] = chk;
	
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, ping, 6, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
	memcpy(dyn->rx_buf, rx_buf, 6);
}

void dyna_set_id(dynamixel_t* dyn, uint8_t new_id){
	uint8_t setID[8] = {0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x03, new_id, 0x00};
	uint8_t chk = checksum_generator(setID, 8);
	setID[7] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, setID, 8, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
	dyn->id = new_id;
}

void dyna_reset_mem(dynamixel_t* dyn){
	uint8_t reset[6] = {0xFF, 0xFF, dyn->id, 0x02, 0x06, 0x00};
	uint8_t chk = checksum_generator(reset, 6);
	reset[5] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, reset, 6, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
}

void dyna_set_delay(dynamixel_t* dyn, uint8_t time){
	uint8_t timeMsg[8] = {0xFF, 0xFF, dyn->id, 0x04, 0x03, 0x05, time, 0x00};
	uint8_t chk = checksum_generator(timeMsg, 8);
	timeMsg[7] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, timeMsg, 8, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
}

void dyna_set_bautrate(dynamixel_t* dyn, uint8_t speed){
	uint8_t baudrate[8] = {0xFF, 0xFF, dyn->id, 0x04, 0x03, 0x04, speed, 0x00};
	uint8_t chk = checksum_generator(baudrate, 8);
	baudrate[7] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, baudrate, 8, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
}

void dyna_set_limit_CW(dynamixel_t* dyn, uint16_t angle){
	uint8_t angleMsg[9] = {0xFF, 0xFF, dyn->id, 0x05, 0x03, 0x06, (angle & 0xFF),((angle >> 8) & 0xFF), 0x00};
	uint8_t chk = checksum_generator(angleMsg, 9);	
	angleMsg[8] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, angleMsg, 9, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
}

void dyna_set_limit_CCW(dynamixel_t* dyn, uint16_t angle){
	uint8_t angleMsg[9] = {0xFF, 0xFF, dyn->id, 0x05, 0x03, 0x08, (angle & 0xFF), ((angle >> 8) & 0xFF), 0x00};
	uint8_t chk = checksum_generator(angleMsg, 9);	
	angleMsg[8] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, angleMsg, 9, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
}

void dyna_set_voltage_limit(dynamixel_t* dyn, uint8_t up, uint8_t down){
	uint8_t voltageMsg[10] = {0xFF, 0xFF, dyn->id, 0x06, 0x03, 0x0C, down, 0x0D, up, 0x00};
	uint8_t chk = checksum_generator(voltageMsg, 10);	
	voltageMsg[9] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, voltageMsg, 10, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
}

void dyna_set_torque_value(dynamixel_t* dyn, uint16_t torq){
	uint8_t torMsg[10] = {0xFF, 0xFF, dyn->id, 0x06, 0x03, 0x0E, (torq & 0xFF), 0x0F, ((torq >> 8) & 0xFF), 0x00};
	uint8_t chk = checksum_generator(torMsg, 10);	
	torMsg[9] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, torMsg, 10, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
}

void dyna_set_torque_enabler(dynamixel_t* dyn, dynamixel_torque_t condition){
	uint8_t torque[9] = {0xFF, 0xFF, dyn->id, 0x05, 0x03, 0x18, condition, 0x01, 0x00};
	uint8_t chk = checksum_generator(torque, 9);
	torque[8] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, torque, 9, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
}

void dyna_set_led(dynamixel_t* dyn, dynamixel_led_t led){
	uint8_t ledMsg[8] = {0xFF, 0xFF, dyn->id, 0x04, 0x03, 0x19, led, 0x00};
	uint8_t chk = checksum_generator(ledMsg, 8);
	ledMsg[7] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, ledMsg, 8, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
}

void dyna_set_margin_CW(dynamixel_t* dyn, uint8_t margin){
	uint8_t compliance[8] = {0xFF, 0xFF, dyn->id, 0x04, 0x03, 0x1A, margin, 0x00};
	uint8_t chk = checksum_generator(compliance, 8);
	compliance[7] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, compliance, 8, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
}

void dyna_set_margin_CCW(dynamixel_t* dyn, uint8_t margin){
	uint8_t compliance[8] = {0xFF, 0xFF, dyn->id, 0x04, 0x03, 0x1B, margin, 0x00};
	uint8_t chk = checksum_generator(compliance, 8);
	compliance[7] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, compliance, 8, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
}

void dyna_set_slope_CW(dynamixel_t* dyn, uint8_t margin){
	uint8_t compliance[8] = {0xFF, 0xFF, dyn->id, 0x04, 0x03, 0x1C, margin, 0x00};
	uint8_t chk = checksum_generator(compliance, 8);
	compliance[7] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, compliance, 8, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
}

void dyna_set_slope_CCW(dynamixel_t* dyn, uint8_t margin){
	uint8_t compliance[8] = {0xFF, 0xFF, dyn->id, 0x04, 0x03, 0x1D, margin, 0x00};
	uint8_t chk = checksum_generator(compliance, 8);
	compliance[7] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, compliance, 8, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
}

void dyna_set_goal_position(dynamixel_t* dyn, uint16_t goal){
	uint8_t pos[9] = {0xFF, 0xFF, dyn->id, 0x05, 0x03, 0x1E, (goal & 0xFF), ((goal >> 8) & 0xFF), 0x00};
	uint8_t chk = checksum_generator(pos, 9);	
	pos[8] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, pos, 9, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
}

void dyna_set_moving_speed(dynamixel_t* dyn, uint16_t speed, dynamixel_direction_t dir){
	uint8_t par = ((speed >> 8) & 0xFF);
	if(dir == MOVING_CW) par |= 0x04;
	uint8_t spd[9] = {0xFF, 0xFF, dyn->id, 0x05, 0x03, 0x20, (speed & 0xFF), par, 0x00};
	uint8_t chk = checksum_generator(spd, 9);	
	spd[8] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, spd, 9, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 6, 10);
}

uint8_t dyna_temperature(dynamixel_t* dyn){
	uint8_t readTemp[8] = {0xFF, 0xFF, dyn->id, 0x04, 0x02, 0x2B, 0x01, 0x00};
	uint8_t chk = checksum_generator(readTemp, 8);
	readTemp[7] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, readTemp, 8, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 7, 10);
	memcpy(dyn->rx_buf, rx_buf, 7);
}


uint8_t dyna_read_posisition_H(dynamixel_t* dyn){
	uint8_t pos[8] = {0xFF, 0xFF, dyn->id, 0x04, 0x02, 0X25, 0x01, 0x00};
	uint8_t chk = checksum_generator(pos, 8);
	pos[7] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, pos, 8, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 8, 10);
	memcpy(dyn->rx_buf, rx_buf, 8);
	return rx_buf[5];
}

uint8_t dyna_read_posisition_L(dynamixel_t* dyn){
	uint8_t pos[8] = {0xFF, 0xFF, dyn->id, 0x04, 0x02, 0X24, 0x01, 0x00};
	uint8_t chk = checksum_generator(pos, 8);
	pos[7] = chk;
	HAL_HalfDuplex_EnableTransmitter(huart);
	HAL_UART_Transmit(huart, pos, 8, 10);
	HAL_HalfDuplex_EnableReceiver(huart);
	HAL_UART_Receive(huart, rx_buf, 8, 10);
	memcpy(dyn->rx_buf, rx_buf, 8);
	return rx_buf[5];
}

static uint8_t checksum_generator(uint8_t* msg, uint8_t len){
	uint8_t buf = 0x00;
	for(int i = 2; i < len; i++){
		buf += msg[i];
	} 
	return ~buf;
}

// ---------------------------------- START FOR COMPETITION PURPOUSE -------------------------------------
uint16_t dyna_read_posisition(dynamixel_t* dyn){
	uint8_t mem_L = dyna_read_posisition_L(dyn);
	uint8_t mem_H = dyna_read_posisition_H(dyn);
	return ((mem_H << 8) | mem_L);
}

void dyna_endless_turn(dynamixel_t* dyn, uint16_t time, uint16_t speed, dynamixel_direction_t dir){
	dyna_set_torque_enabler(dyn, TORQUE_ON);
	dyna_set_limit_CW(dyn, 0);
	dyna_set_limit_CCW(dyn, 0);
	dyna_set_moving_speed(dyn, speed, dir);
	HAL_Delay(time);
	dyna_set_moving_speed(dyn, 0, dir);
}

void dyna_calibrate(dynamixel_t* dyn){
	dyna_set_torque_enabler(dyn, TORQUE_ON);
	dyna_set_limit_CW(dyn, 0);
	dyna_set_limit_CCW(dyn, 1023);
	dyna_set_goal_position(dyn, 523);
}

void dyna_scan(dynamixel_t* dyn, uint16_t start, uint16_t speed, dynamixel_direction_t dir){
	dyna_set_torque_enabler(dyn, TORQUE_ON);
	dyna_set_limit_CW(dyn, 0);
	dyna_set_limit_CCW(dyn, 1023);
	dyna_set_moving_speed(dyn, 500, dir);
	dyna_set_goal_position(dyn, start);
	HAL_Delay(1000);
	dyna_set_moving_speed(dyn, speed, dir);
	dyna_set_goal_position(dyn, 1023);
}
// ---------------------------------- END FOR COMPETITION PURPOUSE ---------------------------------------
/*
 * Huskylens_driver.c
 *
 *  Created on: Jan 18, 2022
 *      Author: Maulana Reyhan Savero
 */

#include "Huskylens_driver.h"

#define TIME_TRANSMIT 100
#define TIME_RECEIVE 100

static I2C_HandleTypeDef* hi2c;
static uint8_t HUSKY_ADDR = 0;

huskylens_status_t husky_setup(I2C_HandleTypeDef *i2cHandler ){
	HAL_Delay(50);
	hi2c = i2cHandler;
	for(int i =0; i<255;i++){
		if(HAL_I2C_IsDeviceReady(hi2c, i, 1, 10) == HAL_OK){
			 HUSKY_ADDR = i;
		}
	}
	uint8_t knock[] = {0x55, 0xAA, 0x11, 0x00, 0x2C, 0x3C};
	uint8_t rxBuff[6];
	HAL_I2C_Master_Transmit(hi2c, HUSKY_ADDR, knock, 6, TIME_TRANSMIT);
	
	// Return OK
	HAL_I2C_Master_Receive(hi2c, HUSKY_ADDR, rxBuff, 6, TIME_RECEIVE);
	
	if((rxBuff[0] == 0x55) && (rxBuff[1] == 0xAA) && (rxBuff[2] == 0x11) && (rxBuff[3] == 0x00) && (rxBuff[4] == 0x2E) && (rxBuff[5] == 0x3E) ){
		return HUSKY_OK;
	}
	else{
		return HUSKY_ERR;
	}
	
	return HUSKY_TIMEOUT;
}

huskylens_info_t husky_getAllArrowBlock(void){
	huskylens_info_t handler;
	// Command Request
	uint8_t req[] = {0x55, 0xAA, 0x11, 0x00, 0x20, 0x30};
	HAL_I2C_Master_Transmit(hi2c, HUSKY_ADDR, req, 6, TIME_TRANSMIT);
	
	// Return INFO
	uint8_t rxBuff[16];
	HAL_I2C_Master_Receive(hi2c, HUSKY_ADDR, rxBuff, 16, TIME_RECEIVE);
	handler.num_block_arr = (rxBuff[6] << 8) | rxBuff[5];
	handler.num_id = (rxBuff[8] << 8) | rxBuff[7];
	handler.current_frame = (rxBuff[10] << 8) | rxBuff[9];
	
	return handler;
}

huskylens_arrow_t husky_getArrows(void){
	huskylens_arrow_t handler;
	// Command Request
	uint8_t reqArr[] = {0x55, 0xAA, 0x11, 0x00, 0x22, 0x32};
	HAL_I2C_Master_Transmit(hi2c, HUSKY_ADDR, reqArr, 6, TIME_TRANSMIT);
	
	// Return 
	uint8_t rxBuff[50];
  HAL_I2C_Master_Receive(hi2c, HUSKY_ADDR, rxBuff, 50, TIME_RECEIVE);
	if( rxBuff[4] == 0x29){
		handler.info.num_block_arr = (rxBuff[6] << 8) | rxBuff[5];
		handler.info.num_id = (rxBuff[8] << 8) | rxBuff[7];
		handler.info.current_frame = (rxBuff[10] << 8) | rxBuff[9];
	}
	if( rxBuff[20] == 0x2B){
		handler.X_origin = (rxBuff[22] << 8) | rxBuff[21];
		handler.Y_origin = (rxBuff[24] << 8) | rxBuff[23];
		handler.X_target = (rxBuff[26] << 8) | rxBuff[25];
		handler.Y_target = (rxBuff[28] << 8) | rxBuff[27];
		handler.id = (rxBuff[30] << 8) | rxBuff[29];
	}
	return handler;
}

huskylens_block_t husky_getBlocks(void){
	huskylens_block_t handler;
	uint8_t reqBlock[] = {0x55, 0xAA, 0x11, 0x00, 0x21, 0x31};
	HAL_I2C_Master_Transmit(hi2c, HUSKY_ADDR, reqBlock, 6, TIME_TRANSMIT);
	
	// Return INFO
	uint8_t rxBuff[50];
  HAL_I2C_Master_Receive(hi2c, HUSKY_ADDR, rxBuff, 50, TIME_RECEIVE);
	if( rxBuff[4] == 0x29){
		handler.info.num_block_arr = (rxBuff[6] << 8) | rxBuff[5];
		handler.info.num_id = (rxBuff[8] << 8) | rxBuff[7];
		handler.info.current_frame = (rxBuff[10] << 8) | rxBuff[9];
	}
	if( rxBuff[20] == 0x2A){
		handler.X_center = (rxBuff[22] << 8) | rxBuff[21];
		handler.Y_center = (rxBuff[24] << 8) | rxBuff[23];
		handler.width = (rxBuff[26] << 8) | rxBuff[25];
		handler.height = (rxBuff[28] << 8) | rxBuff[27];
		handler.id = (rxBuff[30] << 8) | rxBuff[29];
	}
	
	return handler;
}

huskylens_block_t husky_getLearnedBlocks(void){
	huskylens_block_t handler;
	uint8_t reqLearnedBlock[] = {0x55, 0xAA, 0x11, 0x00, 0x24, 0x34};
	HAL_I2C_Master_Transmit(hi2c, HUSKY_ADDR, reqLearnedBlock, 6, TIME_TRANSMIT);
	
	// Return INFO
	uint8_t rxBuff[50];
  HAL_I2C_Master_Receive(hi2c, HUSKY_ADDR, rxBuff, 50, TIME_RECEIVE);
	if( rxBuff[4] == 0x29){
		handler.info.num_block_arr = (rxBuff[6] << 8) | rxBuff[5];
		handler.info.num_id = (rxBuff[8] << 8) | rxBuff[7];
		handler.info.current_frame = (rxBuff[10] << 8) | rxBuff[9];
	}
	if( rxBuff[20] == 0x2A){
		handler.X_center = (rxBuff[22] << 8) | rxBuff[21];
		handler.Y_center = (rxBuff[24] << 8) | rxBuff[23];
		handler.width = (rxBuff[26] << 8) | rxBuff[25];
		handler.height = (rxBuff[28] << 8) | rxBuff[27];
		handler.id = (rxBuff[30] << 8) | rxBuff[29];
	}
	
	return handler;
}

huskylens_arrow_t husky_getLearnedArrows(void){
	huskylens_arrow_t handler;
	// Command Request
	uint8_t reqArr[] = {0x55, 0xAA, 0x11, 0x00, 0x25, 0x35};
	HAL_I2C_Master_Transmit(hi2c, HUSKY_ADDR, reqArr, 6, TIME_TRANSMIT);
	
	// Return 
	uint8_t rxBuff[50];
  HAL_I2C_Master_Receive(hi2c, HUSKY_ADDR, rxBuff, 50, TIME_RECEIVE);
	if( rxBuff[4] == 0x29){
		handler.info.num_block_arr = (rxBuff[6] << 8) | rxBuff[5];
		handler.info.num_id = (rxBuff[8] << 8) | rxBuff[7];
		handler.info.current_frame = (rxBuff[10] << 8) | rxBuff[9];
	}
	if( rxBuff[20] == 0x2B){
		handler.X_origin = (rxBuff[22] << 8) | rxBuff[21];
		handler.Y_origin = (rxBuff[24] << 8) | rxBuff[23];
		handler.X_target = (rxBuff[26] << 8) | rxBuff[25];
		handler.Y_target = (rxBuff[28] << 8) | rxBuff[27];
		handler.id = (rxBuff[30] << 8) | rxBuff[29];
	}
	
	return handler;
}

huskylens_info_t husky_getAllById(uint16_t id){
	huskylens_info_t handler;
	// Command Request
	uint8_t reqArr[] = {0x55, 0xAA, 0x11, 0x02, 0x26, 0x01, 0x00, 0x39};
	HAL_I2C_Master_Transmit(hi2c, HUSKY_ADDR, reqArr, 8, TIME_TRANSMIT);
	
	// Return INFO
	uint8_t rxBuff[16];
	HAL_I2C_Master_Receive(hi2c, HUSKY_ADDR, rxBuff, 16, TIME_RECEIVE);
	handler.num_block_arr = (rxBuff[6] << 8) | rxBuff[5];
	handler.num_id = (rxBuff[8] << 8) | rxBuff[7];
	handler.current_frame = (rxBuff[10] << 8) | rxBuff[9];
	
	return handler;
}

huskylens_block_t husky_getBlockById(uint16_t id){
	huskylens_block_t handler;
	uint8_t msg[] = {0x55, 0xAA, 0x11, 0x02, 0x27, (id & 0xFF), ((id >> 8)&(0xFF)), 0x00 };
	msg[7] = (0x55 + 0xAA + 0x11 + 0x02+ 0x27+ (id & 0xFF) + ((id >> 8)&(0xFF)) + 0x00) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c, HUSKY_ADDR, msg, 8, 100);
	
	// Return
	uint8_t rxBuff[50];
  HAL_I2C_Master_Receive(hi2c, HUSKY_ADDR, rxBuff, 50, TIME_RECEIVE);
	if( rxBuff[4] == 0x29){
		handler.info.num_block_arr = (rxBuff[6] << 8) | rxBuff[5];
		handler.info.num_id = (rxBuff[8] << 8) | rxBuff[7];
		handler.info.current_frame = (rxBuff[10] << 8) | rxBuff[9];
	}
	if( rxBuff[20] == 0x2A){
		handler.X_center = (rxBuff[22] << 8) | rxBuff[21];
		handler.Y_center = (rxBuff[24] << 8) | rxBuff[23];
		handler.width = (rxBuff[26] << 8) | rxBuff[25];
		handler.height = (rxBuff[28] << 8) | rxBuff[27];
		handler.id = (rxBuff[30] << 8) | rxBuff[29];
	}
	return handler;
}
huskylens_arrow_t husky_getArrowById(uint16_t id){
	huskylens_arrow_t handler;
	// Command Request
	uint8_t msg[] = {0x55, 0xAA, 0x11, 0x02, 0x28, (id & 0xFF), ((id >> 8)&(0xFF)), 0x00 };
	msg[7] = (0x55 + 0xAA + 0x11 + 0x02+ 0x28+ (id & 0xFF) + ((id >> 8)&(0xFF)) + 0x00) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c, HUSKY_ADDR, msg, 8, TIME_TRANSMIT);
	
	// Return INFO
	uint8_t rxBuff[50];
  HAL_I2C_Master_Receive(hi2c, HUSKY_ADDR, rxBuff, 50, TIME_RECEIVE);
	if( rxBuff[4] == 0x29){
		handler.info.num_block_arr = (rxBuff[6] << 8) | rxBuff[5];
		handler.info.num_id = (rxBuff[8] << 8) | rxBuff[7];
		handler.info.current_frame = (rxBuff[10] << 8) | rxBuff[9];
	}
	if( rxBuff[20] == 0x2B){
		handler.X_origin = (rxBuff[22] << 8) | rxBuff[21];
		handler.Y_origin = (rxBuff[24] << 8) | rxBuff[23];
		handler.X_target = (rxBuff[26] << 8) | rxBuff[25];
		handler.Y_target = (rxBuff[28] << 8) | rxBuff[27];
		handler.id = (rxBuff[30] << 8) | rxBuff[29];
	}
}

huskylens_status_t husky_setAlgorithm(uint16_t algorithm){
	// Command Request
	uint8_t msg[] = {0x55, 0xAA, 0x11, 0x02, 0x2D, (algorithm & 0xFF), ((algorithm >> 8)&(0xFF)), 0x00 };
	msg[7] = (0x55 + 0xAA + 0x11 + 0x02+ 0x2D+ (algorithm & 0xFF) + ((algorithm >> 8)&(0xFF)) + 0x00) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c, HUSKY_ADDR, msg, 8, TIME_TRANSMIT);
	uint8_t rxBuff[6];
	HAL_I2C_Master_Receive(hi2c, HUSKY_ADDR, rxBuff, 6, TIME_RECEIVE);
	if((rxBuff[0] == 0x55) && (rxBuff[1] == 0xAA) && (rxBuff[2] == 0x11) && (rxBuff[3] == 0x00) && (rxBuff[4] == 0x2E) && (rxBuff[5] == 0x3E) ){
		return HUSKY_OK;
	}
	else{
		return HUSKY_ERR;
	}
	return HUSKY_TIMEOUT;
}

huskylens_status_t husky_savePic(void){
	uint8_t msg[] = {0x55, 0xAA, 0x11, 0x00, 0x30, 0x40};
	HAL_I2C_Master_Transmit(hi2c, HUSKY_ADDR, msg, 6, TIME_TRANSMIT);
	
	return HUSKY_OK;
}

huskylens_status_t husky_saveAlgorithm(uint16_t fileNumb){
// Command Request
	uint8_t msg[] = {0x55, 0xAA, 0x11, 0x02, 0x32, (fileNumb & 0xFF), ((fileNumb >> 8)&(0xFF)), 0x00 };
	msg[7] = (0x55 + 0xAA + 0x11 + 0x02+ 0x32+ (fileNumb & 0xFF) + ((fileNumb >> 8)&(0xFF)) + 0x00) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c, HUSKY_ADDR, msg, 8, TIME_TRANSMIT);
	
	return HUSKY_OK;
}

huskylens_status_t husky_loadAlgorithm(uint16_t fileNumb){
// Command Request
	uint8_t msg[] = {0x55, 0xAA, 0x11, 0x02, 0x33, (fileNumb & 0xFF), ((fileNumb >> 8)&(0xFF)), 0x00 };
	msg[7] = (0x55 + 0xAA + 0x11 + 0x02+ 0x33+ (fileNumb & 0xFF) + ((fileNumb >> 8)&(0xFF)) + 0x00) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c, HUSKY_ADDR, msg, 8, TIME_TRANSMIT);
	
	return HUSKY_OK;
}

huskylens_status_t husky_startLearn(uint16_t id){
// Command Request
	uint8_t msg[] = {0x55, 0xAA, 0x11, 0x02, 0x36, (id & 0xFF), ((id >> 8)&(0xFF)), 0x00 };
	msg[7] = (0x55 + 0xAA + 0x11 + 0x02+ 0x36+ (id & 0xFF) + ((id >> 8)&(0xFF)) + 0x00) & 0xFF;
	HAL_I2C_Master_Transmit(hi2c, HUSKY_ADDR, msg, 8, TIME_TRANSMIT);
	
	return HUSKY_OK;
}

huskylens_status_t husky_forget(void){
	uint8_t msg[] = {0x55, 0xAA, 0x11, 0x00, 0x37, 0x47};
	HAL_I2C_Master_Transmit(hi2c, HUSKY_ADDR, msg, 6, TIME_TRANSMIT);
	
	return HUSKY_OK;
}

huskylens_status_t husky_saveScreenShoot(void){
	uint8_t msg[] = {0x55, 0xAA, 0x11, 0x00, 0x39, 0x49};
	HAL_I2C_Master_Transmit(hi2c, HUSKY_ADDR, msg, 6, TIME_TRANSMIT);
	
	return HUSKY_OK;
}

// ---------------------------------- START FOR COMPETITION PURPOUSE ---------------------------------------

huskylens_area_identification_t husky_get_position(void){
		huskylens_block_t block = husky_getBlocks();
		if(block.id == 0x00001) return HOME_AREA;
		else if(block.id == 0x0002) return PILE_OF_WOOD_OBSTACLE;
		else if(block.id == 0x0003) return CLIMBING_OBSTACLE;
		else if(block.id == 0x0004) return ROOM_1_AREA;
		else if(block.id == 0x0005) return PYRAMID_OBSTACLE;
		else if(block.id == 0x0006) return ROOM_2_AREA;
		else return ERROR_DETECTION;
	return ERROR_DETECTION;
}

huskylens_victim_detection_t husky_victim_position(void){
		huskylens_block_t block = husky_getBlocks();
		if(block.id == 0x0001){
			if(block.X_center <= 160) return VICTIM_ON_LEFT;
			else if(block.X_center > 160) return VICTIM_ON_RIGHT;
		}
	return VICTIM_DETECTION_ERROR;
}

double husky_distance_prediction(void){
		huskylens_block_t block = husky_getBlocks();
		if(block.id == 0x0001){
			double angle, distance;
			
			angle = atan(block.height/block.width);
			distance = block.height/angle;
			
		}
	return 0;
}

// ---------------------------------- END FOR COMPETITION PURPOUSE ---------------------------------------

/*
 * Huskylens_driver.h
 *
 *  Created on: Jan 18, 2022
 *      Author: Maulana Reyhan Savero
 */
#ifndef HUSKYLENS_DRIVER_H_
#define HUSKYLENS_DRIVER_H_

#include "main.h"

#include <stdbool.h>
#include <string.h>
#include <math.h>

// Define
#define HEADER1	0x55
#define HEADER2 0xAA

#define ALGORITHM_FACE_RECOGNITION 0x0000
#define ALGORITHM_OBJECT_TRACKING 0x0001
#define ALGORITHM_OBJECT_RECOGNITION 0x0002
#define ALGORITHM_LINE_TRACKING 0x0003
#define ALGORITHM_COLOR_RECOGNITION 0x0004
#define ALGORITHM_TAG_RECOGNITION 0x0005
#define ALGORITHM_OBJECT_CLASSIFICATION 0x0006

// ---------------------------------- START FOR COMPETITION PURPOUSE ---------------------------------------
#define VICTIM_HEIGHT 10

typedef enum{
	HOME_AREA = 0x0001U,
	PILE_OF_WOOD_OBSTACLE = 0x0002U,
	CLIMBING_OBSTACLE = 0x0003U,
	ROOM_1_AREA = 0x0004U,
	PYRAMID_OBSTACLE = 0x0005U,
	ROOM_2_AREA = 0x0006U,
	ERROR_DETECTION = 0x0009U
}huskylens_area_identification_t;

typedef enum{
	VICTIM_ON_RIGHT = 0x0001U,
	VICTIM_ON_LEFT = 0x0002U,
	VICTIM_DETECTION_ERROR = 0x0003U
}huskylens_victim_detection_t;

// ---------------------------------- END FOR COMPETITION PURPOUSE ---------------------------------------

typedef struct{
	uint16_t num_block_arr;
	uint16_t num_id;
	uint16_t current_frame;
}huskylens_info_t;

typedef enum{
	HUSKY_OK = 0x00,
	HUSKY_TIMEOUT = 0x01,
	HUSKY_ERR = 0x03,
} huskylens_status_t;

// Typedef
typedef struct{
	huskylens_info_t info;
	uint16_t X_origin;
	uint16_t Y_origin;
	uint16_t X_target;
	uint16_t Y_target;
	uint16_t id;
} huskylens_arrow_t;

typedef struct{
	huskylens_info_t info;
	uint16_t X_center;
	uint16_t Y_center;
	uint16_t width;
	uint16_t height;
	uint16_t id;
} huskylens_block_t;

typedef struct{
	uint16_t num_block_arr;
	uint16_t num_id;
	uint16_t current_frame;
	uint8_t buff[25];
}huskylens_all_byid_t;

typedef struct{
	I2C_HandleTypeDef* i2c;
	uint8_t addr;
	uint8_t buff[20];
	uint16_t num_block_arr;
	uint16_t num_id;
	uint16_t current_frame;
	huskylens_arrow_t arrow;
	huskylens_block_t block;
} huskylens_t;


// Function

// Setup
huskylens_status_t husky_setup(I2C_HandleTypeDef *i2cHandler );

// Get All Row & Block
huskylens_info_t husky_getAllArrowBlock(void);

// Get All Arrow
huskylens_arrow_t husky_getArrows(void);

// Get All Block
huskylens_block_t husky_getBlocks(void);

// Get All Learned Block
huskylens_block_t husky_getLearnedBlocks(void);

// Get All Learned Arrows
huskylens_arrow_t husky_getLearnedArrows(void);

// Get All Block & Arrow by ID
huskylens_info_t husky_getAllById(uint16_t id);

// Get Block By Id
huskylens_block_t husky_getBlockById(uint16_t id);

// Get Arrow By Id
huskylens_arrow_t husky_getArrowById(uint16_t id);

// Change Algorithm
huskylens_status_t husky_setAlgorithm(uint16_t algorithm);

// Save Picture
huskylens_status_t husky_savePic(void);

// Save Algorithm
huskylens_status_t husky_saveAlgorithm(uint16_t fileNumb);

// Load Algorithm
huskylens_status_t husky_loadAlgorithm(uint16_t fileNumb);

// Start Learn
huskylens_status_t husky_startLearn(uint16_t id);

// Forget
huskylens_status_t husky_forget(void);

// Save Screen Shoot
huskylens_status_t husky_saveScreenShoot(void);


// ---------------------------------- START FOR COMPETITION PURPOUSE ---------------------------------------

huskylens_area_identification_t husky_get_position(void);

huskylens_victim_detection_t husky_victim_position(void);

double husky_distance_prediction(void);

// ---------------------------------- END FOR COMPETITION PURPOUSE ---------------------------------------

#endif
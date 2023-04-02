#ifndef USER_CLASSES
#define USER_CLASSES

#include <stdint.h>
#include <stdlib.h>
#include "stdbool.h"
#include <math.h>

#include "defines.h"
#include "helper_funcs.h"

typedef struct Base{
	uint16_t xCoord;
	uint16_t yCoord;
} cBase;

typedef struct Board{
	cBase base;
	uint8_t length;
	void (*displayBoard)(struct Board board);
} cBoard;

typedef struct Target {
	cBase base;
	uint8_t size;
	bool isActive;
	void (*displayTarget)(uint32_t color, struct Target *this);
	bool (*isWithingRange)(struct Target *targetA, struct Target *targetB);
}cTarget;

typedef struct Ball {
	struct Base base;
	uint16_t rad;
	int16_t xSpeed;
	int16_t ySpeed;
	void (*displayBall)(struct Ball ball);
} cBall;

// Returns true if the two targets overlap
bool isWithingRange(struct Target *targetA, struct Target *targetB) {
	if( sqrt(((targetA->base.xCoord-targetB->base.xCoord)
		    *(targetA->base.xCoord-targetB->base.xCoord))
		    +((targetA->base.yCoord-targetB->base.yCoord)
		    *(targetA->base.yCoord-targetB->base.yCoord)))
		    < (sqrt(2) * targetA->size)){
		    	return true;
	}
	else
		return false;
}

// Displays plot
void displayBoard(cBoard board) {
	BSP_LCD_DrawHLine(board.base.xCoord, board.base.yCoord, board.length);
}

// Displays ball
void displayBall(cBall ball) {
	BSP_LCD_FillCircle(ball.base.xCoord, ball.base.yCoord, ball.rad);
}

// displays target
void displayTarget(uint32_t color, struct Target *this)
{
    uint32_t *ptr = (uint32_t*) (FRAME_BUF_1_ADDRESS);
    uint8_t size=this->size;

    if (size & 1)
    {
        return;
    }
    size >>= 1;
    uint16_t startX = this->base.xCoord - size, startY = this->base.yCoord - size,
             endX = this->base.xCoord + size, endY = this->base.yCoord + size;
    if (startX > LCD_WIDTH || startY > LCD_HEIGHT || endX > LCD_WIDTH || endY > LCD_HEIGHT)
    {
        return;
    }

    uint16_t i, j;
    for (i = startY; i <= endY; ++i)
    {
        for (j = startX; j <= endX; ++j)
        {
            if (i == 0)
            {
                *(ptr + j) = color;
            }
            else
            {
                *(ptr + (i - 1) * LCD_WIDTH + j) = color;
            }
        }
    }
}

// Creates, initializes and returns pointer to new cBall object
cBall* newBall() {

	cBall* newBall = malloc(sizeof(cBall));
	newBall->base.xCoord = generate_random_position(20, 220);
	newBall->base.yCoord = generate_random_position(170, 210);
	newBall->rad = BALL_RAD;

	newBall->xSpeed = 1 * pow(-1, generate_random_position(1, 2));
	newBall->ySpeed = 1 * pow(-1, generate_random_position(1, 2));

	newBall->displayBall = &displayBall;

	return newBall;
}

// Creates, initializes and returns pointer to new cTarget object
cTarget* newTarget() {

	cTarget* newTarget = malloc(sizeof(cTarget));
	newTarget->base.xCoord = generate_random_position(TARGET_X_LIMIT_1, TARGET_X_LIMIT_2);
	newTarget->base.yCoord = generate_random_position(TARGET_Y_LIMIT_1, TARGET_Y_LIMIT_2);
	newTarget->size = TARGET_SIZE;
	newTarget->isActive = true;
	newTarget->displayTarget=&displayTarget;
	newTarget->isWithingRange=&isWithingRange;

	return newTarget;
}

// Creates, initializes and returns pointer to new cBoard object
cBoard* newBoard() {

	cBoard* newBoard = malloc(sizeof(cBoard));

	newBoard->base.xCoord = BOARD_X;
	newBoard->base.yCoord = BOARD_Y;
	newBoard->length = BOARD_LEN;
	newBoard->displayBoard = &displayBoard;

	return newBoard;
}

#endif

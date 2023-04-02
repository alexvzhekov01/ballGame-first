#ifndef USER_DEFINES
#define USER_DEFINES

// Frame buffer defines
#define FRAME_BUF_1         	1           			///< first frame buffer is the integer 1
#define FRAME_BUF_2         	2           			///< second frame buffer is the integer 2
#define FRAME_BUF_1_ADDRESS 	0xD0000000  			///< start address of first frame buffer's pixels
#define FRAME_BUF_2_ADDRESS	 	0xD0100000  			///< start address of second frame buffer's pixels (no overlap)

// LCD size defines
#define LCD_WIDTH				240                		///< screen width
#define LCD_HEIGHT 				320                		///< screen height
#define LCD_OFFSET 				5						///< edges of the play area

// Game object and background color defines
#define OBJECT_COLOR 			LCD_COLOR_GREEN			///< color of all objects
#define TARGET_COLOR 			LCD_COLOR_RED			///< color of targets
#define BACKGROUND_COLOR  		LCD_COLOR_BLACK			///< color of background
#define SUBTEXT_COLOR			LCD_COLOR_YELLOW		///< color of subtext in start screen

// Initial game data
#define INITIAL_TARGETS			5						///< initial number of targets
#define INITIAL_LIVES			3						///< initial number of lives

// Board parameters
#define BOARD_X					80						///< board X coordinate
#define BOARD_Y					300						///< board Y coordinate
#define BOARD_LEN				80						///< board length

// Target spawn limits and size
#define TARGET_X_LIMIT_1		10						///< target spawn area, limit left
#define TARGET_X_LIMIT_2		225						///< target spawn area, limit right
#define TARGET_Y_LIMIT_1		30						///< target spawn area, limit top
#define TARGET_Y_LIMIT_2		160						///< target spawn area, limit bottom
#define TARGET_SIZE				20						///< target size

// Ball spawn limits, speed variation and radius
#define BALL_X_LIMIT_1			20						///< ball spawn area, limit left
#define BALL_X_LIMIT_2			220						///< ball spawn area, limit left
#define BALL_Y_LIMIT_1			250						///< ball spawn area, limit left
#define BALL_Y_LIMIT_2			280						///< ball spawn area, limit left
#define BALL_RAD				7						///< ball radius
#define BALL_SPEED_LIMIT_1		1						///< lowest ball speed
#define BALL_SPEED_LIMIT_2		2						///< highest ball speed

#endif

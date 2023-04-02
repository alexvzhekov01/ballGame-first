#ifndef LCD_FUNCS
#define LCD_FUNCS

// Fills the frame buffer  with color
void fillWithColorLCD(uint8_t frameBufIndex, uint32_t color) {

	int size = LCD_WIDTH * LCD_HEIGHT;
	uint32_t *ptr;

	if (frameBufIndex == FRAME_BUF_1) {

		ptr = (uint32_t*) (FRAME_BUF_1_ADDRESS);
	} else if (frameBufIndex == FRAME_BUF_2) {

		ptr = (uint32_t*) (FRAME_BUF_2_ADDRESS);
	} else {

		return;
	}

	while (size--) {

		*ptr = color;
		ptr++;
	}
}

// Sets new frame buffer
void setActiveFrameBufLCD(uint8_t frameBufIndex) {

	if (frameBufIndex == FRAME_BUF_1) {

		BSP_LCD_SetLayerAddress(0, FRAME_BUF_1_ADDRESS);
	} else if (frameBufIndex == FRAME_BUF_2) {

		BSP_LCD_SetLayerAddress(0, FRAME_BUF_2_ADDRESS);
	}
}

#endif

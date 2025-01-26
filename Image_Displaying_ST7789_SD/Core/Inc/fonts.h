#define FONT_WIDTH 	   	   28
#define FONT_HEIGHT	   	   34
#define FIRST_CHARCODE 	   32
#define CHAR_COLBYTESNUM   (FONT_HEIGHT % 8 == 0 ? (FONT_HEIGHT / 8) : (FONT_HEIGHT / 8) + 1)
#define CHAR_ALLIGNROWSNUM CHAR_COLBYTESNUM * 8
#define CHAR_BYTESNUM  	   (FONT_WIDTH * CHAR_COLBYTESNUM + 1)
#define MAX_ROWCOUNT	   (ST7789_HEIGHT / FONT_HEIGHT)

extern const unsigned short Ink_Free28x34[];

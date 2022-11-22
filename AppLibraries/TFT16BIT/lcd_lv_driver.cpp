
#if 0

#include "lcd_lv_driver.h"
#include "TFT16BIT.h"
#include "stdlib.h"

void LCD_flush(lv_disp_drv_t *Disp_Driver, const lv_area_t *Disp_Area,  lv_color_t * Color_Buffer){
	TFT_SetWindow(Disp_Area->x1, Disp_Area->y1, Disp_Area->x2, Disp_Area->y2);
//	uint32_t size = (Disp_Area->x2 - Disp_Area->x1 + 1) * (Disp_Area->y2 - Disp_Area->y1 + 1);
	TFT_PushData((uint16_t *)Color_Buffer, (Disp_Area->x2 - Disp_Area->x1 + 1) * (Disp_Area->y2 - Disp_Area->y1 + 1));
	lv_disp_flush_ready(Disp_Driver);
}

#endif





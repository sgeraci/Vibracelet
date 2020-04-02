/**
 * Copyright (c) 2017-2018 Tara Keeling
 * 
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <esp_attr.h>
#include "ssd1306.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"
#include "ssd1306_default_if.h"
#include "DrawTask.h"

void ScreenSetup( struct SSD1306_Device* DisplayHandle, const struct SSD1306_FontDef* Font ) {
  SSD1306_Clear( DisplayHandle, SSD_COLOR_BLACK );
  SSD1306_SetFont( DisplayHandle, Font );
}

void DrawText( struct SSD1306_Device* DisplayHandle, const char* Text ) {
    SSD1306_FontDrawAnchoredString( DisplayHandle, TextAnchor_Center, Text, SSD_COLOR_WHITE );
    SSD1306_Update( DisplayHandle );
}

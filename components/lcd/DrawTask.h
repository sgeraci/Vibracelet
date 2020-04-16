#ifdef __cplusplus
extern "C" {
#endif


void ScreenSetup( struct SSD1306_Device* DisplayHandle, const struct SSD1306_FontDef* Font );
void DrawText( struct SSD1306_Device* DisplayHandle, const char* Text );

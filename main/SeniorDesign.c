#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "driver/dac.h"
#include "driver/i2s.h"
#include "esp_system.h"
#include "ssd1306.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"
#include "ssd1306_default_if.h"
#include "DrawTask.h"
#include <math.h>
#include <stdbool.h>
#include <unistd.h>



/****************************DESIGN CONCEPT*******************************
 * This code has been developed from a code testing peripherals needed by
 * the ESP32. Now the code is entering testing of all the peripherals. A
 * circuit setup is required and algorithm specifics will be indeterminate
 * until testing starts. 								SUMMARY:
 * The code sets up the peripherals needed on select pins. I2S then
 * records microphone data and prints it to the terminal. This data is
 * used to verify functionality of the microphone and successful
 * interfacing with the microprocessor. The output on a gpio pin upon
 * a software trigger has also been verified.
 ***************************PERIPHERALS***********************************
 * (1) GPIO output will be used for the vibrate motor
 * (1) GPIO input will be used for an "snooze" button
 * (3) pins used for I2S connection for microphone interface
 * 	(1) Clock Pin
 *	(1) Word Select Pin
 *	(1) Data Pin
 * (2) pins will be used for I2C connection to LED screen
 *	(1) Clock Pin
 *	(1) Data Pin
 ***********************************NOTES********************************
 * Data from a microphone is read through I2S [24-bit (last 6 bits are
 * 0's], MSB, 2's complement). Microphone has built in filtering and
 * decimation for DSP so the algorithm lighter. Only one microphone is
 * used so the data is read on the left channel:
 * 	USING THE WSCLK FOR THE MICROPHONE SELECT MAY BE A VIABLE SOLUTION SO
 *	MICROPHONE OUPUTS ON LEFT AND RIGHT CHANNEL
 * Microphone data was printed to terminal to export to MATLAB and verify
 * with plot (input whistle).
 *
 * The vibrate motor has been testing using shop BJT successfully. Smaller
 * parts will be needed for PCB implementation.
 ***********************************ITEMS*********************************
 * -Write algorithm to determine min and max of input to determine whether
 * data is viable for FFT comparison or if volume threshold should be
 * triggered.
 * 		-Decide how to implement volume and custom-sound trigger
 * 		-Can be calculated based on amplitude of trigger sounds or trigger
 * 			sounds can be normalized in order for peak-to-peak value to be
 * 			equal to the max value
 * 		=Should volume trigger be hardware or software?
 * 		-If FFt is viable, then tolerance has to be established so
 * 			something is not always triggered (based on testing).
 * -FFT data and and decide length of sample for comparison to trigger
 * sounds.
 * -Design structures for triggers to hold data about the following:
 * 		-Alarm Name (LED Display)
 * 		-Matrix of FFT Coefficients
 *
 * Input whistle into microphone and FFt it. Use the minimum samples
 * 		possible and set up the code to trigger based on the MATLAB code.
 * 		Once triggered, the vibrate should go off. Then move on to
 * 		incorporating the snooze function.
 * 	-After this is established, the method of sampling can be determined
 * 	 and dual core processes can be utilized to help with computational
 * 	 speed.
 ********************************END*************************************/
 #define USE_I2C_DISPLAY
#if defined USE_I2C_DISPLAY
    static const int I2CDisplayAddress = 0x3C;    //WROTE these first so that if the defaults establsihed in the #if use i2c are impacting your code we can overwrite those assignments in the next part of the code
    static const int I2CDisplayWidth = 128;
    static const int I2CDisplayHeight = 32;
    static const int I2CResetPin = -1;
    struct SSD1306_Device I2CDisplay;
#endif

void ScreenSetup( struct SSD1306_Device* DisplayHandle, const struct SSD1306_FontDef* Font );
void DrawText( struct SSD1306_Device* DisplayHandle, const char* Text );

bool DefaultBusInit( void ) {
    #if defined USE_I2C_DISPLAY
        assert( SSD1306_I2CMasterInitDefault( ) == true );
        assert( SSD1306_I2CMasterAttachDisplayDefault( &I2CDisplay, I2CDisplayWidth, I2CDisplayHeight, I2CDisplayAddress, I2CResetPin ) == true );
    #endif

    return true;
}

// TEST MACROS ****************//
#define GPIO_INPUT_REC 			5
//#define GPIO_INPUT_UNK		2
//*****************************//


#define GPIO_OUTPUT_VIBRATE		23
#define GPIO_INPUT_SNOOZE		18
#define SAMPLE_RATE     		(44100)
#define NSAMPLES 				((1/100) * SAMPLE_RATE) 			//10-ms
#define WAVE_FREQ_HZ    		(100)
#define PI              		(3.14159265)
#define I2S_BCK_IO      		25
#define I2S_WS_IO       		32
#define I2S_DI_IO      			33
#define I2S_DO_IO       		(-1)
#define SAMPLE_PER_CYCLE 		(SAMPLE_RATE/WAVE_FREQ_HZ)

static xQueueHandle snooze_evt_queue = NULL;
static xQueueHandle rec_evt_queue = NULL;
static xQueueHandle i2s_queue = NULL;

TaskHandle_t listen_handle = NULL;
TaskHandle_t snooze_handle = NULL;
TaskHandle_t record_handle = NULL;

static void IRAM_ATTR snooze_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(snooze_evt_queue, &gpio_num, NULL);
}

static void IRAM_ATTR rec_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	//printf("hey look im in the interrupt handler!\n\r");
	xQueueSendFromISR(rec_evt_queue, &gpio_num, NULL);

}
void ScreenSetup( struct SSD1306_Device* DisplayHandle, const struct SSD1306_FontDef* Font ) {
  SSD1306_Clear( DisplayHandle, SSD_COLOR_BLACK );
  SSD1306_SetFont( DisplayHandle, Font );
}
void DrawText( struct SSD1306_Device* DisplayHandle, const char* Text ) {
    SSD1306_FontDrawAnchoredString( DisplayHandle, TextAnchor_Center, Text, SSD_COLOR_WHITE ); //wasnt recognizing some functions in a header so i just hard coded for now 
    SSD1306_Update( DisplayHandle );

}

static void listen(void * arg)
{
	uint32_t data32;
	uint16_t data16[NSAMPLES];
	size_t bytes_read;
	int n = 0;
	for(;;) {
		esp_task_wdt_reset();

		i2s_read(I2S_NUM_0, &data32, sizeof(data32), &bytes_read, 0);		//24-bit stored into 32-bit variable
//		printf("%x\n",data32);
		data16[n] = (uint16_t) data32 >> 16;
		//data32 = ~data32;
		//data32 = data32 + (1 >> 14);
		//data16[n] = (uint16_t) (data32 >> 16);		n++;
//		printf("%x\n\n",data16[n]);
		if(n>NSAMPLES)		n = 0;
	}
}


static void snooze(void * arg)
{
	uint32_t io_num;
	
	for(;;){
		xQueueReceive(snooze_evt_queue, &io_num, portMAX_DELAY);
		printf("SNOOZE PRESSED!\n");
	gpio_set_level(GPIO_OUTPUT_VIBRATE, 1);	
		gpio_set_level(GPIO_OUTPUT_VIBRATE, 0);							//turn off vibrate
		SSD1306_DisplayOff(&I2CDisplay );		//turn off led screen
	}

}

static void record(void * arg)
{
	uint32_t io_num;
	for(;;){
		xQueueReceive(rec_evt_queue, &io_num, portMAX_DELAY);
		printf("I am going to record a basis sound now\n\r");
	}
}

static void setup(void)
{
	//set up peripherals
	//set up GPIO for vibrate motor output
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;									//disable interrupt
	io_conf.mode = GPIO_MODE_OUTPUT;											//set as output mode
	io_conf.pin_bit_mask = (1ULL << GPIO_OUTPUT_VIBRATE);						//bit mask of the pin
	io_conf.pull_down_en = 0;													//disable pull-down mode
	io_conf.pull_up_en = 0;														//disable pull-up mode
	gpio_config(&io_conf);		 												//configure GPIO with the given settings

	//set up GPIO for acknowledge input
	io_conf.pull_up_en = 1;														//enable pull-up
	io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;									//interrupt of falling edge
	io_conf.pin_bit_mask = (1ULL << GPIO_INPUT_SNOOZE);							//bit mask of the pin
	io_conf.mode = GPIO_MODE_INPUT;    											//set as input mode
	gpio_config(&io_conf);														//configure GPIO with the given settings

			
	// USING PUSHBUTTONS FOR TESTING //
	// THIS WILL BECOME CONTROLLED VIA BT LATER //
	// NEED TO SETUP STRUCTURE OF FUNCTIONS //
	io_conf.pin_bit_mask = (1ULL << GPIO_INPUT_REC);
	// ****************************************** //	 
 
	gpio_config(&io_conf);														//configure GPIO with the given settings
	
	//create a queue to handle events from isr:
	//snooze and record basis sound
	snooze_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	rec_evt_queue = xQueueCreate(5, sizeof(uint32_t));
	
	//install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);

	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_INPUT_SNOOZE, snooze_isr_handler, (void*) GPIO_INPUT_SNOOZE);
	

	gpio_isr_handler_add(GPIO_INPUT_REC, rec_isr_handler, (void*) GPIO_INPUT_REC);


	//set up i2s input from microphone
	i2s_config_t i2s_config = {
		.mode = I2S_MODE_MASTER | I2S_MODE_RX,						            //set ESP as master and recieve
		.sample_rate = SAMPLE_RATE,												//set sample rate
		.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,							//32 bits per sample
		.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,                          	//1 channel, left
		.communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,	//default format with msb first
		.dma_buf_count = 2,
		.dma_buf_len = 8,
		.use_apll = true,
		.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1

	};
	i2s_pin_config_t pin_config = {
		.bck_io_num = I2S_BCK_IO,
		.ws_io_num = I2S_WS_IO,
		.data_out_num = I2S_DO_IO,
		.data_in_num = I2S_DI_IO
	};
	i2s_queue = xQueueCreate(10, sizeof(uint32_t));
	i2s_driver_install(I2S_NUM_0, &i2s_config, 10, i2s_queue);							//configure i2s with the given settings
	i2s_set_pin(I2S_NUM_0, &pin_config);											//set i2s pins

}



void app_main(void)
{
	setup();
	//test i2s
	//bool trigger = 0;
	if ( DefaultBusInit( ) == true ) {
		const char* text = "START BITCH";
        #if defined USE_I2C_DISPLAY
			ScreenSetup( &I2CDisplay, &Font_droid_sans_mono_7x13);
			DrawText( &I2CDisplay, text);
        #endif
        printf( "Done!\n" );
    }
	gpio_set_level(GPIO_OUTPUT_VIBRATE, 1);									//turn off vibrate

	xTaskCreate(listen, "listen", 2048, NULL, tskIDLE_PRIORITY, &listen_handle);
	xTaskCreate(snooze, "snooze", 2048, NULL, 1, &snooze_handle);
	xTaskCreate(record, "record", 2048, NULL, 2, &record_handle);

	esp_task_wdt_add(listen_handle);

	//test motor
	bool trigger = 0;
	int cnt = 0;
	gpio_set_level(GPIO_OUTPUT_VIBRATE, 1);									//turn off vibrate
}





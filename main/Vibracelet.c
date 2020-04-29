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
#include "esp_dsp.h"
#include <math.h>
#include <stdbool.h>
#include <unistd.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"




// TEST MACROS ****************//
#define GPIO_INPUT_TRIG 		16
#define GPIO_INPUT_BT			17
#define GPIO_INPUT_PINS 		(1ULL << GPIO_INPUT_BT) | (1ULL << GPIO_INPUT_TRIG)
//*****************************//


#define GPIO_OUTPUT_VIBRATE		21
#define GPIO_INPUT_SNOOZE		4
#define SAMPLE_RATE     		(44100)
#define NSAMPLES 				4096			//~10-ms
#define PI              		(3.14159265)
#define I2S_BCK_IO      		32
#define I2S_WS_IO       		25
#define I2S_DI_IO      			33
#define I2S_DO_IO       		(-1)


#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "Vibracelet"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_DATA    /*Choose show mode: show data or speed*/


struct SignalFFT {
	char name[10];
	int16_t RealImagFFTData[NSAMPLES * 2];
};

static xQueueHandle snooze_evt_queue = NULL;
static xQueueHandle bt_evt_queue = NULL;
static xQueueHandle trig_evt_queue = NULL;
static xQueueHandle i2s_queue = NULL;
static xQueueHandle dsproc_queue = NULL;

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

TaskHandle_t listen_handle = NULL;
TaskHandle_t snooze_handle = NULL;
TaskHandle_t bt_handle = NULL;
TaskHandle_t trig_handle = NULL;
TaskHandle_t dsproc_handle = NULL;

//struct SignalFFT BasisSignalsFFT[5];
//struct SignalFFT InputSignalFFT;

int16_t data16;

float * table_buff = NULL;
int arraynum = 0;


#define USE_I2C_DISPLAY
#if defined USE_I2C_DISPLAY
    static const int I2CDisplayAddress = 0x3C;    
	static const int I2CDisplayWidth = 128;
    static const int I2CDisplayHeight = 32;
    static const int I2CResetPin = -1;
    struct SSD1306_Device I2CDisplay;
#endif


bool DefaultBusInit( void ) {
    #if defined USE_I2C_DISPLAY
        assert( SSD1306_I2CMasterInitDefault( ) == true );
        assert( SSD1306_I2CMasterAttachDisplayDefault( &I2CDisplay, I2CDisplayWidth, I2CDisplayHeight, I2CDisplayAddress, I2CResetPin ) == true );
    #endif
    return true;
}
static void IRAM_ATTR snooze_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(snooze_evt_queue, &gpio_num, NULL);
}

static void IRAM_ATTR rec_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(bt_evt_queue, &gpio_num, NULL);
}

static void IRAM_ATTR trig_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(trig_evt_queue, &gpio_num, NULL);
}

void ScreenSetup( struct SSD1306_Device* DisplayHandle, const struct SSD1306_FontDef* Font ) 
{
  SSD1306_Clear( DisplayHandle, SSD_COLOR_BLACK );
  SSD1306_SetFont( DisplayHandle, Font );
}

void DrawText( struct SSD1306_Device* DisplayHandle, const char* Text ) 
{
    SSD1306_FontDrawAnchoredString( DisplayHandle, TextAnchor_Center, Text, SSD_COLOR_WHITE ); //wasnt recognizing some functions in a header so i just hard coded for now 
    SSD1306_Update( DisplayHandle );
}

static void listen(void * arg)
{
	int32_t data32;
	size_t bytes_read;
	esp_task_wdt_add(NULL);
	for(;;) {
		esp_task_wdt_reset();
		i2s_read(I2S_NUM_0, &data32, sizeof(data32), &bytes_read, 0);		//24-bit stored into 32-bit variable
		//data32 = (data32 >> 14);											//invert
		//data32 = ~data32 & ~0xFFFC0000;										//make the msbs zero after inverting
		//data32 = data32 + 1;
		//TAKING OUT TO SEE IF THIS FIXES THE ISSUE WITH THE OUTPUT FOR MIC DATA
		//data32 = data32 >> 2;
		//data16[n] = (uint16_t *) data32;
		data16 = (int16_t) (data32 >> 16);// >> 16);

//		printf("%x\r\n", data16);
		xTaskNotify(dsproc_handle, 0, eNoAction);
	}
}




static void snooze(void * arg)
{
	uint32_t io_num;
	
	for(;;){
		xQueueReceive(snooze_evt_queue, &io_num, portMAX_DELAY);
		printf("SNOOZE PRESSED!\n");
		gpio_set_level(GPIO_OUTPUT_VIBRATE, 0);							//turn off vibrate
		SSD1306_Clear(&I2CDisplay, SSD_COLOR_BLACK);
		SSD1306_Update(&I2CDisplay);
	}

}

static void bt_data(void * arg)
{
	uint32_t io_num;
	for(;;){
		xQueueReceive(bt_evt_queue, &io_num, portMAX_DELAY);
		SSD1306_Clear(&I2CDisplay, SSD_COLOR_BLACK);
		SSD1306_Update(&I2CDisplay);
		DrawText(&I2CDisplay, "Bluetooth...");
		printf("bluetooth button pressed...\r\n");

		esp_err_t ret = nvs_flash_init();
		if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
			ESP_ERROR_CHECK(nvs_flash_erase());
			ret = nvs_flash_init();
		}
		ESP_ERROR_CHECK( ret );

		ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

		esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
		
		

		if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
			ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
			return;
		}


		if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
			ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
			return;
		}

		if ((ret = esp_bluedroid_init()) != ESP_OK) {
			ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
			return;
		}

		if ((ret = esp_bluedroid_enable()) != ESP_OK) {
			ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
			return;
		}

//		if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
//			ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
//			return;
//		}

//		if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
//			ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
//			return;
//		}

		if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
			ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
			return;
		}

		
		

	#if (CONFIG_BT_SSP_ENABLED == true)
		/* Set default parameters for Secure Simple Pairing */
		esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
		esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
		esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
	#endif

		/*
		 * Set default parameters for Legacy Pairing
		 * Use variable pin, input pin code when pairing
		 */
		esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
		esp_bt_pin_code_t pin_code;
		esp_bt_gap_set_pin(pin_type, 0, pin_code);
		}
}

static void trigger(void * arg)
{
	uint32_t io_num;
	for(;;){
		xQueueReceive(trig_evt_queue, &io_num, portMAX_DELAY);
		printf("trigger button pressed...\r\n");
		SSD1306_Clear(&I2CDisplay, SSD_COLOR_BLACK);
		SSD1306_Update(&I2CDisplay);
		DrawText(&I2CDisplay, "Triggered");
		gpio_set_level(GPIO_OUTPUT_VIBRATE, 1);							//turn on vibrate
		
	}
}

static void dsproc(void * arg)
{	
	esp_task_wdt_add(NULL);
	uint32_t io_num;
	for(;;) 
	{
		esp_task_wdt_reset();
//		InputSignalFFT.RealImagFFTData[arraynum * 2] = data16;		//storing time domain data into real (evens)
//		printf("%d\r\n%d\r\n", InputSignalTime[arraynum],InputSignalTime[arraynum-1]);
		arraynum++;
		if(arraynum >= NSAMPLES) 
		{
//			dsps_fft2r_sc16_ae32(InputSignalFFT.RealImagFFTData, NSAMPLES);
			arraynum = 0;
			for(int i = 0; i < NSAMPLES/2; i++){
//				printf("%d + j%d    ", InputSignalFFT.RealImagFFTData[i*2],InputSignalFFT.RealImagFFTData[(i*2)+1]);
			}
//			printf("\r\n\r\n\r\n");


		}



	}
}


static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
        
        
    case ESP_SPP_DATA_IND_EVT:
//        gettimeofday(&time_new, NULL);
//        data_num += param->data_ind.len;
//        store_data(param);
        break;
    
    
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
        
        
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
    
    
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
//        gettimeofday(&time_old, NULL);
        break;
    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
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
	io_conf.pin_bit_mask = GPIO_INPUT_PINS;
	// ****************************************** //	 
 
	gpio_config(&io_conf);														//configure GPIO with the given settings
	
	//create a queue to handle events from isr:
	//snooze and record basis sound
	snooze_evt_queue = xQueueCreate(5, sizeof(uint32_t));
	bt_evt_queue = xQueueCreate(5, sizeof(uint32_t));
	trig_evt_queue = xQueueCreate(5, sizeof(uint32_t));

	
	//install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);

	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_INPUT_SNOOZE, snooze_isr_handler, (void*) GPIO_INPUT_SNOOZE);
	

	// USING PUSHBUTTONS FOR TESTING //
	// THIS WILL BECOME CONTROLLED VIA BT LATER //
	// NEED TO SETUP STRUCTURE OF FUNCTIONS //
	gpio_isr_handler_add(GPIO_INPUT_BT, rec_isr_handler, (void*) GPIO_INPUT_BT);
	gpio_isr_handler_add(GPIO_INPUT_TRIG, trig_isr_handler, (void*) GPIO_INPUT_TRIG);
	// ****************************************** //	 

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
	i2s_queue = xQueueCreate(5, sizeof(uint32_t));
	i2s_driver_install(I2S_NUM_0, &i2s_config, 1, i2s_queue);						//configure i2s with the given settings
	i2s_set_pin(I2S_NUM_0, &pin_config);											//set i2s pins



	// Set up I2C for LED screen (pulled from library)
	if ( DefaultBusInit( ) == true ) {
		const char* text = "PROGRAM START";
		#if defined USE_I2C_DISPLAY
			ScreenSetup( &I2CDisplay, &Font_droid_sans_mono_7x13);
			DrawText( &I2CDisplay, text);
		#endif
	}
	
	dsproc_queue = xQueueCreate(5, sizeof(uint32_t));
	dsps_fft2r_init_sc16(NULL, CONFIG_DSP_MAX_FFT_SIZE);
	
}

void app_main(void)
{
	setup();
	struct SignalFFT * BasisSignalsFFT[5];
	for(int i = 0; i <= 4; i++){
		BasisSignalsFFT[i] = (struct SignalFFT*) malloc(sizeof(struct SignalFFT));
	
	}
	struct SignalFFT * InputSignalFFT = (struct SignalFFT*) malloc(sizeof(struct SignalFFT));
	xTaskCreatePinnedToCore(listen, "listen", 2048, NULL, tskIDLE_PRIORITY, &listen_handle, 0);
	xTaskCreate(snooze, "snooze", 1024, NULL, 1, &snooze_handle);
	xTaskCreate(bt_data, "record", 2048, NULL, 3, &bt_handle);
	xTaskCreate(trigger, "trigger", 1024, NULL, 2, &trig_handle);
	xTaskCreatePinnedToCore(dsproc, "dsproc", 2048, NULL, tskIDLE_PRIORITY, &dsproc_handle, 1);

}





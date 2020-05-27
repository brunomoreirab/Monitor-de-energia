#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <math.h>

#include "esp_system.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_wifi.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"
#include "cJSON.h"

#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "cJSON.h"

// ---------------------------------- DEFINES ---------------------------------- //    
// GPIOs ----------------------------------------------------------------------- //
#define 	LED1	        GPIO_NUM_2
#define 	LED2	        GPIO_NUM_17
#define 	LED3	        GPIO_NUM_16

// Sampling ----------------------------------------- // 
#define     NO_OF_POINTS    100             // Number of points read
#define     NO_OF_SAMPLES   10              // Number of samples per point
#define		NO_OF_CHANNELS	6               // Number of channels read

// Timer -------------------------------------------- //
#define		CLOCK			80000000        // ESP32 clock
#define		f_e				60				// Electrical frequency

#define     T0_DIVIDER      400             // Reads 100 points in 3 cycles (2Ksps)     
#define     T0_OVERFLOW     100             // f = 80M/(divider*overflow)

#define     T1_DIVIDER      40000           // 5 seconds interval
#define     T1_OVERFLOW     20000           // t = (divider*overflow)/80M

// ADC ---------------------------------------------- //
#define     DEFAULT_VREF    1100
#define     VREF            3.3

// Wifi --------------------------------------------- //
#define EXAMPLE_WIFI_SSID "NET_2G 102C"
#define EXAMPLE_WIFI_PASS "001B2G3J"

// ---------------- GLOBAL VARIABLES ---------------- //
// Timer config ------------------------------------- //
typedef struct
{
	timer_idx_t timer_ID;
	TaskHandle_t Taskptr;	
} Timer_Spec_t;

TaskHandle_t Handle_T0;
TaskHandle_t Handle_T1;

// ADC config --------------------------------------- //
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel[NO_OF_CHANNELS] = {ADC_CHANNEL_0, ADC_CHANNEL_3, ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_4, ADC_CHANNEL_5};
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

// Sampling and analysis -------------------------------------------------------- //
float ratio[NO_OF_CHANNELS] = {387.93, 15, 15, 15, 15, 15};

uint16_t msgBuffer_raw[NO_OF_CHANNELS][NO_OF_POINTS];
uint32_t msgBuffer_sum[NO_OF_CHANNELS] = {0};

float V_PHA1[NO_OF_POINTS];
float V_PHA2[NO_OF_POINTS];
float I_PHA1[NO_OF_POINTS];
float I_PHA2[NO_OF_POINTS];
float I_CIR1[NO_OF_POINTS];
float I_CIR2[NO_OF_POINTS];
float I_CIR3[NO_OF_POINTS];

float sample_time, sample_angle;
uint16_t points2shift;
bool ready2send;

// --------------------------- Functions declaration --------------------------- // 
static void check_efuse(void);
static void print_char_val_type(esp_adc_cal_value_t val_type);
void ADC_config(void);
void GPIO_config(void);
void Timer_Init(timer_group_t timer_group, timer_idx_t timer_ID, uint32_t timer_divider, uint64_t Timer_overflow, bool timer_reload,TaskHandle_t *Taskptr);
void IRAM_ATTR timer_group0_isr(void *para);
void IRAM_ATTR timer_group1_isr(void *para);
void Handler_T0(void *para);
void Handler_T1(void *para);
float power_calculator(float V[], float I[]);
float RMS_calculator(float I[]);
static esp_err_t event_handler(void *ctx, system_event_t *event);
static esp_err_t event_handler(void *ctx, system_event_t *event);
void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen, IoT_Publish_Message_Params *params, void *pData);
void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data);
void aws_iot_task(void *param);
static void initialise_wifi(void);

// Wifi ------------------------------------------------------------------------- //
static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

// AWS IoT Core ----------------------------------------------------------------- //
static const char *TAG = "subpub";

extern const uint8_t aws_root_ca_pem_start[] asm("_binary_src_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_src_aws_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_src_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_src_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_src_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_src_private_pem_key_end");

char HostAddress[255] = "a10lekoccab51o-ats.iot.sa-east-1.amazonaws.com";
uint32_t port = 8883;

void app_main()
{
    // Inicializa a memoria NVS
    NVS_config();

    // Inicializa a conexao Wi-FI
    initialise_wifi();

    // Configura os ADCs
    ADC_config();

    // Configura as GPIOs
    GPIO_config();

    // Cria as tasks
	xTaskCreate(Handler_T0,    "Handler_TIMER0",    2048,   NULL,   3,  &Handle_T0);
	xTaskCreate(Handler_T1,    "Handler_TIMER1",    2048,   NULL,   4,  &Handle_T1);
    xTaskCreate(&aws_iot_task, "AWS_IoT",     36864,  NULL,   5,  NULL);

    // Inicializa os timers
    Timer_Init(TIMER_GROUP_0, TIMER_0, T0_DIVIDER, T0_OVERFLOW, TIMER_AUTORELOAD_EN, &Handle_T0);
	Timer_Init(TIMER_GROUP_0, TIMER_1, T1_DIVIDER, T1_OVERFLOW, TIMER_AUTORELOAD_EN, &Handle_T1);
}

void NVS_config();
{
    // Inicializa a memoria NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
}

void constant_calculations(void)
{
    // Calcula quantos pontos devem ser deslocados para extrapolar a segunda fase
	sample_time = (float)(T0_DIVIDER*T0_OVERFLOW)/CLOCK;
	sample_angle = (float)360*sample_time*f_e;
	points2shift = round(30/sample_angle);
}

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

void ADC_config(void)
{
    check_efuse();

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel[0] | channel[3] | channel[4] | channel[5] | channel[6] | channel[7], atten);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
}

void GPIO_config(void)
{
    gpio_config_t io_conf;
	
    //Configura o descritor de Outputs (Leds).
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;  			            //Disables GPIO interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;            			            //Configures it as an output
    io_conf.pin_bit_mask = (1ULL<<LED1) | (1ULL<<LED2) | (1ULL<<LED3);   //Informs wich GPIO(s) will be configured
    gpio_config(&io_conf);                      			            //Configure(s) it/them
}

void Timer_Init(timer_group_t timer_group, timer_idx_t timer_ID, uint32_t timer_divider, uint64_t timer_overflow, bool timer_reload, TaskHandle_t *Taskptr)
{
	//define Struct for ISR Pointer
	Timer_Spec_t *Timer_Specs_p = calloc(sizeof(Timer_Spec_t), 1);
	Timer_Specs_p->Taskptr = *Taskptr;
	Timer_Specs_p->timer_ID = timer_ID;
	
	//Timer Init. Group0, Timer0
	timer_config_t timerConfig;
	timerConfig.divider = timer_divider;
	timerConfig.counter_dir = TIMER_COUNT_UP;
	timerConfig.counter_en = TIMER_PAUSE;
	timerConfig.alarm_en = TIMER_ALARM_EN;
	timerConfig.auto_reload = timer_reload;
	timerConfig.intr_type = TIMER_INTR_LEVEL;
	
	timer_init(timer_group, timer_ID, &timerConfig);
	timer_set_counter_value(timer_group, timer_ID, 0x00000000ULL);
	timer_set_alarm_value(timer_group, timer_ID, timer_overflow);
	timer_enable_intr(timer_group, timer_ID);
	
	
	if (timer_group == TIMER_GROUP_0)
	{
		timer_isr_register(timer_group, timer_ID, timer_group0_isr, (void *) Timer_Specs_p, ESP_INTR_FLAG_IRAM, NULL);	
	}
	else
	{
		timer_isr_register(timer_group, timer_ID, timer_group1_isr, (void *) Timer_Specs_p, ESP_INTR_FLAG_IRAM, NULL);	
	}

	timer_start(timer_group, timer_ID);		
}

void IRAM_ATTR timer_group0_isr(void *para)
{
	//save the data from the pointer
	Timer_Spec_t *Settings =  (Timer_Spec_t *) para;
	//Get interrupt status
	uint32_t intr_status = TIMERG0.int_st_timers.val;

	//Delete interrupt flags 
	if((intr_status & BIT(Settings->timer_ID)) && (Settings->timer_ID) == TIMER_0) 
	{
		TIMERG0.int_clr_timers.t0 = 1;
	}
		else if((intr_status & BIT(Settings->timer_ID)) && (Settings->timer_ID) == TIMER_1) 
	{
		TIMERG0.int_clr_timers.t1 = 1;
	}	
	
	//Reactivate alarm
	TIMERG0.hw_timer[Settings->timer_ID].config.alarm_en = TIMER_ALARM_EN;	
	
	//TaskNotify
	xTaskNotifyFromISR(Settings->Taskptr,0x00,eNoAction,NULL);
	portYIELD_FROM_ISR();
}

void IRAM_ATTR timer_group1_isr(void *para)
{	
	// Saves the data from the pointer
	Timer_Spec_t *Settings =  (Timer_Spec_t *) para;
	
	// Get interrupt status
	uint32_t intr_status = TIMERG1.int_st_timers.val;
	
	//Delete interrupt flags 
	if((intr_status & BIT(Settings->timer_ID)) && (Settings->timer_ID) == TIMER_0) 
	{
		TIMERG1.int_clr_timers.t0 = 1;
	}
	else if((intr_status & BIT(Settings->timer_ID)) && (Settings->timer_ID) == TIMER_1) 
	{
		TIMERG1.int_clr_timers.t1 = 1;
	}	
	
	// Reactivate alarm
	TIMERG1.hw_timer[Settings->timer_ID].config.alarm_en = TIMER_ALARM_EN;	
	
	//TaskNotify
	xTaskNotifyFromISR(Settings->Taskptr, 0x00, eNoAction, NULL);
	portYIELD_FROM_ISR();
	
}

void Handler_T0(void *para)
{
	uint16_t cnt = 0;
	uint16_t i, j, k;
    uint32_t adc_reading = 0;
    
	while (true)
	{
		// Semaforo que aguarda ser habilitado pela interrupcao
		xTaskNotifyWait(0x00, 0xffffffff, NULL, portMAX_DELAY);
		gpio_set_level(LED1, 1);

        // Indica qual canal esta sendo lido
		k = cnt / NO_OF_POINTS;
		// Indica o numero de amostras do mesmo ponto
        j = cnt % NO_OF_POINTS;

		// Ciclo que le os tres ciclos
        if(cnt < (NO_OF_POINTS*NO_OF_CHANNELS))
        {
			// Ciclo para ler 10x o mesmo ponto
            for (i = 0; i < NO_OF_SAMPLES; i++) 
            {
                adc_reading += adc1_get_raw((adc1_channel_t)channel[k]);
            }

            // Soma o valor de cada amostra a um buffer
            msgBuffer_raw[k][j] = adc_reading/NO_OF_SAMPLES;
			msgBuffer_sum[k] += msgBuffer_raw[k][j];

            // Indica que a leitura esta ocorrendo
            gpio_set_level(LED1, cnt%2);
            
            adc_reading = 0;
            cnt++;
        }

        else
        {
            // Indica que a leitura terminou
			gpio_set_level(LED1, 0);
            
            // Pausa o Timer0 ao final de todas as leituras
            timer_pause(TIMER_GROUP_0, TIMER_0);
            
            // Flag que indica que os dados estao prontos para serem enviados
            ready2send = 1;
            cnt = 0;
        }	
	}
}
	
void Handler_T1(void *para)
{
	while (true)
	{
        // Semaforo que aguarda ser habilitado pela interrupcao
		xTaskNotifyWait(0x00, 0xffffffff, NULL, portMAX_DELAY);

        // Reinicia o timer
		Timer_Init(TIMER_GROUP_0, TIMER_0, T0_DIVIDER, T0_OVERFLOW, TIMER_AUTORELOAD_EN, &Handle_T0);	
	}
}

float power_calculator(float V[], float I[])
{
	float power = 0, aux = 0;
	uint16_t i;

	for(i = 1; i <= NO_OF_POINTS; i++)
	{
		aux += (V[i]*I[i] + V[i-1]*I[i-1])/2;
	}

	power = aux/(NO_OF_POINTS - 1);
	return power;
}

float RMS_calculator(float I[])
{
	float RMS = 0;
	float aux = 0;
	uint16_t i;

	for(i = 1; i <= NO_OF_POINTS; i++)
	{
		aux += I[i]*I[i];
	}

	RMS = sqrt(aux/(NO_OF_POINTS));
	return RMS;
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        gpio_set_level(LED2, 0);
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }

    return ESP_OK;
}

void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData) 
{
    ESP_LOGI(TAG, "Subscribe callback");
    ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);
}

void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data) 
{
    ESP_LOGW(TAG, "MQTT Disconnect");
    IoT_Error_t rc = FAILURE;

    if(NULL == pClient) {
        return;
    }

    if(aws_iot_is_autoreconnect_enabled(pClient)) {
        ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
    } else {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if(NETWORK_RECONNECTED == rc) {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        } else {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}

void aws_iot_task(void *param) 
{
    IoT_Error_t rc = FAILURE;

    uint16_t j, k, j_aux, ID = 0;
    cJSON *root, *ph1, *ph2, *cir1, *cir2, *cir3;
	char *rendered;

    AWS_IoT_Client client;
    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

    IoT_Publish_Message_Params paramsQOS1;

    ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    mqttInitParams.enableAutoReconnect = false;
    mqttInitParams.pHostURL = HostAddress;
    mqttInitParams.port = port;

    mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
    mqttInitParams.pDeviceCertLocation = (const char *)certificate_pem_crt_start;
    mqttInitParams.pDevicePrivateKeyLocation = (const char *)private_pem_key_start;

    rc = aws_iot_mqtt_init(&client, &mqttInitParams);
    if(SUCCESS != rc) 
    {
        gpio_set_level(LED3, 0);
        ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
        abort();
    }

    gpio_set_level(LED2, 0);
    // Aguarda a conexao Wifi
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    gpio_set_level(LED2, 1);

    connectParams.keepAliveIntervalInSec = 10;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;
    
    // Configura o CLientID
    connectParams.pClientID = "power_monitor_0";
    connectParams.clientIDLen = (uint16_t) strlen("power_monitor_0");
    connectParams.isWillMsgPresent = false;

    ESP_LOGI(TAG, "Connecting to AWS...");
    
    do 
    {
        rc = aws_iot_mqtt_connect(&client, &connectParams);
        if(SUCCESS != rc) {
            gpio_set_level(LED3, 0);
            ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    } while(SUCCESS != rc);

    rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
    if(SUCCESS != rc) 
    {
        gpio_set_level(LED3, 0);
        ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
        abort();
    }

    const char *TOPIC = "test_topic/esp32";
    const int TOPIC_LEN = strlen(TOPIC);

    ESP_LOGI(TAG, "Subscribing...");
    rc = aws_iot_mqtt_subscribe(&client, TOPIC, TOPIC_LEN, QOS0, iot_subscribe_callback_handler, NULL);
    if(SUCCESS != rc) 
    {
        gpio_set_level(LED3, 0);
        ESP_LOGE(TAG, "Error subscribing : %d ", rc);
        abort();
    }

    paramsQOS1.qos = QOS1;
    paramsQOS1.isRetained = 0;

    gpio_set_level(LED3, 1);

	

        if(ready2send == 1)
        {
			// Loop para cada ponto
			for (k = 0; k < NO_OF_CHANNELS; k++)
			{
				// Seleciona o canal
				for(j = 0; j < NO_OF_POINTS; j++)
				{

					switch (k)
					{
						case 0:
						V_PHA1[j] = ratio[k]*(msgBuffer_raw[k][j] - (float)msgBuffer_sum[k]/NO_OF_POINTS)*VREF/4095;
						
						if(j < points2shift)
						{
							j_aux = (NO_OF_POINTS - points2shift) + j;
							V_PHA2[j] = ratio[k]*(msgBuffer_raw[k][j_aux] - (float)msgBuffer_sum[k]/NO_OF_POINTS)*VREF/4095;
						}
						else
						{
							j_aux = j - points2shift;
							V_PHA2[j] = ratio[k]*(msgBuffer_raw[k][j_aux] - (float)msgBuffer_sum[k]/NO_OF_POINTS)*VREF/4095;
						}

						break;

						case 1:
						I_PHA1[j] = ratio[k]*(msgBuffer_raw[k][j] - (float)msgBuffer_sum[k]/NO_OF_POINTS)*VREF/4095;
						break;

						case 2:
						I_PHA2[j] = ratio[k]*(msgBuffer_raw[k][j] - (float)msgBuffer_sum[k]/NO_OF_POINTS)*VREF/4095;
						break;

						case 3:
						I_CIR1[j] = ratio[k]*(msgBuffer_raw[k][j] - (float)msgBuffer_sum[k]/NO_OF_POINTS)*VREF/4095;
						break;

						case 4:
						I_CIR2[j] = ratio[k]*(msgBuffer_raw[k][j] - (float)msgBuffer_sum[k]/NO_OF_POINTS)*VREF/4095;
						break;

						case 5:
						I_CIR3[j] = ratio[k]*(msgBuffer_raw[k][j] - (float)msgBuffer_sum[k]/NO_OF_POINTS)*VREF/4095;
						break;

						default:
						break;
					}
				}
			}

			for (j = 0; j < NO_OF_POINTS; j++)
			{
			    printf("%f ,", msgBuffer_raw[0][j]*VREF/4095);
			}

			//printf("\n Power1 = %f \n Power2 = %f \n I_RMS = %f", power_calculator(V_PHA1, I_PHA1), power_calculator(V_PHA2, I_PHA1), RMS_calculator(I_PHA1));

            printf("\n\n");

			for (j = 0; j < NO_OF_POINTS; j++)
			{
			 	printf("%f ,", msgBuffer_raw[1][j]*VREF/4095);
			}

            ready2send = 0;
			memset(msgBuffer_sum, 0, sizeof(msgBuffer_sum));

			root = cJSON_CreateObject();
        	cJSON_AddNumberToObject(root, "ID", ID);
        	cJSON_AddItemToObject(root, "Phase 1", ph1 = cJSON_CreateObject());
			cJSON_AddNumberToObject(ph1, "CURRENT", RMS_calculator(I_PHA1));
			cJSON_AddNumberToObject(ph1, "POWER", power_calculator(V_PHA1, I_PHA1));
			cJSON_AddItemToObject(root, "Phase 2", ph2 = cJSON_CreateObject());
			cJSON_AddNumberToObject(ph2, "CURRENT", RMS_calculator(I_PHA2));
			cJSON_AddNumberToObject(ph2, "POWER", power_calculator(V_PHA2, I_PHA2));
            cJSON_AddItemToObject(root, "Circuit 1", cir1 = cJSON_CreateObject());
			cJSON_AddNumberToObject(cir1, "CURRENT", RMS_calculator(I_CIR1));
			cJSON_AddNumberToObject(cir1, "POWER", power_calculator(V_PHA1, I_CIR1));
            cJSON_AddItemToObject(root, "Circuit 2", cir2 = cJSON_CreateObject());
			cJSON_AddNumberToObject(cir2, "CURRENT", RMS_calculator(I_CIR2));
			cJSON_AddNumberToObject(cir2, "POWER", power_calculator(V_PHA1, I_CIR2));
            cJSON_AddItemToObject(root, "Circuit 3", cir3 = cJSON_CreateObject());
			cJSON_AddNumberToObject(cir3, "CURRENT", RMS_calculator(I_CIR3));
			cJSON_AddNumberToObject(cir3, "POWER", power_calculator(V_PHA1, I_CIR3));

			rendered = cJSON_Print(root);
            paramsQOS1.payload = (void *) rendered;
            paramsQOS1.payloadLen = strlen(rendered);

            rc = aws_iot_mqtt_publish(&client, TOPIC, TOPIC_LEN, &paramsQOS1);
            if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
                ESP_LOGW(TAG, "QOS1 publish ack not received.");
                rc = SUCCESS;
            }

            cJSON_Delete(root);
			ID++;

			//printf("%s", rendered);
        }

    }

    ESP_LOGE(TAG, "An error occurred in the main loop.");
    abort();
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

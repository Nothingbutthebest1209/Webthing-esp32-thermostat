/* *********************************************************
 * IoT thermostat (ESP32 and DS18B20)
 * Compatible with Web Thing API (https://webthings.io/api/)
 *
 *  Created on:		Dec 20, 2019
 * Last update:		Dec 15, 2020
 *      Author:		Krzysztof Zurek
 *		E-mail:		krzzurek@gmail.com
 *
 * 1-wire from: github.com/DavidAntliff/esp32-ds18b20-example
 ************************************************************/
#include <inttypes.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "simple_web_thing_server.h"
#include "thing_thermostat.h"
#include "thermostat_properties.h"

//1-wire
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

#define ESP_INTR_FLAG_DEFAULT 0

//1-wire DS18B20, digital temperature sensor
#define GPIO_DS18B20_0       	(CONFIG_ONE_WIRE_GPIO)
#define MAX_DEVICES          	(3)
#define DS18B20_RESOLUTION   	(DS18B20_RESOLUTION_12_BIT)
#define DS18B20_SAMPLE_PERIOD	(1000)   // milliseconds
#define TEMP_SAMPLES			(5)

//heater
#define GPIO_HEATER			(CONFIG_HEATER_RELAY_GPIO)
#define GPIO_HEATER_LED		(CONFIG_HEATER_LED_GPIO)
#define GPIO_HEATER_MASK	((1ULL << GPIO_HEATER) | (1ULL << GPIO_HEATER_LED))

static OneWireBus * owb;
static DS18B20_Info *devices[MAX_DEVICES] = {0};
static owb_rmt_driver_info rmt_driver_info;
static int num_devices = 0;

xSemaphoreHandle therm_mux;
xTaskHandle thermostat_task; //task for reading temperature

//THINGS AND PROPERTIES
static double temperature, target_temperature, last_sent_temperature = 0.0;
static int32_t temp_errors, hysteresis, mode;
char *prev_mode_in_action = NULL;
//------------------------------------------------------------
thing_t *thermostat = NULL;
property_t *prop_temperature, *prop_target_temperature, *prop_heating;
property_t *prop_mode, *prop_daily_on_time, *prop_hysteresis;
action_t *timer_action;
static bool init_data_sent = false;
action_input_prop_t *timer_duration;
static int daily_on_time_min = 0, daily_on_time_sec = 0;
static bool heater_is_on = false;
static time_t on_time_last_update = 0;
static TimerHandle_t timer = NULL;

enum_item_t enum_heating_on, enum_heating_off;
enum_item_t enum_mode_on, enum_mode_off, enum_mode_auto;

//set functions
int16_t set_mode(char *new_value_str);
int16_t set_target_temperature(char *new_value_str);
int16_t set_hysteresis(char *new_value_str);
void on_time_set_fun(void);
void update_on_time(bool);

//task function
void thermostat_fun(void *param); //thread function

//other functions
void read_nvs_thermostat_data(void);
void write_nvs_thermostat_data(int i, int data);


/******************************************************
 *
 * end of timer period
 * turn OFF heater and delete timer
 *
 * *****************************************************/
void timer_fun(TimerHandle_t xTimer){
	bool b1 = false, b2 = false;
	
	//printf("warm up timer fun\n");
	complete_action(0, "timer", ACT_COMPLETED);
	
	xSemaphoreTake(therm_mux, portMAX_DELAY);
	if (prev_mode_in_action != prop_mode -> value){
		prop_mode -> value = prev_mode_in_action;
		b1 = true;
	}
	if (prop_mode -> value == mode_tab[0]){
		//switch OFF heater
		prop_heating -> value = heating_tab[0]; //heating = OFF
		gpio_set_level(GPIO_HEATER, 0);
		gpio_set_level(GPIO_HEATER_LED, 0);
		heater_is_on = false;
		b2 = true;
	}
	xSemaphoreGive(therm_mux);
	
	xTimerDelete(xTimer, 100);
	
	if (b1 == true){
		inform_all_subscribers_prop(prop_mode);
	}
	if (b2 == true){
		inform_all_subscribers_prop(prop_heating);
	}
}


/**********************************************************
 *
 * run timer action
 * inputs:
 * 	- minutes of turn ON in json, e.g.: "duration":10
 *
 * *******************************************************/
int8_t timer_run(char *inputs){
	int duration = 0, len;
	char *p1, buff[6], *prev_heating;

	//get duration value
	p1 = strstr(inputs, "duration");
	if (p1 == NULL){
		goto inputs_error;
	}
	p1 = strchr(p1, ':');
	if (p1 == NULL){
		goto inputs_error;
	}
	len = strlen(inputs) - (p1 + 1 - inputs);
	if (len > 5){
		goto inputs_error;
	}
	memset(buff, 0, 6);
	memcpy(buff, p1 + 1, len);
	duration = atoi(buff);
	if ((duration > 600) || (duration == 0)){
		goto inputs_error;
	}

	//printf("duration: %i\n", duration);
	
	xSemaphoreTake(therm_mux, portMAX_DELAY);
	prev_mode_in_action = (char *)prop_mode -> value;
	prop_mode -> value = mode_tab[1]; //heat
	
	prev_heating = (char *)prop_heating -> value;
	prop_heating -> value = heating_tab[1]; //heating
	gpio_set_level(GPIO_HEATER, 1); //turn ON heater
	vTaskDelay(30 / portTICK_PERIOD_MS);
	gpio_set_level(GPIO_HEATER_LED, 1);
	heater_is_on = true;
	
	//start timer
	timer = xTimerCreate("timer_task",
						pdMS_TO_TICKS(duration * 60 * 1000),
						pdFALSE,
						pdFALSE,
						timer_fun);
	xSemaphoreGive(therm_mux);
	
	if (xTimerStart(timer, 5) == pdFAIL){
		printf("warm up timer failed\n");
	}
	
	//if state and/or mode is changed inform subscribers
	if (prev_mode_in_action != mode_tab[1]){
		inform_all_subscribers_prop(prop_mode);
	}
	
	if (prev_heating != heating_tab[1]){
		inform_all_subscribers_prop(prop_heating);
	}

	return 0;

inputs_error:
	printf("timer ERROR\n");
	return -1;
}


/*******************************************************************
*
* set mode, called after http PUT method
* output:
* 	0 - value is ok, but not changed (the same as previous one)
* 	1 - value is changed, subscribers will be informed
*  < 0 - error
*
*******************************************************************/
int16_t set_mode(char *new_value_str){
	int8_t res = -1;
	char *prev_mode, *buff = NULL;
	
	//in websocket quotation mark is not removed
	//(in http should be the same but is not)
	buff = malloc(strlen(new_value_str));
	if (new_value_str[0] == '"'){
		memset(buff, 0, strlen(new_value_str));
		char *ptr = strchr(new_value_str + 1, '"');
		int len = ptr - new_value_str - 1;
		memcpy(buff, new_value_str + 1, len);
	}
	else{
		strcpy(buff, new_value_str);
	}
	
	//set mode
	prev_mode = (char *)prop_mode -> value;
	if (prop_mode -> enum_list != NULL){
		enum_item_t *enum_item = prop_mode -> enum_list;
		while (enum_item != NULL){
			if (strcmp(buff, enum_item -> value.str_addr) == 0){
				prop_mode -> value = enum_item -> value.str_addr;
				res = 1;
				break;
			}
			else{
				enum_item = enum_item -> next;
			}
		}
	}

	//if mode sets "off" or "heat" check if it must change the state
	//of heater (turn it OFF or ON), if yes inform subscribers about
	//change in "heating" property
	if (res == 1){
		update_on_time(false);
		
		//first check if action is running
		if (thermostat -> actions -> running_request_index != -1){
			xTimerDelete(timer, 0);
			complete_action(0, "timer", ACT_DELETED);
		}
		
		if (prev_mode != (char *)prop_mode -> value){
			char *prev_heating;
			int mode_i = 2; //AUTO default
			
			//new mode set
			prev_heating = (char *)prop_heating -> value;
			if (strcmp(prop_mode -> value, mode_tab[1]) == 0){
				//turn heater ON
				xSemaphoreTake(prop_mode -> mux, portMAX_DELAY);
				mode_i = 1;
				prop_heating -> value = heating_tab[1];
				gpio_set_level(GPIO_HEATER, 1);
				vTaskDelay(30 / portTICK_PERIOD_MS);
				gpio_set_level(GPIO_HEATER_LED, 1);
				heater_is_on = true;
				xSemaphoreGive(prop_mode -> mux);
			}
			else if (strcmp(prop_mode -> value, mode_tab[0]) == 0){
				//turn heater OFF
				xSemaphoreTake(prop_mode -> mux, portMAX_DELAY);
				mode_i = 0;
				prop_heating -> value = heating_tab[0];
				gpio_set_level(GPIO_HEATER, 0);
				gpio_set_level(GPIO_HEATER_LED, 0);
				heater_is_on = false;
				xSemaphoreGive(prop_mode -> mux);
			}
			//if state "heating" is changed inform subscribers
			if (prev_heating != prop_heating -> value){
				inform_all_subscribers_prop(prop_heating);
			}
			write_nvs_thermostat_data(2, mode_i);
		}
		else{
			//mode set but not changed
			res = 0;
		}
	}
	free(buff);
	
	return res;
}


/*******************************************************************
*
* set target temperature
*
*******************************************************************/
int16_t set_target_temperature(char *new_value_str){
	int8_t res = 0;
	double prev_tt, tt;
	
	//printf("set target temperature: %s\n", new_value_str);
	
	prev_tt = target_temperature;
	tt = atof(new_value_str);
	if (tt != prev_tt){
		if ((tt >= (double)prop_target_temperature -> min_value.float_val) &&
			(tt <= (double)prop_target_temperature -> max_value.float_val)){
			xSemaphoreTake(prop_target_temperature -> mux, portMAX_DELAY);
			target_temperature = tt;
			xSemaphoreGive(prop_target_temperature -> mux);
			write_nvs_thermostat_data(0, (int32_t)(target_temperature * 10));
			res = 1;
		}
		else{
			res = -1;
		}
	}
	
	return res;
}



/****************************************************************
 *
 * Set hysteresis for temperature controller
 * called after http PUT method
 *
 * **************************************************************/
int16_t set_hysteresis(char *new_value_str){
	int8_t res = 0;
	int new_hyst, vmin, vmax, prev_hyst;
	
	//printf("set hysteresis: %s\n", new_value_str);
	
	prev_hyst = hysteresis;
	new_hyst = atoi(new_value_str);
	vmin = prop_hysteresis -> min_value.int_val;
	vmax = prop_hysteresis -> max_value.int_val;
	if (new_hyst != prev_hyst){
		if ((new_hyst >= vmin) && (new_hyst <= vmax)){
			xSemaphoreTake(prop_hysteresis -> mux, portMAX_DELAY);
			hysteresis = new_hyst;
			xSemaphoreGive(prop_hysteresis -> mux);
			write_nvs_thermostat_data(1, new_hyst);
			res = 1;
		}
		else{
			res = -1;
		}
	}
	
	return res;
}


/****************************************************************
 *
 * Initialize temperature sensor 1-wire DS18B20 (12 bit)
 *
 * **************************************************************/
int init_ds18b20(void){
	// Create a 1-Wire bus, using the RMT timeslot driver
	int num_devices = 0;

	//vTaskDelay(200 / portTICK_PERIOD_MS);

	owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);
	owb_use_crc(owb, true);  // enable CRC check for ROM code

	// Find all connected devices
	printf("Find devices:\n");
	OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = {0};
	OneWireBus_SearchState search_state = {0};
	bool found = false;
	owb_search_first(owb, &search_state, &found);
	while (found){
		char rom_code_s[17];
		owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
		printf("  %d : %s\n", num_devices, rom_code_s);
		device_rom_codes[num_devices] = search_state.rom_code;
		++num_devices;
		owb_search_next(owb, &search_state, &found);
	}
	printf("Found %d device%s\n", num_devices, num_devices == 1 ? "" : "s");

	if (num_devices > 0){
		if (num_devices == 1){
			// For a single device only:
			OneWireBus_ROMCode rom_code;
			owb_status status = owb_read_rom(owb, &rom_code);
			if (status == OWB_STATUS_OK){
				char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
				owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
				printf("Single device %s present\n", rom_code_s);
			}
			else{
				printf("An error occurred reading ROM code: %d", status);
			}
		}
		else{
			// Search for a known ROM code (LSB first):
			// For example: 0x1502162ca5b2ee28
			OneWireBus_ROMCode known_device = {
					.fields.family = { 0x28 },
					.fields.serial_number = { 0xee, 0xb2, 0xa5, 0x2c, 0x16, 0x02 },
					.fields.crc = { 0x15 },
			};
			char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
			owb_string_from_rom_code(known_device, rom_code_s, sizeof(rom_code_s));
			bool is_present = false;

			owb_status search_status = owb_verify_rom(owb, known_device, &is_present);
			if (search_status == OWB_STATUS_OK){
				printf("Device %s is %s\n", rom_code_s, is_present ? "present" : "not present");
			}
			else{
				printf("An error occurred searching for known device: %d", search_status);
			}
		}

		// Create DS18B20 devices on the 1-Wire bus
		for (int i = 0; i < num_devices; ++i){
			DS18B20_Info *ds18b20_info = ds18b20_malloc();  // heap allocation
			devices[i] = ds18b20_info;
	
			if (num_devices == 1){
				printf("Single device optimizations enabled\n");
				ds18b20_init_solo(ds18b20_info, owb); // only one device on bus
			}
			else{
				ds18b20_init(ds18b20_info, owb, device_rom_codes[i]); // associate with bus and device
			}
			ds18b20_use_crc(ds18b20_info, true); // enable CRC check for temperature readings
			ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);
		}
	}
	return num_devices;
}


/*********************************************************************
 *
 * main thermostat task, it's main functions are:
 *	- reads current temperature (Tc)
 *	- compares Tc with target temperature (Tt) and if Tt - Tc >= Th then
 *	  turn ON heating
 *	- if Tc - Tt >= Th then switch OFF heating
 * where:
 * Th - hysteresis temperature
 *
 * ******************************************************************/
void thermostat_fun(void *param){
	//int errors_count[MAX_DEVICES] = {0};
	int sample_nr = 0;//, on_time_cnt = 0;
	float readings[MAX_DEVICES] = { 0 };
	DS18B20_ERROR errors[MAX_DEVICES] = { 0 };
	double temp_sum = 0;
	int correct_samples = 0;
	
	TickType_t last_wake_time = xTaskGetTickCount();
	for (;;){
		if (num_devices > 0){
			//DS18B20 sensor is found
			last_wake_time = xTaskGetTickCount();

			ds18b20_convert_all(owb);
			ds18b20_wait_for_conversion(devices[0]);

			for (int i = 0; i < num_devices; ++i){
				errors[i] = ds18b20_read_temp(devices[i], &readings[i]);
			}

			//read first temperature (only one sensor is available in this version)
			if (errors[0] == DS18B20_OK){
				correct_samples++;
				temp_sum += (double)readings[0];
			}
			else{
				temp_errors++;
			}

			sample_nr++;
			if (sample_nr == TEMP_SAMPLES){
				int temp_correctness = 0.0;
				
				update_on_time(false);
				//set new correctness
				temp_correctness = (100 * correct_samples)/TEMP_SAMPLES;
				
				if (temp_correctness > 10){
					//set new temperature
					xSemaphoreTake(prop_mode -> mux, portMAX_DELAY);
					temperature = temp_sum / correct_samples;
					xSemaphoreGive(prop_mode -> mux);
					
					if (abs((int)((temperature - last_sent_temperature)*100)) >= 10){
						inform_all_subscribers_prop(prop_temperature);
						last_sent_temperature = temperature;
					}
				}
				
				//heater control
				xSemaphoreTake(prop_mode -> mux, portMAX_DELAY);
				char *prev_heating = prop_heating -> value;
				char *new_heating = prop_heating -> value;
				
				if (strcmp(prop_mode -> value, mode_tab[2]) == 0){
					//mode = AUTO
					if (temp_correctness > 10){
						double hyst = ((double)hysteresis)/10;
						//at least one reading must be OK
						if (temperature <= target_temperature - hyst){
							//turn heater ON
							prop_heating -> value = heating_tab[1];
							new_heating = heating_tab[1];
							gpio_set_level(GPIO_HEATER, 1);
							vTaskDelay(30 / portTICK_PERIOD_MS);
							gpio_set_level(GPIO_HEATER_LED, 1);
							if (heater_is_on == false){
								heater_is_on = true;
							} 
						}
						else if (temperature >= target_temperature + hyst){
							//turn heater OFF
							prop_heating -> value = heating_tab[0];
							new_heating = heating_tab[0];
							gpio_set_level(GPIO_HEATER, 0);
							gpio_set_level(GPIO_HEATER_LED, 0);
							if (heater_is_on == true){
								heater_is_on = false;
							} 						
						}
					}
					else{
						//temperature sensor failed, switch off heater
						prop_heating -> value = heating_tab[0];
						new_heating = heating_tab[0];
						gpio_set_level(GPIO_HEATER, 0);
						gpio_set_level(GPIO_HEATER_LED, 0);
						heater_is_on = false;
					}
				}
				xSemaphoreGive(prop_mode -> mux);
				
				//if state "heating" is changed inform subscribers
				if (prev_heating != new_heating){
					inform_all_subscribers_prop(prop_heating);
				}

				//reset other values
				sample_nr = 0;
				temp_sum = 0;
				correct_samples = 0;
			} //if (sample_nr == TEMP_SAMPLES)
		}
		
		if (init_data_sent == false){
			int8_t s1 = inform_all_subscribers_prop(prop_heating);
			int8_t s2 = inform_all_subscribers_prop(prop_mode);
			int8_t s3 = inform_all_subscribers_prop(prop_daily_on_time);
			if ((s1 == 0) && (s2 == 0) && (s3 == 0)){
				init_data_sent = true;
			}
		}
		vTaskDelayUntil(&last_wake_time, DS18B20_SAMPLE_PERIOD / portTICK_PERIOD_MS);
	}//for
}


/***************************************************************
*
* daily ON time update and inform subscribers if necessary
*
****************************************************************/
void update_on_time(bool reset){
	struct tm timeinfo;
	int delta_t = 0, prev_minutes = 0, new_minutes = 0;
	time_t current_time, prev_time;
	bool send_data = false;

	prev_time = on_time_last_update;
	time(&current_time);
	localtime_r(&current_time, &timeinfo);
	if (timeinfo.tm_year > (2018 - 1900)) {
		//time is correct
		xSemaphoreTake(therm_mux, portMAX_DELAY);
		if (heater_is_on == true){
			prev_minutes = daily_on_time_min;
			new_minutes = prev_minutes;
			delta_t = current_time - prev_time;
			if (delta_t > 0){
				daily_on_time_sec += delta_t;
				daily_on_time_min = daily_on_time_sec / 60;
				new_minutes = daily_on_time_min;
			}	
		}
		on_time_last_update = current_time;
		xSemaphoreGive(therm_mux);
		
		if (new_minutes != prev_minutes){
			send_data = true;
		}
		
		if (reset == true){
			xSemaphoreTake(therm_mux, portMAX_DELAY);
			daily_on_time_sec = 0;
			daily_on_time_min = 0;
			xSemaphoreGive(therm_mux);
			send_data = true;
		}
		
		if (send_data == true){
			inform_all_subscribers_prop(prop_daily_on_time);
		}
    }
}


/*************************************************************
*
* at the beginning of the day reset minuts and seconds counters
* and inform subscribers if necessary
*
**************************************************************/
void daily_on_time_reset(void){
	update_on_time(true);
}


/*******************************************************************
 *
 * initialize GPIOs for heater and led
 *
 * ******************************************************************/
void init_gpio(void){
	gpio_config_t io_conf;

	//disable interrupt
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	//bit mask of the pins
	io_conf.pin_bit_mask = GPIO_HEATER_MASK;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	io_conf.pull_down_en = 0;
	gpio_config(&io_conf);
	
	gpio_set_level(GPIO_HEATER, 0);
	gpio_set_level(GPIO_HEATER_LED, 0); //led is off
}


/*****************************************************************
 *
 * Initialization termostat thing and all it's properties
 *
 * ****************************************************************/
thing_t *init_thermostat(void){

	temp_errors = 0;
	temperature = 0.0;
	read_nvs_thermostat_data();
	
	init_gpio();
	
	//start thing
	therm_mux = xSemaphoreCreateMutex();
	//create thing 1, thermostat ---------------------------------
	thermostat = thing_init();

	thermostat -> id = therm_id_str;
	thermostat -> at_context = things_context;
	thermostat -> model_len = 2300;
	//set @type
	therm_type.at_type = therm_attype_str;
	therm_type.next = NULL;
	set_thing_type(thermostat, &therm_type);
	thermostat -> description = therm_disc;

	//"temperature" property -------------------------------------------
	prop_temperature = property_init(NULL, NULL);
	prop_temperature -> id = temp_prop_id;
	prop_temperature -> description = temp_prop_disc;
	temp_prop_type.at_type = temp_prop_attype_str;
	temp_prop_type.next = NULL;
	prop_temperature -> at_type = &temp_prop_type;
	prop_temperature -> type = VAL_NUMBER;
	prop_temperature -> value = &temperature;
	prop_temperature -> max_value.float_val = 125.0;
	prop_temperature -> min_value.float_val = -55.0;
	prop_temperature -> unit = temp_prop_unit;
	prop_temperature -> title = temp_prop_title;
	prop_temperature -> read_only = true;
	prop_temperature -> enum_prop = false;
	prop_temperature -> set = NULL;
	prop_temperature -> mux = therm_mux;

	add_property(thermostat, prop_temperature); //add property to thing
	
	//"target temperature" property -------------------------------------
	prop_target_temperature = property_init(NULL, NULL);
	prop_target_temperature -> id = target_prop_id;
	prop_target_temperature -> description = target_prop_disc;
	target_prop_type.at_type = target_prop_attype_str;
	target_prop_type.next = NULL;
	prop_target_temperature -> at_type = &target_prop_type;
	prop_target_temperature -> type = VAL_NUMBER;
	prop_target_temperature -> value = &target_temperature;
	prop_target_temperature -> max_value.float_val = 30.0;
	prop_target_temperature -> min_value.float_val = 2.0;
	prop_target_temperature -> unit = target_prop_unit;
	prop_target_temperature -> title = target_prop_title;
	prop_target_temperature -> read_only = false;
	prop_target_temperature -> enum_prop = false;
	prop_target_temperature -> set = &set_target_temperature;
	prop_target_temperature -> mux = therm_mux;

	add_property(thermostat, prop_target_temperature); //add property to thing
	
	//"heating" property -------------------------------------
	prop_heating = property_init(NULL, NULL);
	prop_heating -> id = heating_prop_id;
	prop_heating -> description = heating_prop_disc;
	heating_prop_type.at_type = heating_prop_attype_str;
	heating_prop_type.next = NULL;
	prop_heating -> at_type = &heating_prop_type;
	prop_heating -> type = VAL_STRING;
	if (mode == 1){
		//start with 'heating'
		prop_heating -> value = heating_tab[1];
		gpio_set_level(GPIO_HEATER, 1);
		vTaskDelay(30 / portTICK_PERIOD_MS);
		gpio_set_level(GPIO_HEATER_LED, 1); 
	}
	else{
		//start with 'off'
		prop_heating -> value = heating_tab[0];
	}
	prop_heating -> title = heating_prop_title;
	prop_heating -> read_only = true;
	prop_heating -> enum_prop = true;
	prop_heating -> enum_list = &enum_heating_on;
	enum_heating_on.value.str_addr = heating_tab[1];
	enum_heating_on.next = &enum_heating_off;
	enum_heating_off.value.str_addr = heating_tab[0];
	enum_heating_off.next = NULL;
	prop_heating -> set = NULL;
	prop_heating -> mux = therm_mux;

	add_property(thermostat, prop_heating); //add property to thing
	
	//"mode" property ------------------------------------
	prop_mode = property_init(NULL, NULL);
	prop_mode -> id = mode_prop_id;
	prop_mode -> description = mode_prop_disc;
	mode_prop_type.at_type = mode_prop_attype_str;
	mode_prop_type.next = NULL;
	prop_mode -> at_type = &mode_prop_type;
	prop_mode -> type = VAL_STRING;
	prop_mode -> value = mode_tab[mode];
	prop_mode -> title = mode_prop_title;
	prop_mode -> read_only = false;
	prop_mode -> enum_prop = true;
	prop_mode -> enum_list = &enum_mode_on;
	enum_mode_on.value.str_addr = mode_tab[1];
	enum_mode_on.next = &enum_mode_off;
	enum_mode_off.value.str_addr = mode_tab[0];
	enum_mode_off.next = &enum_mode_auto;
	enum_mode_auto.value.str_addr = mode_tab[2];
	enum_mode_auto.next = NULL;
	prop_mode -> set = &set_mode;
	prop_mode -> mux = therm_mux;

	add_property(thermostat, prop_mode); //add property to thing	
	
	//"daily on time" property -------------------------------------------
	prop_daily_on_time = property_init(NULL, NULL);
	prop_daily_on_time -> id = daily_on_prop_id;
	prop_daily_on_time -> description = daily_on_prop_disc;
	daily_on_prop_type.at_type = daily_on_prop_attype_str;
	daily_on_prop_type.next = NULL;
	prop_daily_on_time -> at_type = &daily_on_prop_type;
	prop_daily_on_time -> type = VAL_INTEGER;
	prop_daily_on_time -> value = &daily_on_time_min;
	prop_daily_on_time -> unit = daily_on_prop_unit;
	prop_daily_on_time -> max_value.int_val = 1440;
	prop_daily_on_time -> min_value.int_val = 0;
	prop_daily_on_time -> title = daily_on_prop_title;
	prop_daily_on_time -> read_only = true;
	prop_daily_on_time -> enum_prop = false;
	prop_daily_on_time -> set = NULL;
	prop_daily_on_time -> mux = therm_mux;
	
	add_property(thermostat, prop_daily_on_time); //add property to thing
	
	//"hysteresis" property -------------------------------------
	prop_hysteresis = property_init(NULL, NULL);
	prop_hysteresis -> id = hyst_prop_id;
	prop_hysteresis -> description = hyst_prop_disc;
	hyst_prop_type.at_type = hyst_prop_attype_str;
	hyst_prop_type.next = NULL;
	prop_hysteresis -> at_type = &hyst_prop_type;
	prop_hysteresis -> type = VAL_INTEGER;
	prop_hysteresis -> value = &hysteresis;
	prop_hysteresis -> max_value.int_val = 100;
	prop_hysteresis -> min_value.int_val = 1;
	prop_hysteresis -> unit = hyst_prop_unit;
	prop_hysteresis -> title = hyst_prop_title;
	prop_hysteresis -> read_only = false;
	prop_hysteresis -> enum_prop = false;
	prop_hysteresis -> set = &set_hysteresis;
	prop_hysteresis -> mux = therm_mux;

	add_property(thermostat, prop_hysteresis); //add property to thing
	
	//action "timer_action", turn on heater for specified minutes
	timer_action = action_init();
	timer_action -> id = timer_id;
	timer_action -> title = timer_title;
	timer_action -> description = timer_desc;
	timer_action -> run = timer_run;
	timer_input_attype.at_type = timer_input_attype_str;
	timer_input_attype.next = NULL;
	timer_action -> input_at_type = &timer_input_attype;
	timer_duration = action_input_prop_init(timer_prop_dur_id,
			VAL_INTEGER, true, &timer_duration_min, &timer_duration_max,
			timer_duration_unit);
	add_action_input_prop(timer_action, timer_duration);
	add_action(thermostat, timer_action);

	//--------------------------------------------------------------------------
	num_devices = init_ds18b20();
	if (num_devices == 0){
		// clean up dynamically allocated data
		printf("No DS18B20 devices found\n");
		owb_uninitialize(owb);
	}
	
	//start control thread	
	xTaskCreate(&thermostat_fun, "thermostat", configMINIMAL_STACK_SIZE * 4, NULL, 5, &thermostat_task);

	return thermostat;
}


/****************************************************************
 *
 * read thermostat data written in NVS memory:
 *  - target_temperature
 *  - hysteresis
 *  - mode
 *
 * **************************************************************/
void read_nvs_thermostat_data(void){
	esp_err_t err;
	nvs_handle storage_handle = 0;
	int32_t nvs_target, nvs_hyst, nvs_mode;

	//default values
	target_temperature = 10.0;	//10 celsius degrees
	hysteresis = 10;			//0.1 celsius degrees
	mode = 2;					//AUTO

	// Open
	printf("Reading thermostat data...\n");

	err = nvs_open("storage", NVS_READONLY, &storage_handle);
	if (err != ESP_OK) {
		printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	}
	else {
		// Read data
		if (nvs_get_i32(storage_handle, "therm_target", &nvs_target) == ESP_OK){
			target_temperature = ((double)nvs_target) / 10;
		}
		if (nvs_get_i32(storage_handle, "therm_hyst", &nvs_hyst) == ESP_OK){
			hysteresis = nvs_hyst;
		}
		if (nvs_get_i32(storage_handle, "therm_mode", &nvs_mode) == ESP_OK){
			mode = nvs_mode;
		}

		// Close
		nvs_close(storage_handle);
	}
	//printf("Thermostat\ntarget: %i, hyst: %i, mode: %i\n", nvs_target, nvs_hyst, nvs_mode);
}


/****************************************************************
 *
 * write thermostat data into NVS memory
 * input:
 *  i: 0 - target, 1 - hysteresis, 2 - mode
 *
 * **************************************************************/
void write_nvs_thermostat_data(int i, int data){
	esp_err_t err;
	nvs_handle storage_handle = 0;

	//open NVS falsh memory
	err = nvs_open("storage", NVS_READWRITE, &storage_handle);
	if (err != ESP_OK) {
		printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	}
	else {
		if (i == 0){
			nvs_set_i32(storage_handle, "therm_target", data);
		}
		else if (i == 1){
			nvs_set_i32(storage_handle, "therm_hyst", data);
		}
		else if (i == 2){
			nvs_set_i32(storage_handle, "therm_mode", data);
		}
		else{
			printf("Thermostat, NVS write: index error\n");
		}
		
		err = nvs_commit(storage_handle);
		// Close
		nvs_close(storage_handle);
	}
}

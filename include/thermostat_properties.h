/*
 * webthing_thermostat.h
 *
 *  Created on: Oct 29, 2019
 *      Author: Krzysztof Zurek
 *		krzzurek@gmail.com
 */

#ifndef THERMOSTAT_PROP_H_
#define THERMOSTAT_PROP_H_

#include "simple_web_thing_server.h"

//-------------------------------------------------------------
//					--- THERMOSTAT ---
//------------------------------------------------------------
//thermostat thing description
char therm_id_str[] = "Thermostat";
char therm_attype_str[] = "Thermostat";
char therm_disc[] = "Indoor thermostat";
at_type_t therm_type;

//------  property "temperature"
char temp_prop_id[] = "temperature";
char temp_prop_disc[] = "temperature sensor DS18B20";
char temp_prop_attype_str[] = "TemperatureProperty";
char temp_prop_unit[] = "degree celsius";
char temp_prop_title[] = "Temperature";
at_type_t temp_prop_type;

//------  property "target temperature"
char target_prop_id[] = "target_temp";
char target_prop_disc[] = "target temperature";
char target_prop_attype_str[] = "TargetTemperatureProperty";
char target_prop_unit[] = "degree celsius";
char target_prop_title[] = "Target";
at_type_t target_prop_type;

//------  property "heating"
char heating_prop_id[] = "heating";
char heating_prop_disc[] = "Shows if heating is ON/OFF";
char heating_prop_attype_str[] = "HeatingCoolingProperty";
char heating_prop_title[] = "Heating";
at_type_t heating_prop_type;
char heating_tab[2][10] = {"off", "heating"};

//------  property "mode"
char mode_prop_id[] = "mode";
char mode_prop_disc[] = "Thermostat mode";
char mode_prop_attype_str[] = "ThermostatModeProperty";
char mode_prop_title[] = "Mode";
at_type_t mode_prop_type;
char mode_tab[3][5] = {"off", "heat", "auto"}; 

//------  property "daily on time"
char daily_on_prop_id[] = "daily_on";
char daily_on_prop_disc[] = "amount of time heater is ON";
char daily_on_prop_attype_str[] = "LevelProperty";
char daily_on_prop_unit[] = "min";
char daily_on_prop_title[] = "ON minutes";
at_type_t daily_on_prop_type;

//------  property "hysteresis"
char hyst_prop_id[] = "hysteresis";
char hyst_prop_disc[] = "Hysteresis by temperature control";
char hyst_prop_attype_str[] = "LevelProperty";
char hyst_prop_unit[] = "degree celsius";
char hyst_prop_title[] = "dT x 10";
at_type_t hyst_prop_type;

//------- action timer
char timer_id[] = "timer";
char timer_title[] = "Timer";
char timer_desc[] = "Turn ON heater for specified period of time";
char timer_input_attype_str[] = "ToggleAction";
char timer_prop_dur_id[] = "duration";
at_type_t timer_input_attype;
double timer_duration_min = 1; //minutes
double timer_duration_max = 600;
char timer_duration_unit[] = "min";

#endif /* THERMOSTAT_PROP_H_ */

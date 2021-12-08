/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup pin_change_int_example_main main.c
 * @{
 * @ingroup pin_change_int_example
 * @brief Pin Change Interrupt Example Application main file.
 *
 * This file contains the source code for a sample application using interrupts triggered by GPIO pins.
 *
 */

#include <stdbool.h>
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_drv_ppi.h"

#define TIMER_COUNTER_ID		0
#define TIMER_CAPTURE_ID 		1
#define CAPTURE_TIME				10
#define TEMP_SENS_PIN 			0
#define MOIST_SENS_IN_PIN		1
#define	MOIST_SENS_PWR_PIN	4
#define LED_RED_PIN					17
#define LED_BLU_PIN					19
#define LED_GRN_PIN					18
#define BUTTON_0_PIN 				28
#define BUTTON_1_PIN 				30


typedef struct {
	struct {
		const nrf_drv_timer_t *ptmr_capture;
		const nrf_drv_timer_t *ptmr_counter;
		uint8_t in_pin;
		uint8_t pwr_pin;
		volatile uint32_t curr_ctr;
		uint32_t prev_cnt;
		uint8_t tmr_en;
		} msens;
	
}dmon_drvr_t;


dmon_drvr_t dmon_drvr;

static inline void init_dmon_drvr(dmon_drvr_t *dmon_drvr){
	dmon_drvr->msens.in_pin=MOIST_SENS_IN_PIN;
	dmon_drvr->msens.pwr_pin=MOIST_SENS_PWR_PIN;
	dmon_drvr->msens.prev_cnt=0;
	dmon_drvr->msens.curr_ctr=0;
	const nrf_drv_timer_t tmr_capture = NRF_DRV_TIMER_INSTANCE(TIMER_CAPTURE_ID);
	const nrf_drv_timer_t tmr_counter = NRF_DRV_TIMER_INSTANCE(TIMER_COUNTER_ID);
	dmon_drvr->msens.ptmr_capture = &tmr_capture;
	dmon_drvr->msens.ptmr_counter = &tmr_counter;
}

static inline void init_leds() {
	/* Configure LEDs */
	nrf_gpio_range_cfg_output(LED_RED_PIN, LED_BLU_PIN);
	nrf_gpio_pin_set(LED_RED_PIN);											// LEDs are LOW-on
	nrf_gpio_pin_set(LED_BLU_PIN);
	nrf_gpio_pin_set(LED_GRN_PIN);
	
	//nrf_gpio_cfg_output(TEMP_SENS_PIN);
}


/* Will not be used when using PPI */
void moist_sens_input_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	//dmon_drvr.msens.curr_ctr++; // Increase counter
	//nrf_gpio_pin_toggle(TEMP_SENS_PIN);
}
/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output, 
 * and configures GPIOTE to give an interrupt on pin change.
 */
void moist_sens_gpio_cfg(dmon_drvr_t *dmon_drvr)
{
  ret_code_t err_code;

	// Configure input interrrupts
  
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_config_t moist_sens_config;
  moist_sens_config.pull = NRF_GPIO_PIN_PULLDOWN;
	moist_sens_config.sense= NRF_GPIOTE_POLARITY_LOTOHI;
	moist_sens_config.hi_accuracy = true;
	moist_sens_config.is_watcher = false;
	err_code = nrf_drv_gpiote_in_init(dmon_drvr->msens.in_pin, &moist_sens_config, NULL);
	APP_ERROR_CHECK(err_code);
	
	// Configure power
	nrf_gpio_cfg_output(MOIST_SENS_PWR_PIN);
	nrf_gpio_pin_set(MOIST_SENS_PWR_PIN);
}

void ppi_pulse_in_pulse_out(dmon_drvr_t *pdrvr)
{
	uint32_t pulse_in_event_addr;
	uint32_t pulse_out_task_addr;
	nrf_ppi_channel_t ppi_pin_pout;
	
	ret_code_t err_code;
	nrf_drv_gpiote_out_config_t config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);
	
	err_code = nrf_drv_gpiote_out_init(TEMP_SENS_PIN, &config);
	APP_ERROR_CHECK(err_code);
	
	 err_code = nrf_drv_ppi_channel_alloc(&ppi_pin_pout);
	APP_ERROR_CHECK(err_code);
	
	pulse_in_event_addr = nrf_drv_gpiote_in_event_addr_get(pdrvr->msens.in_pin);
	pulse_out_task_addr = nrf_drv_gpiote_out_task_addr_get(TEMP_SENS_PIN);
	
	err_code = nrf_drv_ppi_channel_assign(ppi_pin_pout, pulse_in_event_addr, pulse_out_task_addr);
	APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_ppi_channel_enable(ppi_pin_pout);
	APP_ERROR_CHECK(err_code);
	
	//nrf_drv_gpiote_out_task_enable(TEMP_SENS_PIN);
	//nrf_drv_gpiote_in_event_enable(pdrvr->msens.in_pin, true);
}

static inline void pulse_out_gpiote_start(uint8_t pin) {
	nrf_drv_gpiote_out_task_enable(pin);
}

	static inline void moist_sens_gpiote_start(uint8_t pin){
	nrf_drv_gpiote_in_event_enable(pin, true);
}

static inline void moist_sens_gpiote_stop(uint8_t pin){
	nrf_drv_gpiote_in_event_disable(pin);
}


/**
 * @brief Handler for timer events.
 */
void moist_sens_tmr_evnt_handler(nrf_timer_event_t event_type, void* p_context)
{
	switch(event_type)
	{
		case NRF_TIMER_EVENT_COMPARE0:
			/* Stop gpio interrupt--> Copy value --> Restart gpio interrupt */
			//moist_sens_gpiote_stop(dmon_drvr.msens.in_pin);
			//dmon_drvr.msens.prev_cnt=dmon_drvr.msens.curr_ctr;
			//dmon_drvr.msens.curr_ctr=0;
			//moist_sens_gpiote_start(dmon_drvr.msens.in_pin);
		
		//NRF_TIMER1->TASKS_CAPTURE[0] = 1;
		dmon_drvr.msens.curr_ctr = nrf_drv_timer_capture(dmon_drvr.msens.ptmr_counter,
																											NRF_TIMER_CC_CHANNEL0);
		
		// Timer is stopped. Clear and Restart the counter timer 
		nrf_drv_timer_clear(dmon_drvr.msens.ptmr_counter);
		nrf_drv_timer_enable(dmon_drvr.msens.ptmr_counter);
		//nrf_gpio_pin_toggle(TEMP_SENS_PIN);
		break;
		default:
			//Do nothing.
		break;
	}    
}
void moist_sens_tmr_capture_cfg(const nrf_drv_timer_t *ptmr) {
	uint32_t time_ms = CAPTURE_TIME; //Time(in miliseconds) between consecutive compare events.
	uint32_t time_ticks;
	uint32_t err_code = NRF_SUCCESS;
	
	err_code = nrf_drv_timer_init(ptmr, NULL, moist_sens_tmr_evnt_handler);
	APP_ERROR_CHECK(err_code);
	
	time_ticks = nrf_drv_timer_ms_to_ticks(ptmr, time_ms);
	
	nrf_drv_timer_extended_compare(
			ptmr, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
}

void tmr_ctr_evnt_handler(nrf_timer_event_t event_type, void* p_context){
}
	
void moist_sens_tmr_counter_cfg(const nrf_drv_timer_t *ptmr) {
  uint32_t err_code = NRF_SUCCESS;
  nrf_drv_timer_config_t cfg = NRF_DRV_TIMER_DEFAULT_CONFIG(TIMER_COUNTER_ID);
	cfg.mode = NRF_TIMER_MODE_COUNTER;
  
  // Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
  err_code = nrf_drv_timer_init(ptmr, &cfg, tmr_ctr_evnt_handler);
  APP_ERROR_CHECK(err_code);
	// false - no interrupts needed. PPI to be used 
  nrf_drv_timer_extended_compare(
       ptmr, NRF_TIMER_CC_CHANNEL0, 100000, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
  
}
static inline void tmr_enable(const nrf_drv_timer_t *ptmr){
	nrf_drv_timer_enable(ptmr);
	dmon_drvr.msens.tmr_en=1;
}

void ppi_moist_sens_sys_cfg(dmon_drvr_t *pdrvr){
	uint32_t capture_evt_addr;
	uint32_t count_stop_task_addr;
	uint32_t pulse_in_event_addr;
	uint32_t count_task_addr;
	nrf_ppi_channel_t ppi_count, ppi_capture;
	ret_code_t err_code;
	
	err_code = nrf_drv_ppi_channel_alloc(&ppi_count);
	APP_ERROR_CHECK(err_code);
	
	count_task_addr			= nrf_drv_timer_task_address_get(pdrvr->msens.ptmr_counter, NRF_TIMER_TASK_COUNT);
	pulse_in_event_addr	= nrf_drv_gpiote_in_event_addr_get(pdrvr->msens.in_pin);
	err_code = nrf_drv_ppi_channel_assign(ppi_count, pulse_in_event_addr, count_task_addr);
	APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_ppi_channel_alloc(&ppi_capture);
	APP_ERROR_CHECK(err_code);
	count_stop_task_addr	= nrf_drv_timer_task_address_get(pdrvr->msens.ptmr_counter, NRF_TIMER_TASK_STOP);
	capture_evt_addr			= nrf_drv_timer_event_address_get(pdrvr->msens.ptmr_capture, NRF_TIMER_EVENT_COMPARE0);
	err_code = nrf_drv_ppi_channel_assign(ppi_capture, capture_evt_addr, count_stop_task_addr);
	APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_ppi_channel_enable(ppi_capture);
	APP_ERROR_CHECK(err_code);
	 
	err_code = nrf_drv_ppi_channel_enable(ppi_count);
	APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
	init_leds();
	ret_code_t err_code;
	
	err_code = nrf_drv_ppi_init();
	APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_gpiote_init();
	APP_ERROR_CHECK(err_code);
	
	init_dmon_drvr(&dmon_drvr);
	moist_sens_gpio_cfg(&dmon_drvr);
	
	moist_sens_tmr_capture_cfg(dmon_drvr.msens.ptmr_capture);
	moist_sens_tmr_counter_cfg(dmon_drvr.msens.ptmr_counter);
	
	ppi_moist_sens_sys_cfg(&dmon_drvr);
	ppi_pulse_in_pulse_out(&dmon_drvr);
	
	tmr_enable(dmon_drvr.msens.ptmr_counter);
	tmr_enable(dmon_drvr.msens.ptmr_capture);
	
	pulse_out_gpiote_start(TEMP_SENS_PIN);
	moist_sens_gpiote_start(dmon_drvr.msens.in_pin);
	
	
  while (true)
  {
		nrf_gpio_pin_clear(LED_RED_PIN);
		nrf_delay_ms(50);
		nrf_gpio_pin_set(LED_RED_PIN);
		nrf_delay_ms(1950);
  }
}


/** @} */

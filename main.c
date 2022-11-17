/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RutDevKit-PSoC62_GEN4_ULCD_43
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2022-10-28
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "visiGenieSerial.h"
#include "sgp40.h"
#include "sensirion_voc_algorithm.h"

#define UART_RX_PRIO				2
#define ARDU_BAUD_RATE       		115200

void handle_error(void);
static cy_rslt_t ardu_uart_init();
cy_rslt_t app_hw_init(void);
static void adc_continuous_event_handler(void* arg, cyhal_adc_event_t event);

/* UserApiConfig */
static bool uartAvailHandler(void);
static uint8_t uartReadHandler(void);
static void uartWriteHandler(uint32_t val);
static uint32_t uartGetMillis(void);
static void resetDisplay(void);

/* Event handlers */
static void myGenieEventHandler(void);

/*Arduino UART object*/
cyhal_uart_t ardu_uart;

/* Timer object used */
cyhal_timer_t timer_obj;

/* ADC Object */
cyhal_adc_t adc_obj;

/* ADC Channel Objects */
cyhal_adc_channel_t adc_potentiometer;

/* Default ADC configuration */
const cyhal_adc_config_t adc_config =
{
        .continuous_scanning=true,  	// Continuous Scanning is enabled
        .average_count=1,           	// Average count disabled
        .vref=CYHAL_ADC_REF_VDDA,   	// VREF for Single ended channel set to VDDA
        .vneg=CYHAL_ADC_VNEG_VSSA,  	// VNEG for Single ended channel set to VSSA
        .resolution = 12u,          	// 12-bit resolution
        .ext_vref = NC,
        .bypass_pin = NC,
};

/* ADC channel configuration */
const cyhal_adc_channel_config_t channel_config =
{
		.enable_averaging = false,  // Disable averaging for channel
        .min_acquisition_ns = 10000, // Minimum acquisition time set to 10us
        .enabled = true
};

/* ADC data storage */
int32_t adc_data = 0;

/*Arduino UART reception variable*/
uint8_t RxByte;

/*I2C Device Global Variables*/
cyhal_i2c_t I2C_scb3;
cyhal_i2c_cfg_t i2c_scb3_cfg =
{
		.is_slave = false,
	    .address = 0,
	    .frequencyhal_hz = 400000UL,
};

/*VOC Index Algorithm Parameters*/
VocAlgorithmParams voc_algorithm_params;

/*LED control variables*/
_Bool led1 = false;
_Bool led2 = false;

/*Use potentiometer instead of SGP40*/
_Bool sgp_present = true;

const cyhal_timer_cfg_t timer_cfg =
{
    .compare_value = 0,                 /* Timer compare value, not used */
    .period = 0xFFFFFFFF-1,             /* Timer period*/
    .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
    .is_compare = false,                /* Don't use compare mode */
    .is_continuous = true,             	/* Run timer indefinitely */
    .value = 0                          /* Initial value of counter */
};

static UserApiConfig userConfig =
{
	.available = uartAvailHandler,
	.read =  uartReadHandler,
	.write = uartWriteHandler,
	.millis = uartGetMillis
};

int main(void)
{
    cy_rslt_t result;
    uint16_t gaugeVal = 0;
    uint32_t waitPeriodDisplay = 0;
    uint32_t waitPeriodSensor = 0;
    uint32_t waitPeriodChart = 0;
    static _Bool sensor_read = false;
    uint16_t sgp_sraw_voc = 0;
    int32_t sgp_voc_index = 0;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
    	handle_error();
    }

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    printf("\x1b[2J\x1b[;H");
    printf("4D Systems Gen4 uLCD Display Demo.\n\r");

    __enable_irq();

    /*Initialize LEDs*/
    result = cyhal_gpio_init( LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Initialize Display RESET pin*/
    result = cyhal_gpio_init(ARDU_IO8, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Initialize The Arduino UART*/
    result = ardu_uart_init();
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Initialize I2C Master*/
    result = cyhal_i2c_init(&I2C_scb3, ARDU_SDA, ARDU_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }
    result = cyhal_i2c_configure(&I2C_scb3, &i2c_scb3_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }

    /*Initialize the timer counting up 1000 times per second*/
    result = cyhal_timer_init(&timer_obj, NC, NULL);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_timer_configure(&timer_obj, &timer_cfg);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_timer_set_frequency(&timer_obj, 1000);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_timer_start(&timer_obj);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

	/*Initialize SGP40 sensor*/
    if(sgp40_probe()  != SGP40_STATUS_OK)
    {
    	printf("SGP40 Sensor Failure\n\r");
    	printf("Initializing potentiometer ADC channel... \n\r");
        /*Configure the hardware*/
        result = app_hw_init();
        if (result != CY_RSLT_SUCCESS)
        {
        	printf("Application Failure.\n\r");
        	CY_ASSERT(0);
        }

        /*Switch to ADC measurements instead of SGP40*/
        printf("Use potentiometer for changing VOC Index values.\n\r");
        sgp_present = false;
    }

    /*VOC Index Algorithm Stack Initialization*/
    VocAlgorithm_init(&voc_algorithm_params);

    /*Initialize ViSi-Genie*/
    genieInitWithConfig(&userConfig);
    genieAttachEventHandler(myGenieEventHandler);
    resetDisplay();
    genieWriteContrast(15);
    genieWriteStr(0, GENIE_VERSION);

    for (;;)
    {
    	genieDoEvents(true);

    	/*Update gauge and LEDs once per 50 ms*/
    	if (uartGetMillis() >= waitPeriodDisplay)
    	{
        	if(gaugeVal > 500){gaugeVal = 500;}
        	genieWriteObject(GENIE_OBJ_ANGULAR_METER, 0, gaugeVal);
        	waitPeriodDisplay = uartGetMillis() + 50;

        	genieWriteObject(GENIE_OBJ_USER_LED, 0, (uint16_t)led1);
        	genieWriteObject(GENIE_OBJ_USER_LED, 1, (uint16_t)led2);
    	}

    	/*If the SGP40 is connected*/
    	if(sgp_present)
    	{
        	/*Measure/Read SGP40 sensor once per 100 ms*/
        	if (uartGetMillis() >= waitPeriodSensor)
        	{
        		if(!sensor_read)
        		{
        			sgp40_measure_raw();
        		}
        		else
        		{
        			sgp40_read_raw(&sgp_sraw_voc);
        			VocAlgorithm_process(&voc_algorithm_params, sgp_sraw_voc, &sgp_voc_index);
        			gaugeVal = (uint16_t)sgp_voc_index;
        		}

        	    waitPeriodSensor = uartGetMillis() + 100;

                /*Toggle measure/read cycles*/
                sensor_read = !sensor_read;
        	}
    	}
    	/*No SGP40 connected, rely on potentiometer*/
    	else
    	{
    		gaugeVal = (uint16_t)adc_data;
    	}


    	/*Update chart once per 10 seconds*/
    	if (uartGetMillis() >= waitPeriodChart)
    	{
        	genieWriteObject(GENIE_OBJ_SCOPE, 0, gaugeVal);
        	waitPeriodChart = uartGetMillis() + 10000;
    	}
    }
}

static cy_rslt_t ardu_uart_init(void)
{
	cy_rslt_t result;
	uint32_t actualbaud;

    /* Initialize the UART configuration structure */
    const cyhal_uart_cfg_t uart_config =
    {
        .data_bits = 8,
        .stop_bits = 1,
        .parity = CYHAL_UART_PARITY_NONE,
        .rx_buffer = NULL,
        .rx_buffer_size = 0
    };

    /* Initialize the UART Block */
    result = cyhal_uart_init(&ardu_uart, ARDU_TX, ARDU_RX, NC, NC, NULL, &uart_config);
	if (result != CY_RSLT_SUCCESS)
	{return result;}

	result = cyhal_uart_set_baud(&ardu_uart, ARDU_BAUD_RATE, &actualbaud);
	if (result != CY_RSLT_SUCCESS)
	{return result;}

	/*Connect internal pull-up resistor*/
	cyhal_gpio_configure(ARDU_RX, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP);

	return result;
}

/* UserApiConfig Handlers */
static bool uartAvailHandler(void)
{
  return (_Bool)cyhal_uart_readable(&ardu_uart);
}

static uint8_t uartReadHandler(void)
{
	uint8_t byte;

	(void)cyhal_uart_getc(&ardu_uart, &byte,0xFFFFFFFF);

	return byte;
}

static void uartWriteHandler(uint32_t val)
{
	(void)cyhal_uart_putc(&ardu_uart,val);
}

static uint32_t uartGetMillis(void)
{
  return cyhal_timer_read(&timer_obj);
}

/*Display Reset Procedure*/
static void resetDisplay(void)
{
	cyhal_gpio_write(ARDU_IO8, false);
	CyDelay(500);
	cyhal_gpio_write(ARDU_IO8, true);
	CyDelay(4000);
}

/*ViSi Genie Event Handler*/
static void myGenieEventHandler(void)
{
  GenieFrame Event;
  genieDequeueEvent(&Event);
  int32_t button_val = 0;

  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
	/*Look for 4Dbutton events*/
    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON)
    {
      /*4Dbutton0 Event*/
      if (Event.reportObject.index == 0)
      {
    	button_val = genieGetEventData(&Event);
    	if(button_val)
    	{
  		  led1 = true;
  		  cyhal_gpio_write(LED1, false);
    	}
    	else
		{
  		  led1 = false;
  		  cyhal_gpio_write(LED1, true);
		}
      }

      /*4Dbutton1 Event*/
      if (Event.reportObject.index == 1)
      {
    	button_val = genieGetEventData(&Event);
    	if(button_val)
    	{
  		  led2 = true;
  		  cyhal_gpio_write(LED2, false);
    	}
    	else
		{
  		  led2 = false;
  		  cyhal_gpio_write(LED2, true);
		}
      }
    }
  }
}

/*Initialization of the ADC*/
cy_rslt_t app_hw_init(void)
{
	cy_rslt_t result;

	/* Initialize the Arduino ADC Block.*/
	cyhal_adc_free(&adc_obj);
	result = cyhal_adc_init(&adc_obj, POT_ADC, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
    	goto return_err;
    }

    /* Initialize a channel A4 and configure it to scan P10_4 in single ended mode. */
    result  = cyhal_adc_channel_init_diff(&adc_potentiometer, &adc_obj, POT_ADC, CYHAL_ADC_VNEG, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {goto return_err;}

    cyhal_adc_register_callback(&adc_obj, &adc_continuous_event_handler, NULL);
    cyhal_adc_enable_event(&adc_obj, CYHAL_ADC_EOS, 0, true);

    /* Update ADC configuration */
    result = cyhal_adc_configure(&adc_obj, &adc_config);
    if(result != CY_RSLT_SUCCESS)
    {goto return_err;}

	return_err:
	return result;
}

/*ADC Measurement Finished Event*/
static void adc_continuous_event_handler(void* arg, cyhal_adc_event_t event)
{
    CY_UNUSED_PARAMETER(arg);
    CY_UNUSED_PARAMETER(event);

    /* Note: arg is configured below to be a pointer to the adc object */
    if(0u != (event & CYHAL_ADC_EOS))
    {
    	/*Take a measurement from the potentiometer channel*/
    	adc_data = cyhal_adc_read(&adc_potentiometer);

    	/*Format the data*/
    	if(adc_data < 0)
    	{adc_data = 0;}
    	adc_data = adc_data/4;
    	if(adc_data > 500)
    	{adc_data = 500;}
    }
}

void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/* [] END OF FILE */

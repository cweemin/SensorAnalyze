/******************************************************************************
 * Copyright (C) 2014 -2016  WeeMin Chan
 *
 * FileName: user_main.c
 *
 * Description: This project is to utilize 8266 and a 433 Receiver to Analyze
 *              and decode the Acurite 00609A1TX temperature/humidity sensor.
 *
 * Modification history:
 * 2016/4/1, v1.0 create this file.
 *******************************************************************************/

#include "esp_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <float.h>
#include "gpio.h"
#include "uart.h"
#include <ctype.h>

#define INCLUDE_vTaskSuspend 1

#define WAIT_TIME 50000 //Wait 400 ms
#define WANTED_PULSE 391
#define timing_t uint16

#define TEST_PIN 14
#define MAX_ALLOCATE 128
#define Q_NUM  (10)

#define DEMO_AP_SSID "weeminnet"
#define DEMO_AP_PASSWORD "weemin1979"
#define DEBUG
#ifdef DEBUG
#define DBG os_printf
#else
#define DBG 
#endif

//GLOBAL
uint32 old_timer=0;
GPIO_ConfigTypeDef test_gpio;
//LOCAL uint16 frac_ustime;
//LOCAL uint16 whole_ustime;
xTaskHandle ISR_Handler;
xQueueHandle ISR_Queue;
static void interrupt_gpio(void *para);

/******************************************************************************
 * FunctionName : interrupt_GPIO_UART
 * Description  : Interrupt service routine
 * Parameters   : none
 * Returns      : none
 *******************************************************************************/
//static void IRAM_ATTR interrupt_gpio(void *para) {
static void  IRAM_ATTR interrupt_gpio(void *para) {
    uint16 gpio_status = 0;
    uint8 i;
    uint32 new_timer = 0;
    //  portBASE_TYPE xHigherPriorityTaskWoken;

    /* We have not woken a task at the start of the ISR. */
    //  xHigherPriorityTaskWoken = pdFALSE;

    gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
    // TEST_PIN interrupts
    if (gpio_status & BIT(TEST_PIN)) {
	// Disable any interrupts from TEST_PIN
	gpio_pin_intr_state_set(GPIO_ID_PIN(TEST_PIN), GPIO_PIN_INTR_DISABLE);
	// Clear interupts status
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT(TEST_PIN));
	/* Notify the task that the reception is complete by setting the RX_BIT
	   in the task's notification value. */
	new_timer = system_get_time();
	uint16 delta = new_timer - old_timer; //get Time
	if (delta > WANTED_PULSE) { //Only send pulse we are interested in.. weened out other
	    xQueueSendFromISR( ISR_Queue, (void*) &delta, NULL);
	    /* Now the buffer is empty we can switch context if necessary. */
	}
	// Enables interrups from TEST_PIN
	old_timer = new_timer;
	gpio_pin_intr_state_set(GPIO_ID_PIN(TEST_PIN), test_gpio.GPIO_IntrType);

    }
}

char *ftoa(float num) {
    uint8 whole,dec,dec2;
    static char str[6];
    whole = num;
    dec = (num - whole) * 10;
    dec2 = (num - whole) * 100;
    if (dec2 - dec*10 > 5)
	dec++; //round up
    sprintf(str, "%d.%i", whole , dec);
    return str;
}

void emptyArray( uint8* arr, uint8 size) {
    uint8 i;
    for(i=0;i< size;i++)
	arr[i] = 0;
}


/*

The protocol consists of a "sync sequence" and follow by 3 repeated sequence of
each ending with a long pulse.

*/

/* Task to be created. */
void mainLoop( void * pvParameters )
{
    uint16 delta = 0;
    enum RX_STATE {
	sync_high,
	sync_low,
	sync_complete,
	get_data_high,
	get_data_low,
	end_transmission,
    } rx_state = sync_high;
    uint8 sync_count = 0;
    //timing_t lookback[MAX_ALLOCATE];
    uint8 i, j;
    uint8 value[5];
    bool wrap = false;

    os_printf("\nStart Main Loop\n");
    old_timer = system_get_time(); //Baseline
    for( ;; ) {
	/* Wait to be notified of an interrupt. */
	if(pdTRUE == xQueueReceive( ISR_Queue, &delta,(portTickType)portMAX_DELAY)) {
	    if (delta > WAIT_TIME) {
		DBG("\n");
	    }
/*
	    lookback[k++] = delta;
	    if (k >= MAX_ALLOCATE) {
		wrap = true;
		k=0;
	    }
	    if (delta > WAIT_TIME) {
		os_printf("{\n");
		if (wrap) {
		    for(i=k;i<MAX_ALLOCATE; i++) {
			os_printf("%d\n", lookback[i]);
		    }
		}
		for(i=0; i<k; i++) {
		    os_printf("%d\n",lookback[i]);

		}
		os_printf("}\n");
		k=0;
	    }
	    */
	    switch(rx_state) {
		case sync_high:
		    if( 500 < delta && delta < 590 ) {
			DBG("H");
			rx_state = sync_low;
			if (sync_count == 2 ) {
			    sync_count = 0;
			    rx_state = sync_complete;
			}
		    }
		    break;
		case sync_low:
		    if( 390 < delta && delta < 470 ) {
			DBG("L");
			//past_delta[i++] = (uint16) delta;
			sync_count++;
		    } else {
			sync_count = 0;
		    }
		    rx_state = sync_high;
		    break;
		case sync_complete:
		    if (8500 < delta && delta < 9000) {
		//	DBG("S %d %d\n", (delta - 8500), (9000 - delta));
			DBG("S");
			i=0;
			j=0;
			value[j]=0;
			rx_state = get_data_high;
		    } else {
			rx_state = sync_high;
		    }
		    break;
		case get_data_high: 
		    if ( 470 < delta && delta < 600 ) {
		//	DBG("H %d %d\n", (delta - 480), (600 - delta));
			rx_state = get_data_low;
		    } else {
			DBG("^%d\n", delta);
			rx_state = sync_high;
		    }
		    break;
		case get_data_low:
		    if ( delta < 2010 ) { //1
			value[j] = value[j] << 1;
			if (delta < 1100 ) {
			    //DBG("0", (delta - 900), (1100 - delta));
			    DBG("0");
			    value[j] = value[j] & 0b11111110;
			} else {
			    //DBG("1", (delta - 1850), (2000 - delta));
			    DBG("1");
			    value[j] = value[j] | 0b00000001;
			}
			i++;
			if (i == 8 ) {
			    DBG(" ");
			    j++;
			    value[j] = 0;
			    if ( j > 5) {
				os_printf("Error: we got more than 5 bytes!\n");
				j=0;
			    }
			    i=0;
			}
			rx_state = get_data_high;
		    } else {
			//digestData(value,j);
			os_printf("\nGot Data %X %X %X %X %X!\n", value[0], value[1], value[2], value[3],value[4]);
			os_printf("Temperature: %sC", ftoa((float) value[2] /10.0));
			os_printf(" %sF", ftoa(((float) value[2] * 9.0 / 50.0) + 32.0));
			os_printf(" Humidity %d%\n", value[3]);

			if (45000 < delta) {
			    DBG("E");
			} else if (8500 < delta && delta < 11100) {
		//	    DBG("S %d %d\n", (delta - 8500), (11100 - delta));
			    DBG("S");
			} else {
			//    DBG("|%d\n", delta);
			    DBG("|");
			}
			rx_state = sync_high;
		    }
		    break;
		default:
		    os_printf("Error state\n");
		    break;
	    }
	}
    }
    vTaskDelete(NULL);
}



/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
 *******************************************************************************/
void user_init(void)
{
    uart_init_new();
    uart_div_modify(0, UART_CLK_FREQ / 115200);
    os_printf("SDK version:%s\n", system_get_sdk_version());


    //Setup Interrupt


    os_printf("WeeMin's GPIO Test!\n");
    struct station_config * config = (struct station_config *)zalloc(sizeof(struct
		station_config));
    sprintf(config->ssid, DEMO_AP_SSID);
    sprintf(config->password, DEMO_AP_PASSWORD);
    wifi_station_set_config(config);
    free(config);
    wifi_station_connect(); 

    //Start Task
    //FreeRtos
    ISR_Queue = xQueueCreate( 10 , sizeof(uint16) );
    xTaskCreate(mainLoop, "Loop" ,sizeof(timing_t) * MAX_ALLOCATE + 20, NULL, tskIDLE_PRIORITY, ISR_Handler); 


    //Start ISR
    //Test gpio

    test_gpio.GPIO_Pin = BIT(TEST_PIN);
    test_gpio.GPIO_Mode = GPIO_Mode_Input;
    test_gpio.GPIO_IntrType = GPIO_PIN_INTR_ANYEDGE;
  //  test_gpio.GPIO_Pullup = GPIO_PullUp_DIS;
    test_gpio.GPIO_Pullup = GPIO_PullUp_EN;
    gpio_config(&test_gpio);


    //  GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT(ETS_GPIO_INUM | ETS_UART_INUM)); 

    gpio_intr_handler_register( interrupt_gpio, NULL);                   // Register the interrupt function
    //Enable the GPIO interrupt
    GPIO_INTR_ENABLE();       


}



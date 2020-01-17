#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_types.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_intr_alloc.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include <stdio.h>
#include <string.h>

//----Requests to MEDIDOR----------------------------------------------------------
char* identificador = "MEDIDOR \r";
char* activa = "ENERGIA\r";
char* reactiva = "ENERGIA R\r";

char* voltaje_A = "VOLTAJE A\r";
char* corriente_A = "CORRIENTE A\r";
char* potencia_A = "POTENCIA A\r";

char* voltaje_C = "VOLTAJE C\r";
char* corriente_C = "CORRIENTE C\r";
char* potencia_C = "POTENCIA C\r";

char* corte = "ABRIR\r";
char* reco = "CERRAR\r";

//----TIMER DEFINES----------------------------------------------------------
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (10) // sample test interval for the first timer
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

//----UART DEFINES----------------------------------------------------------
#define UART_0 UART_NUM_0
#define UART_1 UART_NUM_1
#define TX_MEDIDOR  (GPIO_NUM_5)
#define RX_MEDIDOR  (GPIO_NUM_4)
#define TX_WISOL  (GPIO_NUM_23)
#define RX_WISOL  (GPIO_NUM_22)
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

//----GLOBAL VARS----------------------------------------------------------
static intr_handle_t handle_console0;
static intr_handle_t handle_console1;
uint8_t global_flag=0;
uint8_t rxbuf[256]; // Receive buffer to collect incoming data
uint16_t msg_len;
uint8_t MEDIDOR_response[50];



//----START FUNCTIONS----------------------------------------------------------

/*
 * Delay on ms
 */
void delay_ms(uint16_t time_ms)
{
	vTaskDelay(time_ms / portTICK_PERIOD_MS);
}


/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
void IRAM_ATTR timer_intr_handle(void *para)
{
	//uart_write_bytes(UART_0, (const char*) "timer event", 11);

	// Clear the interrupt
	TIMERG0.int_clr_timers.t0 = 1;

	/* After the alarm has been triggered
	we need enable it again, so it is triggered the next time */
	TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

	timer_disable_intr(TIMER_GROUP_0, TIMER_0);
	timer_pause(TIMER_GROUP_0, TIMER_0);

	global_flag = 1;
}


/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 */
static void timer_initialize(int timer_idx,
    bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_intr_handle,(void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    //timer_enable_intr(TIMER_GROUP_0, timer_idx);
    //timer_start(TIMER_GROUP_0, timer_idx);

    //Disable timer
    timer_disable_intr(TIMER_GROUP_0, TIMER_0);
}


/*
 * Define UART interrupt subroutine to ackowledge interrupt
 */
static void IRAM_ATTR uart0_intr_handle(void *arg)
{
	uint16_t rx_fifo_len;
    //uint16_t status;
    uint16_t i=0;

    //status = UART0.int_st.val; // read UART interrupt Status
    rx_fifo_len = msg_len = UART0.status.rxfifo_cnt; // read number of bytes in UART buffer

    while(rx_fifo_len)
    {
        rxbuf[i] = UART0.fifo.rw_byte; // read all bytes
        rx_fifo_len--;
        i++;
    }

    // after reading bytes from buffer clear UART interrupt status
    uart_clear_intr_status(UART_0, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);

    // a test code or debug code to indicate UART receives successfully,
    // you can redirect received byte as echo also
    //uart_write_bytes(UART_0, (const char*)rxbuf, msg_len);
    //uart_write_bytes(UART_0, (const char*) "RX Done", 7);
    uart_disable_rx_intr(UART_0);

    global_flag = 2;
}


/*
 * Define UART interrupt subroutine to ackowledge interrupt
 */
static void IRAM_ATTR uart1_intr_handle(void *arg)
{
    uint16_t rx_fifo_len;
    //uint16_t status;
    uint16_t i=0;

    //status = UART0.int_st.val; // read UART interrupt Status
    rx_fifo_len = msg_len = UART1.status.rxfifo_cnt; // read number of bytes in UART buffer

    while(rx_fifo_len)
    {
        rxbuf[i] = UART1.fifo.rw_byte; // read all bytes
        rx_fifo_len--;
        i++;
    }

    // after reading bytes from buffer clear UART interrupt status
    uart_clear_intr_status(UART_1, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);

    // a test code or debug code to indicate UART receives successfully,
    // you can redirect received byte as echo also
    //uart_write_bytes(UART_0, (const char*)rxbuf, msg_len);
    //uart_write_bytes(UART_0, (const char*) "RX Done", 7);
    uart_disable_rx_intr(UART_1);

    global_flag = 2;
}


/*
 * Define UART initial parameters and interrupts
 */
void uart0_initialize()
{
	/* Configure parameters of an UART driver,
	 * communication pins and install the driver */
	uart_config_t uart_config =
	{
	     .baud_rate = 9600,
	     .data_bits = UART_DATA_8_BITS,
	     .parity = UART_PARITY_DISABLE,
	     .stop_bits = UART_STOP_BITS_1,
	     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	//Apply configuration
	uart_param_config(UART_0, &uart_config);
	//Set UART pins (using UART0 default pins ie no changes.)
	uart_set_pin(UART_0, TX_MEDIDOR, RX_MEDIDOR, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	//Install UART driver, and get the queue.
	uart_driver_install(UART_0, BUF_SIZE * 2, 0, 0, NULL, 0);

	// release the pre registered UART handler/subroutine
	uart_isr_free(UART_0);
	// register new UART subroutine
	uart_isr_register(UART_0,uart0_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console0);
	// enable RX interrupt
	//uart_enable_rx_intr(UART_0);
	// disable RX interrupt
	uart_disable_rx_intr(UART_0);
}


/*
 * Define UART initial parameters and interrupts
 */
void uart1_initialize()
{
	/* Configure parameters of an UART driver,
	 * communication pins and install the driver */
	uart_config_t uart_config =
	{
	     .baud_rate = 9600,
	     .data_bits = UART_DATA_8_BITS,
	     .parity = UART_PARITY_DISABLE,
	     .stop_bits = UART_STOP_BITS_1,
	     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	//Apply configuration
	uart_param_config(UART_1, &uart_config);
	//Set UART pins (using UART0 default pins ie no changes.)
	uart_set_pin(UART_1, TX_WISOL, RX_WISOL, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	//Install UART driver, and get the queue.
	uart_driver_install(UART_1, BUF_SIZE * 2, 0, 0, NULL, 0);

	// release the pre registered UART handler/subroutine
	uart_isr_free(UART_1);
	// register new UART subroutine
	uart_isr_register(UART_1,uart1_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console1);
	// enable RX interrupt
	//uart_enable_rx_intr(UART_0);
	// disable RX interrupt
	uart_disable_rx_intr(UART_1);
}


/*
 * Wait for response
 */
void wait_response(uint8_t source)
{
	uint8_t uart_port = source;
	//Set & Enable components ISRs
	global_flag = 0;
	uart_flush (uart_port);
	uart_enable_rx_intr(uart_port);
	timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
	timer_enable_intr(TIMER_GROUP_0, TIMER_0);
	timer_start(TIMER_GROUP_0, TIMER_0);

	//wait for event
	while(global_flag == 0)delay_ms(100);


	if(global_flag == 1)
	{
		uart_disable_rx_intr(uart_port);
		//uart_write_bytes(uart_port, (const char*) "TIMEOUT\r", 8);
	}
	else if(global_flag == 2)
	{
		timer_disable_intr(TIMER_GROUP_0, TIMER_0);
		timer_pause(TIMER_GROUP_0, TIMER_0);
		//uart_write_bytes(uart_port, (const char*) "RX Done: ", 8);
		//uart_write_bytes(uart_port, (const char*)rxbuf, msg_len);
	}
}


/*
 * Request data to MEDIDOR
 */
void request_MEDIDOR(uint8_t data_requested)
{
	switch(data_requested)
	{
	case(0x31):
		uart_write_bytes(UART_0, (const char*)identificador, strlen(identificador));
	    break;
	case(0x32):
		uart_write_bytes(UART_0, (const char*)activa, strlen(activa));
	    break;
	case(0x33):
		uart_write_bytes(UART_0, (const char*)reactiva, strlen(reactiva));
		break;
	case(0x34):
		uart_write_bytes(UART_0, (const char*)voltaje_A, strlen(voltaje_A));
		break;
	case(0x35):
		uart_write_bytes(UART_0, (const char*)corriente_A, strlen(corriente_A));
		break;
	case(0x36):
		uart_write_bytes(UART_0, (const char*)potencia_A, strlen(potencia_A));
		break;
	case(0x37):
		uart_write_bytes(UART_0, (const char*)voltaje_C, strlen(voltaje_C));
		break;
	case(0x38):
		uart_write_bytes(UART_0, (const char*)corriente_C, strlen(corriente_C));
		break;
	case(0x39):
		uart_write_bytes(UART_0, (const char*)potencia_C, strlen(potencia_C));
		break;
	case(0x41):
		uart_write_bytes(UART_0, (const char*)corte, strlen(corte));
		break;
	case(0x42):
		uart_write_bytes(UART_0, (const char*)reco, strlen(reco));
		break;
	default:
		uart_write_bytes(UART_0, (const char*) "ERR\r", 4);
	    break;
	}

	wait_response(UART_0);
}


/*
 * MAIN CODE
 */
void app_main(void)
{
	uart0_initialize();
	uart1_initialize();
	timer_initialize(TIMER_0, TEST_WITH_RELOAD, TIMER_INTERVAL0_SEC);

	for(;;)
	{
		//Request action to WISOL
		uart_write_bytes(UART_1, (const char*) "CMD to SEND to MEDIDOR...\r", 26);
		wait_response(UART_1);

		if(global_flag == 1)
		{
			uart_write_bytes(UART_1, (const char*) "TIMEOUT\r", 8);
		}
		else if(global_flag == 2)
		{
			uart_write_bytes(UART_1, (const char*) "CMD received: \r", 15);
			uart_write_bytes(UART_1, (const char*)rxbuf, msg_len);

			delay_ms(1000);

			//Send request cmd to MEDIDOR
			request_MEDIDOR(rxbuf[0]);

			if(global_flag == 1)
			{
				uart_write_bytes(UART_1, (const char*) "TIMEOUT FROM MEDIDOR\r", 21);
			}
			else if(global_flag == 2)
			{
				int i=0,count;
				count=msg_len;
				while(count)
				{
					MEDIDOR_response[i] = rxbuf[i]; // read all bytes
					count--;
					i++;
				}
			    //Return MEDIDOR data to WISOL
			    uart_write_bytes(UART_1, (const char*)MEDIDOR_response, (msg_len-1)); //(msg_len-1) because last char is always 0x0A (??????)
			}
		}
		delay_ms(5000);
	}
}

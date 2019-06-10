#include "asf.h"
#include "main.h"
#include <string.h>

#include "OLED1/gfx_mono_ug_2832hsweg04.h"
#include "OLED1/gfx_mono_text.h"
#include "OLED1/sysfont.h"


/* BUTTONS */
#define BUT1_PIO           PIOD
#define BUT1_PIO_ID        ID_PIOD
#define BUT1_PIO_IDX       28u
#define BUT1_PIO_IDX_MASK  (1u << BUT1_PIO_IDX)

#define BUT2_PIO           PIOC
#define BUT2_PIO_ID        ID_PIOC
#define BUT2_PIO_IDX       31u
#define BUT2_PIO_IDX_MASK  (1u << BUT2_PIO_IDX)

#define BUT3_PIO           PIOA
#define BUT3_PIO_ID        ID_PIOA
#define BUT3_PIO_IDX       19u
#define BUT3_PIO_IDX_MASK  (1u << BUT3_PIO_IDX)


/* LEDS */
#define LED1_PIO           PIOA
#define LED1_PIO_ID        ID_PIOA
#define LED1_PIO_IDX       0u
#define LED1_PIO_IDX_MASK  (1u << LED1_PIO_IDX)

#define LED2_PIO           PIOC
#define LED2_PIO_ID        ID_PIOC
#define LED2_PIO_IDX       30u
#define LED2_PIO_IDX_MASK  (1u << LED2_PIO_IDX)

#define LED3_PIO           PIOB 
#define LED3_PIO_ID        ID_PIOB
#define LED3_PIO_IDX       2u
#define LED3_PIO_IDX_MASK  (1u << LED3_PIO_IDX)

/************************************************************************/
/* GENERIC DEFINES                                                      */
/************************************************************************/

/************************************************************************/
/* generic globals                                                      */
/************************************************************************/

/************************************************************************/
/*  RTOS    (defines + globals)                                         */
/************************************************************************/

#define TASK_STRING_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_STRING_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_IO_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_IO_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_OLED_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY        (tskIDLE_PRIORITY)

//Semaphores
SemaphoreHandle_t xSemaphoreLED;

//QueueHandle
QueueHandle_t xQueueUart;
QueueHandle_t xQueueOLed;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}
extern void vApplicationIdleHook(void)
{
	
}
extern void vApplicationTickHook(void)
{
}
extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}


/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

int protocol_check_led(char *string);
int io_init(void);
void led_on(uint id, uint on);

/************************************************************************/
/* IRQS / callbacks                                                     */
/************************************************************************/

void USART1_Handler(void){

  BaseType_t xHigherPriorityTaskWoken = pdFALSE; // freeRTOs
  uint32_t ret = usart_get_status(CONF_UART);  // ACK IRQ
  
  char dado;
	
	
  // Por qual motivo entrou na interrupçcao ? Pode ser varios!
  //  1. RX: Dado disponível para leitura
  if(ret & US_IER_RXRDY){
    // LER DADO DA UART
    // ENVIAR VIA FILA
		usart_serial_getchar(CONF_UART, &dado);
		xQueueSendFromISR( xQueueUart, &dado, &xHigherPriorityTaskWoken);
	

	
  } 
}

void button0_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreLED, &xHigherPriorityTaskWoken);	
}

void button1_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreLED, &xHigherPriorityTaskWoken);	
}

void button2_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreLED, &xHigherPriorityTaskWoken);	
}


/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void task_string(void *pvParameters){
    
  xQueueUart = xQueueCreate( 52, sizeof( char ) );	
  
  usart_enable_interrupt(CONF_UART, US_IER_RXRDY);
  NVIC_EnableIRQ(CONSOLE_UART_ID);
  NVIC_SetPriority(CONSOLE_UART_ID, 5);
 

		
  char b[52];
  uint i = 0;
  
  while(1){
   if( xQueueReceive(xQueueUart, &(b[i]), 10)){
	   
      if(b[i]=='\n'){
        b[i] = NULL;
        i=0;
        printf("recebido: %s\n", b);
		int verif =  protocol_check_led(b);
		if (verif){
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGive(xSemaphoreLED);
		}
		if (strstr(b, "OLED:") != NULL){
			xQueueSendFromISR(xQueueOLed, &b, 0);
		}
      }
	  else{
        i++;
      }          
   }    
  } 
}  


void task_io(void *pvParameters){
	
	xSemaphoreLED = xSemaphoreCreateBinary();
	io_init();
	int pisca = 0;
	
	while (true) {
			
		if( xSemaphoreTake(xSemaphoreLED, ( TickType_t ) 0) == pdTRUE ){
			pisca = !pisca;
		}
	
		if (pisca){
				
				for (int i = 0; i <= 2; i++)
				led_on(i,0);
				vTaskDelay(100);
				for (int i = 0; i <= 2; i++)
				led_on(i,1);
				vTaskDelay(100);
		}
			
	}
}

void task_oled1(void *pvParameters){
	xQueueOLed = xQueueCreate( 2, sizeof( char[52]));
	gfx_mono_ssd1306_init();	char buf[52];
	while(1){
		if( xQueueReceive(xQueueOLed, &buf, 50)){
			gfx_mono_draw_string(&buf[5], 25,20, &sysfont);
		}
		vTaskDelay(100);
	}
	
 
}  




/************************************************************************/
/* CONFIGS/ INITS                                                       */
/************************************************************************/

static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate =		CONF_UART_BAUDRATE,
		.charlength =	CONF_UART_CHAR_LENGTH,
		.paritytype =	CONF_UART_PARITY,
		.stopbits =		CONF_UART_STOP_BITS,
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}


/************************************************************************/
/* functions                                                            */
/************************************************************************/


int protocol_check_led(char *string){
	char led[52] = "LEDS";
	int verif = strcmp(led,string);
	if (verif == 0){
		return 1;
	}
	else{
		return 0;
	}
}
	

int io_init(void){

	
	// Inicializa e configura os LEDS
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0 );	
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 0, 0, 0 );
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0 );		
	
	// Inicializa clock do periférico PIO responsavel pelos botoes
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);

	// Configura PIO para lidar com os pinos dos botoes como entrada
	// com pull-up
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada

	pio_handler_set(BUT1_PIO,
	BUT1_PIO_ID,
	BUT1_PIO_IDX,
	PIO_IT_FALL_EDGE,
	button0_callback);
	
	pio_handler_set(BUT2_PIO,
	BUT2_PIO_ID,
	BUT2_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	button1_callback);
	
	pio_handler_set(BUT3_PIO,
	BUT3_PIO_ID,
	BUT3_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	button2_callback);	

	NVIC_SetPriority(BUT3_PIO_ID, 5);	
	NVIC_SetPriority(BUT1_PIO_ID, 5);
	NVIC_SetPriority(BUT2_PIO_ID, 5);

	// Configura NVIC para receber interrupcoes do PIO do botao
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_EnableIRQ(BUT3_PIO_ID);
	
	// Ativa interrupção
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	
	return 1;

}
  
void led_on(uint id, uint on){
	if(on){
		if (id == 0){
			pio_set(PIOA,LED1_PIO_IDX_MASK);
		}
		if(id == 1){
			pio_set(PIOC,LED2_PIO_IDX_MASK);
		}
		if(id == 2){
			pio_set(PIOB,LED3_PIO_IDX_MASK);
		}
	}
	else{
		if (id == 0){
			pio_clear(PIOA,LED1_PIO_IDX_MASK);
		}
		if(id == 1){
			pio_clear(PIOC,LED2_PIO_IDX_MASK);
		}
		if(id == 2){
			pio_clear(PIOB,LED3_PIO_IDX_MASK);
		}
	}

}


/************************************************************************/
/* MAIN                                                                 */
/************************************************************************/
int main(void)
{
	/* Initialize the board. */
	sysclk_init();
	board_init();

	/* Initialize the UART console. */
	configure_console();
   
	if (xTaskCreate(task_string, "string", TASK_STRING_STACK_SIZE, NULL, TASK_STRING_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create string task\r\n");
	}

	if (xTaskCreate(task_io, "io", TASK_IO_STACK_SIZE, NULL, TASK_IO_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create io task\r\n");
	}	

	if (xTaskCreate(task_oled1, "oled1", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create led task\r\n");
	}	

	vTaskStartScheduler();
	
	while(1) {};
	return 0;

}

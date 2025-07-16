#ifndef HAL_GSM_H_
#define HAL_GSM_H_
#include "stm32f4xx.h"
#include "string.h"
#include "stm32f4_discovery.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
void usart2_init();
void send_command_to_init_gsm(void *parameter);
void SIM900_PutFrame(char *buf);
uint8_t SIM900_GetFrame(void *buf, void *len);
void gsm_call(void *num);
void send_sms(void *num);
void gsm_os_init(void *parameter);
#endif
#include "GSM.h"
#include "GSM_config.h"
TaskHandle_t handle_send_command_to_init_gsm = NULL;
TaskHandle_t handle_gsm_call = NULL;
TaskHandle_t handle_send_sms = NULL;
TaskHandle_t handle_gsm_os_init = NULL;
TaskHandle_t task2_h;
QueueHandle_t queue1 = NULL;
SemaphoreHandle_t sem1 = NULL;
char buffer[10];
uint8_t flag_delay = 0, current = 0;
#define SIM900_HAL_Enable usart2_init
char test_connect[] = "AT\r\n";      // //Once the handshake test is successful, it will back to OK
char quality_test[] = "AT+CSQ\r\n";  // Signal quality test, value range is 0-31 , 31 is the best
char sim_info[] = "AT+CCID\n";       // get the SIM card number this tests that the SIM card is found OK
char net_reg[] = "AT+CREG?\n";       // Check whether it has registered in the network
char config_mod[] = "AT+CMGF=1\r\n"; // Selects SMS message format as text.
char MESSAGE[] = "hello from gsm module";
char ctrl_z[] = "26";
void usart2_init()
{
  GPIO_InitTypeDef gpio_uart_pins;
  USART_InitTypeDef uart2_init;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  memset(&gpio_uart_pins, 0, sizeof(gpio_uart_pins));
  gpio_uart_pins.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  gpio_uart_pins.GPIO_Mode = GPIO_Mode_AF;
  gpio_uart_pins.GPIO_PuPd = GPIO_PuPd_UP;
  gpio_uart_pins.GPIO_OType = GPIO_OType_PP;
  gpio_uart_pins.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio_uart_pins);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
  EXTI_InitTypeDef exti_init;
  exti_init.EXTI_Line = EXTI_Line0;
  exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
  exti_init.EXTI_Trigger = EXTI_Trigger_Rising;
  exti_init.EXTI_LineCmd = ENABLE;
  EXTI_Init(&exti_init);
  NVIC_SetPriority(EXTI0_IRQn, 6);
  NVIC_EnableIRQ(EXTI0_IRQn);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // PA2
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // PA3
  memset(&uart2_init, 0, sizeof(uart2_init));
  uart2_init.USART_BaudRate = 9600;
  uart2_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  uart2_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  uart2_init.USART_Parity = USART_Parity_No;
  uart2_init.USART_StopBits = USART_StopBits_1;
  uart2_init.USART_WordLength = USART_WordLength_8b;
  USART_Init(USART2, &uart2_init);
  USART2->CR1 |= (USART_CR1_RE | USART_CR1_TE); // RX, TX enable.
  USART2->CR1 |= (1 << 6);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  NVIC_SetPriority(USART2_IRQn, 5);
  NVIC_EnableIRQ(USART2_IRQn);
  USART_Cmd(USART2, ENABLE);
  printf("exit from usart2\n");
}
void gsm_os_init(void *parameter)
{
  printf("inside gsm first\n");
  char *m = "01028519757;\r\n";
  usart2_init();
  queue1 = xQueueCreate(10, sizeof(char));
  sem1 = xSemaphoreCreateMutex();
  xTaskCreate(send_command_to_init_gsm, "send_command_to_init_gsm", 128, NULL, 5, &handle_send_command_to_init_gsm);
  xTaskCreate(gsm_call, "gsm_call", 128, (void *)m, 5, &handle_gsm_call);
  xTaskCreate(send_sms, "send_sms", 128, NULL, 5, &handle_send_sms);
  while (1)
  {
    printf("exit gsm first\n");
    vTaskDelete(handle_gsm_os_init);
  }
}
void send_command_to_init_gsm(void *parameter)
{
  printf("first task_send command to init gsm \n");
  uint32_t current_notification_value = 0;
  UBaseType_t p1;
  while (1)
  {
    if (xTaskNotifyWait(0, 0, &current_notification_value, portMAX_DELAY) == pdTRUE)
    {
      p1 = uxTaskPriorityGet(handle_send_command_to_init_gsm);
      vTaskDelay(pdMS_TO_TICKS(20));
      SIM900_PutFrame(test_connect);
      printf("%s", test_connect);
      vTaskDelay(pdMS_TO_TICKS(5000));
      SIM900_PutFrame(quality_test);
      printf("%s", quality_test);
      vTaskDelay(pdMS_TO_TICKS(3000));
      SIM900_PutFrame(net_reg);
      printf("%s", net_reg);
      vTaskDelay(pdMS_TO_TICKS(3000));
      printf("task send command \n");
      xTaskNotify(handle_send_sms, 0x0, eIncrement);
      taskYIELD();
    }
  }
}
void SIM900_PutFrame(char *buf)
{
  xSemaphoreTake(sem1, portMAX_DELAY);
  uint16_t len = strlen(buf);
  for (int i = 0; i < len; i++)
  {
    USART_SendData(USART2, buf[i]);
  }
  xSemaphoreGive(sem1);
}
void gsm_call(void *num)
{
  printf("second task to call gsm \n");
  uint32_t current_notification_value = 0;
  char call[] = "ATD"; // call a specified number.
  while (1)
  {
    if (flag_delay == 1 && current == 0)
    {
      strcat(call, (char *)num);
      SIM900_PutFrame(call);
      printf("%s", call);
      printf("call sms \n");
      flag_delay = 0;
      current = 0;
    }
  }
}
void send_sms(void *num)
{
  unsigned char caller_id[20] = "01028519757"; // will contain the phone number +33612345678
  unsigned char sms_header[30] = {0};
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(25000);
  const TickType_t xFrequency1 = pdMS_TO_TICKS(1000);
  xLastWakeTime = xTaskGetTickCount();
  uint32_t current_notification_value = 0;
  while (1)
  {
    if (xTaskNotifyWait(0, 0, &current_notification_value, portMAX_DELAY) == pdTRUE)
    {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      SIM900_PutFrame("AT+CMGF=1\r\n"); // configure mode as text
      vTaskDelayUntil(&xLastWakeTime, xFrequency1);
      strcpy(sms_header, "AT+CMGS=\""); // Make AT command to send a SMS
      strcat(sms_header, caller_id);    // Add recipient phone number
      strcat(sms_header, "\"\r");       // Terminate the string with ", 129 [ENTER]
      SIM900_PutFrame(sms_header);      // configure mode as text
      printf("%s", sms_header);
      vTaskDelay(pdMS_TO_TICKS(2000));
      SIM900_PutFrame("اHI HAZEM TEAM LEADER OF ACCIDENT SYSTEM \x1a");
      vTaskDelay(pdMS_TO_TICKS(20000));
      flag_delay = 1;
      current = 0;
      printf("send sms\n");
      xTaskNotify(handle_gsm_call, 0x0, eIncrement);
    }
  }
}
void EXTI0_IRQHandler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  EXTI_ClearITPendingBit(EXTI_Line0);
  xTaskNotifyFromISR(handle_send_command_to_init_gsm, 0, eNoAction, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken)
  {
    printf("there is higher priority\n");
    taskYIELD();
  }
}
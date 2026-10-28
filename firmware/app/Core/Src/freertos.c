/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "platform_bsp_key.h"
#include "platform_bsp_led.h"
#include "platform_bsp_buzzer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 环形缓冲必须是 2 的幂，下面用掩码取模 */
#define UART_TEST_RX_BUF_SIZE    128U
#define UART_TEST_ECHO_BUF_SIZE  32U    /* 一次最多回显多少字节，115200 下约 2.8 ms */
#define UART_TEST_TIMEOUT_MS     100U   /* 阻塞发送上限 */

/* 按键/蜂鸣器台架测试：按一次 PB2 翻转一次蜂鸣器，LED 同步显示同一状态。
   10 ms 采样是经验值：去抖阈值 20 ms 即 2~3 个采样点，既能滤掉抖动，
   又不会让人觉得"按了没反应" */
#define KEY_TEST_PERIOD_MS       10U
#define KEY_TEST_STACK_WORDS     96U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 串口回环测试：中断逐字节收 -> 环形缓冲 -> 任务成段原样发回。
   必须走中断：F103 的 USART 只有一个字节的接收寄存器，
   任务里轮询/逐字节回显的速度和 115200 的来速打平，突发数据必丢 */
static uint8_t  g_uart_test_rx_byte;                      /* 中断单字节落点 */
static uint8_t  g_uart_test_rx_buf[UART_TEST_RX_BUF_SIZE];
static uint8_t  g_uart_test_echo_buf[UART_TEST_ECHO_BUF_SIZE];
static volatile uint16_t g_uart_test_rx_head;             /* 中断写，任务读 */
static volatile uint16_t g_uart_test_rx_tail;             /* 任务写，中断读 */
static volatile uint8_t  g_uart_test_rx_alert;            /* 缓冲满/UART 错误，交给任务上报 */

/* 按键测试任务。属性放在 USER CODE 里，重新生成不会丢。
   测试任务不会被外部挂起/删除，所以不保存句柄 */
static const osThreadAttr_t g_key_test_task_attributes = {
  .name = "keyTestTask",
  .stack_size = KEY_TEST_STACK_WORDS * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartKeyTestTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  (void)osThreadNew(StartKeyTestTask, NULL, &g_key_test_task_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* 先挂上中断接收。HAL 的发送(gState)和中断接收(RxState)状态独立，
     任务里阻塞发送不会打断接收 */
  if (HAL_UART_Receive_IT(&huart3, &g_uart_test_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }

  /* Infinite loop */
  for(;;)
  {
    uint16_t len = 0U;

    if (g_uart_test_rx_alert != 0U)
    {
      static const uint8_t alert_msg[] = "\r\n[uart_test] RX overflow or UART error\r\n";

      g_uart_test_rx_alert = 0U;
      (void)HAL_UART_Transmit(&huart3, (uint8_t *)alert_msg,
                              (uint16_t)(sizeof(alert_msg) - 1U),
                              UART_TEST_TIMEOUT_MS);
    }

    /* 一次尽量多搬几个字节再发：成段发送才能跟上连续来流 */
    while ((g_uart_test_rx_tail != g_uart_test_rx_head) &&
           (len < (uint16_t)UART_TEST_ECHO_BUF_SIZE))
    {
      g_uart_test_echo_buf[len] = g_uart_test_rx_buf[g_uart_test_rx_tail];
      g_uart_test_rx_tail = (uint16_t)((g_uart_test_rx_tail + 1U) &
                                       (UART_TEST_RX_BUF_SIZE - 1U));
      len++;
    }

    if (len > 0U)
    {
      (void)HAL_UART_Transmit(&huart3, g_uart_test_echo_buf, len,
                              UART_TEST_TIMEOUT_MS);
    }
    else
    {
      osDelay(1U);   /* 没数据就睡 1 ms，中断继续收 */
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief  按键/蜂鸣器台架测试任务。
  * @note   按一次键翻转一次蜂鸣器，LED 跟随同一状态：灯变了但没响，说明是
  *         蜂鸣器那一侧的问题；灯也不变，说明按键没被采到。
  *         采样间隔用内核 tick 实测而不是直接传 KEY_TEST_PERIOD_MS：
  *         调度抖动会拉长两次采样之间的真实时间，传固定值会让去抖变慢。
  */
void StartKeyTestTask(void *argument)
{
  platform_key_state_t key;
  platform_bool_t      buzzer_on  = PLATFORM_FALSE;
  platform_bool_t      was_pressed = PLATFORM_FALSE;
  uint32_t             last_tick  = osKernelGetTickCount();

  (void)argument;

  for (;;)
  {
    uint32_t now     = osKernelGetTickCount();
    uint32_t elapsed = now - last_tick;   /* 无符号减法，tick 回绕也成立 */

    last_tick = now;

    if ((platform_key_poll(PLATFORM_KEY_ID_0, elapsed) == PLATFORM_ERR_OK) &&
        (platform_key_get_state(PLATFORM_KEY_ID_0, &key) == PLATFORM_ERR_OK))
    {
      /* 只在"松开 -> 按下"的边沿翻转：按住不放不会来回叫 */
      if ((key.pressed == PLATFORM_TRUE) && (was_pressed == PLATFORM_FALSE))
      {
        buzzer_on = (buzzer_on == PLATFORM_TRUE) ? PLATFORM_FALSE : PLATFORM_TRUE;
        (void)platform_buzzer_set(buzzer_on);
        (void)platform_led_set(buzzer_on);
      }
      was_pressed = key.pressed;
    }

    osDelay(KEY_TEST_PERIOD_MS);
  }
}

/**
  * @brief  单字节接收完成：存入环形缓冲并重新挂接收。
  * @note   ISR 里只做搬运，不解析、不打印；缓冲满时丢新字节并置标志。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    uint16_t next = (uint16_t)((g_uart_test_rx_head + 1U) &
                               (UART_TEST_RX_BUF_SIZE - 1U));

    if (next != g_uart_test_rx_tail)
    {
      g_uart_test_rx_buf[g_uart_test_rx_head] = g_uart_test_rx_byte;
      g_uart_test_rx_head = next;
    }
    else
    {
      g_uart_test_rx_alert = 1U;
    }

    (void)HAL_UART_Receive_IT(&huart3, &g_uart_test_rx_byte, 1U);
  }
}

/**
  * @brief  串口错误（过载等）：不重新挂接收的话，接收会永久停摆。
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    g_uart_test_rx_alert = 1U;
    (void)HAL_UART_Receive_IT(&huart3, &g_uart_test_rx_byte, 1U);
  }
}
/* USER CODE END Application */


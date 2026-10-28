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
#include "platform_bsp_motor.h"
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

/* ---------------- 台架测试开关：同一时间只开一个 ---------------- */
/* 电机测试会让轮子自己转，编码器测试要靠手转，两个同时开读数会被污染 */
#define BENCH_MOTOR_TEST_ENABLE    0
#define BENCH_ENCODER_TEST_ENABLE  1

#define BENCH_MOTOR_COUNT        4U
#define BENCH_LINE_MAX           160U

/* 四电机台架测试：一次只驱动一个轮子，正反各转 1.5 s，记录四个编码器的增量。
   300‰ 是能看清转向又不至于让架空的车跳起来的力度 */
#define MOTOR_TEST_DUTY          ((int16_t)300)
#define MOTOR_TEST_RUN_MS        1500U
#define MOTOR_TEST_BRAKE_MS      200U
#define MOTOR_TEST_REST_MS       700U
#define MOTOR_TEST_PASS_GAP_MS   5000U
#define MOTOR_TEST_STACK_WORDS   160U

/* 编码器台架测试：每 500 ms 打印一次四个轮子的增量与累计值 */
#define ENCODER_TEST_PERIOD_MS   500U
#define ENCODER_TEST_STACK_WORDS 160U
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

#if (BENCH_MOTOR_TEST_ENABLE != 0)
/* 电机测试任务的栈要放得下行缓冲（96 字节）和 HAL 发送的局部变量 */
static const osThreadAttr_t g_motor_test_task_attributes = {
  .name = "motorTestTask",
  .stack_size = MOTOR_TEST_STACK_WORDS * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
#endif

/* 编码器测试任务：四个 int32 的基准值 + 行缓冲 + HAL 发送的局部变量 */
static const osThreadAttr_t g_encoder_test_task_attributes = {
  .name = "encTestTask",
  .stack_size = ENCODER_TEST_STACK_WORDS * 4,
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
#if (BENCH_MOTOR_TEST_ENABLE != 0)
void StartMotorTestTask(void *argument);
#endif
#if (BENCH_ENCODER_TEST_ENABLE != 0)
void StartEncoderTestTask(void *argument);
#endif
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
#if (BENCH_MOTOR_TEST_ENABLE != 0)
  (void)osThreadNew(StartMotorTestTask, NULL, &g_motor_test_task_attributes);
#endif
#if (BENCH_ENCODER_TEST_ENABLE != 0)
  (void)osThreadNew(StartEncoderTestTask, NULL, &g_encoder_test_task_attributes);
#endif
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

/* ----------------------------- 台架测试公共件 ----------------------------- */

static uint16_t bench_append_str(char *buf, uint16_t pos, const char *text)
{
  while ((*text != '\0') && (pos < BENCH_LINE_MAX))
  {
    buf[pos] = *text;
    pos++;
    text++;
  }

  return pos;
}

static uint16_t bench_append_i32(char *buf, uint16_t pos, int32_t value)
{
  char     digits[11];
  uint8_t  count = 0U;
  uint32_t magnitude;

  if (value < 0)
  {
    if (pos < BENCH_LINE_MAX)
    {
      buf[pos] = '-';
      pos++;
    }
    /* 先加一再取反，INT32_MIN 也不会溢出 */
    magnitude = (uint32_t)(-(value + 1)) + 1U;
  }
  else
  {
    magnitude = (uint32_t)value;
  }

  do
  {
    digits[count] = (char)('0' + (magnitude % 10U));
    count++;
    magnitude /= 10U;
  } while ((magnitude != 0U) && (count < (uint8_t)sizeof(digits)));

  while ((count > 0U) && (pos < BENCH_LINE_MAX))
  {
    count--;
    buf[pos] = digits[count];
    pos++;
  }

  return pos;
}

static void bench_write(const char *buf, uint16_t len)
{
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, UART_TEST_TIMEOUT_MS);
}

static void bench_send(const char *text)
{
  uint16_t len = 0U;

  while ((text[len] != '\0') && (len < BENCH_LINE_MAX))
  {
    len++;
  }

  bench_write(text, len);
}

/* ----------------------------- 四电机台架测试 ----------------------------- */

#if (BENCH_MOTOR_TEST_ENABLE != 0)

/* 四个编码器都读一遍。读的动作同时把硬件计数清掉，所以每次读数都代表
   "上次读到现在"的增量 */
static void motor_test_read_all(int32_t counts[BENCH_MOTOR_COUNT])
{
  platform_motor_id_t id;

  for (id = 0u; id < BENCH_MOTOR_COUNT; id++)
  {
    counts[id] = 0;
    (void)platform_motor_get_encoder(id, &counts[id]);
  }
}

static void motor_test_phase(platform_motor_id_t id, int16_t duty)
{
  int32_t  before[BENCH_MOTOR_COUNT];
  int32_t  after[BENCH_MOTOR_COUNT];
  char     line[BENCH_LINE_MAX];
  uint16_t pos;
  uint8_t  m;

  motor_test_read_all(before);

  (void)platform_motor_set_enable(id, PLATFORM_TRUE);
  (void)platform_motor_set_duty(id, duty);
  osDelay(MOTOR_TEST_RUN_MS);

  /* 先刹车再读数：失能是滑行，轮子会再转一会儿，那部分会算进下一次测量 */
  (void)platform_motor_set_duty(id, 0);
  osDelay(MOTOR_TEST_BRAKE_MS);
  (void)platform_motor_set_enable(id, PLATFORM_FALSE);

  motor_test_read_all(after);

  pos = bench_append_str(line, 0U, "[motor] ");
  line[pos] = (char)('A' + id);
  pos++;
  pos = bench_append_str(line, pos, (duty > 0) ? " fwd  " : " rev  ");

  for (m = 0U; m < BENCH_MOTOR_COUNT; m++)
  {
    line[pos] = (char)('A' + m);
    pos++;
    line[pos] = ':';
    pos++;
    pos = bench_append_i32(line, pos, after[m] - before[m]);

    if ((m + 1U) < BENCH_MOTOR_COUNT)
    {
      line[pos] = ' ';
      pos++;
    }
  }

  line[pos] = '\r';
  pos++;
  line[pos] = '\n';
  pos++;

  bench_write(line, pos);
}

/**
  * @brief  四电机台架测试任务。
  * @note   一次只驱动一个轮子，正反各 1.5 s，每行打印四个编码器的增量。
  *         看两件事：
  *         1) 被驱动的轮子对应的那个编码器有没有数、正反号是否稳定；
  *         2) 其他三个是否为 0——不为 0 说明编码器通道接错了轮子。
  *         方向对不对（+300 是否让车往前）只能靠眼睛看，反了就把
  *         board_init.c 里那个电机的 invert_direction 改成 TRUE。
  */
void StartMotorTestTask(void *argument)
{
  platform_motor_id_t id;

  (void)argument;

  /* 等串口和调度稳定，再开始动电机 */
  osDelay(1000U);

  bench_send("\r\n[motor] bench test: duty +/-300 permille\r\n");
  bench_send("[motor] WHEELS MUST BE OFF THE GROUND\r\n");
  bench_send("[motor] line = wheel/direction, then A-D encoder deltas\r\n");

  for (;;)
  {
    for (id = 0u; id < BENCH_MOTOR_COUNT; id++)
    {
      motor_test_phase(id, MOTOR_TEST_DUTY);
      osDelay(MOTOR_TEST_REST_MS);

      motor_test_phase(id, (int16_t)(-MOTOR_TEST_DUTY));
      osDelay(MOTOR_TEST_REST_MS);
    }

    bench_send("[motor] pass done, all motors disabled\r\n");
    osDelay(MOTOR_TEST_PASS_GAP_MS);
  }
}

#endif /* BENCH_MOTOR_TEST_ENABLE */

/* ------------------------------ 编码器台架测试 ------------------------------ */

/**
  * @brief  编码器台架测试任务：每 500 ms 打印四个轮子的增量与累计值。
  * @note   手转轮子看三件事：
  *         1) 转哪个轮子，哪个字母有数——别的字母也动了说明通道接到了
  *            别的轮子上（board_init.c 里的 encoder 通道写错了）；
  *         2) 按车头方向转，增量应该是正的。B/D 的计数跟着 invert_direction
  *            一起反了号，所以四个轮子往前转都应该是正数；
  *         3) 某一路始终为 0 时，先量编码器板的供电和 A/B 相电压：TIM 的
  *            输入是 NOPULL，靠编码器板自带的上拉，没供电就是死 0。
  *         电机测试里的读数也是靠这个任务之外的调用累积的，所以两个测试
  *         不能同时开（BENCH_MOTOR_TEST_ENABLE / BENCH_ENCODER_TEST_ENABLE）。
  */
void StartEncoderTestTask(void *argument)
{
  int32_t             last[BENCH_MOTOR_COUNT];
  char                line[BENCH_LINE_MAX];
  platform_motor_id_t id;

  (void)argument;

  /* 等串口和调度稳定，顺便让编码器板完成上电自检 */
  osDelay(1000U);

  /* 第一次读只取基准：get_encoder 读的同时会把硬件计数清零，
     所以基准必须在开打印之前取，免得第一帧把上电后的转动算成增量 */
  for (id = 0u; id < BENCH_MOTOR_COUNT; id++)
  {
    last[id] = 0;
    (void)platform_motor_get_encoder(id, &last[id]);
  }

  bench_send("\r\n[enc] hand-turn test, 500 ms period\r\n");
  bench_send("[enc] wheel:delta/total, delta 0 = that wheel did not move\r\n");
  bench_send("[enc] rolling all wheels forward must give positive deltas\r\n");

  for (;;)
  {
    uint16_t pos;

    osDelay(ENCODER_TEST_PERIOD_MS);

    pos = bench_append_str(line, 0U, "[enc]");

    for (id = 0u; id < BENCH_MOTOR_COUNT; id++)
    {
      int32_t now = 0;

      (void)platform_motor_get_encoder(id, &now);

      /* 一行最多 5 + 4 * (1 + 1 + 1 + 11 + 1 + 11) = 109 字节，BENCH_LINE_MAX
         留了余量，这里直接写不会越界 */
      line[pos] = ' ';
      pos++;
      line[pos] = (char)('A' + id);
      pos++;
      line[pos] = ':';
      pos++;
      pos = bench_append_i32(line, pos, now - last[id]);
      line[pos] = '/';
      pos++;
      pos = bench_append_i32(line, pos, now);

      last[id] = now;
    }

    line[pos] = '\r';
    pos++;
    line[pos] = '\n';
    pos++;

    bench_write(line, pos);
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


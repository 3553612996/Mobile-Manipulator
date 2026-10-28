/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_os.c
 *
 * @par dependencies
 * - platform_os_task.h
 * - platform_os_time.h
 * - platform_os_critical.h
 * - FreeRTOS task API
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Implement the platform OS interface on top of FreeRTOS.
 *
 * Tasks are created with xTaskCreateStatic from a fixed pool: the project
 * forbids heap allocation after startup. The pool size is deliberately small
 * (4 tasks, 1.5 KB stack each). Adding a task means adding a pool slot or
 * raising IMPL_OS_TASK_STACK_WORDS, not calling pvPortMalloc.
 *
 * The platform priority enum maps to the CMSIS-RTOS2 numeric values that the
 * CubeMX-generated tasks already use (NORMAL = 24), so new tasks and the
 * legacy ones share one ordering.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

//******************************** Includes *********************************//

#include "platform_os_task.h"
#include "platform_os_time.h"
#include "platform_os_critical.h"

#include "FreeRTOS.h"
#include "task.h"

//******************************** Defines **********************************//

#define IMPL_OS_MAX_TASKS           (4u)
#define IMPL_OS_TASK_STACK_WORDS    (384u)  /* 1536 bytes per task. */

/* CMSIS-RTOS2 priority values, matching the CubeMX task attributes. */
#define IMPL_OS_PRIO_IDLE           (0u)
#define IMPL_OS_PRIO_LOW            (8u)
#define IMPL_OS_PRIO_NORMAL         (24u)
#define IMPL_OS_PRIO_ABOVE_NORMAL   (32u)
#define IMPL_OS_PRIO_HIGH           (40u)
#define IMPL_OS_PRIO_REALTIME       (48u)

//******************************** Declaring ********************************//

struct platform_os_task
{
    StaticTask_t                tcb;
    StackType_t                 stack[IMPL_OS_TASK_STACK_WORDS];
    TaskHandle_t                handle;
    platform_os_task_entry_t    entry;
    void *                      argument;
    platform_bool_t             used;
};

//******************************** Variables ********************************//

static struct platform_os_task s_task_pool[IMPL_OS_MAX_TASKS];

//******************************** Functions ********************************//

static UBaseType_t impl_os_priority(platform_os_priority_t priority)
{
    switch (priority)
    {
        case PLATFORM_OS_PRIORITY_REALTIME:
            return (UBaseType_t)IMPL_OS_PRIO_REALTIME;
        case PLATFORM_OS_PRIORITY_HIGH:
            return (UBaseType_t)IMPL_OS_PRIO_HIGH;
        case PLATFORM_OS_PRIORITY_ABOVE_NORMAL:
            return (UBaseType_t)IMPL_OS_PRIO_ABOVE_NORMAL;
        case PLATFORM_OS_PRIORITY_NORMAL:
            return (UBaseType_t)IMPL_OS_PRIO_NORMAL;
        case PLATFORM_OS_PRIORITY_LOW:
            return (UBaseType_t)IMPL_OS_PRIO_LOW;
        case PLATFORM_OS_PRIORITY_IDLE:
        default:
            return (UBaseType_t)IMPL_OS_PRIO_IDLE;
    }
}

static void impl_os_task_trampoline(void *argument)
{
    struct platform_os_task *task = (struct platform_os_task *)argument;

    task->entry(task->argument);

    /*
     * A platform task is not expected to return. If it does, delete it
     * instead of letting the scheduler fall through into random memory.
     */
    task->used = PLATFORM_FALSE;
    vTaskDelete(NULL);
}

platform_err_t platform_os_task_create(const platform_os_task_cfg_t *cfg,
                                       platform_os_task_t *out_task)
{
    struct platform_os_task *slot = NULL;
    uint32_t i;

    if ((cfg == NULL) || (cfg->entry == NULL) || (out_task == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    if (cfg->stack_size_bytes > (IMPL_OS_TASK_STACK_WORDS * sizeof(StackType_t)))
    {
        return PLATFORM_ERR_NO_RESOURCE;
    }

    for (i = 0u; i < IMPL_OS_MAX_TASKS; i++)
    {
        if (s_task_pool[i].used == PLATFORM_FALSE)
        {
            slot = &s_task_pool[i];
            break;
        }
    }

    if (slot == NULL)
    {
        return PLATFORM_ERR_NO_RESOURCE;
    }

    slot->entry = cfg->entry;
    slot->argument = cfg->argument;
    slot->used = PLATFORM_TRUE;

    slot->handle = xTaskCreateStatic(impl_os_task_trampoline,
                                     cfg->name,
                                     (uint32_t)IMPL_OS_TASK_STACK_WORDS,
                                     slot,
                                     impl_os_priority(cfg->priority),
                                     slot->stack,
                                     &slot->tcb);

    if (slot->handle == NULL)
    {
        slot->used = PLATFORM_FALSE;
        return PLATFORM_ERR_NO_RESOURCE;
    }

    *out_task = slot;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_os_task_delete(platform_os_task_t task)
{
    if (task == NULL)
    {
        vTaskDelete(NULL);
        return PLATFORM_ERR_OK;
    }

    if (task->used == PLATFORM_FALSE)
    {
        return PLATFORM_ERR_PARAM;
    }

    vTaskDelete(task->handle);
    task->used = PLATFORM_FALSE;

    return PLATFORM_ERR_OK;
}

platform_err_t platform_os_task_delay_ms(uint32_t delay_ms)
{
    if (delay_ms == 0u)
    {
        taskYIELD();
        return PLATFORM_ERR_OK;
    }

    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    return PLATFORM_ERR_OK;
}

platform_err_t platform_os_task_delay_until(uint32_t *last_wake_tick,
                                           uint32_t period_ms)
{
    if ((last_wake_tick == NULL) || (period_ms == 0u))
    {
        return PLATFORM_ERR_PARAM;
    }

    vTaskDelayUntil((TickType_t *)last_wake_tick, pdMS_TO_TICKS(period_ms));

    return PLATFORM_ERR_OK;
}

platform_err_t platform_os_task_notify(platform_os_task_t task)
{
    if (task == NULL)
    {
        (void)xTaskNotifyGive(xTaskGetCurrentTaskHandle());
        return PLATFORM_ERR_OK;
    }

    if (task->handle == NULL)
    {
        return PLATFORM_ERR_PARAM;
    }

    (void)xTaskNotify(task->handle, 1u, eIncrement);

    return PLATFORM_ERR_OK;
}

platform_err_t platform_os_task_notify_from_isr(platform_os_task_t task)
{
    BaseType_t woken = pdFALSE;

    if ((task == NULL) || (task->handle == NULL))
    {
        return PLATFORM_ERR_PARAM;
    }

    (void)xTaskNotifyFromISR(task->handle, 1u, eIncrement, &woken);
    portYIELD_FROM_ISR(woken);

    return PLATFORM_ERR_OK;
}

platform_err_t platform_os_task_wait_notify(uint32_t timeout_ms,
                                            uint32_t *notify_count)
{
    TickType_t timeout_ticks;
    uint32_t   count;

    timeout_ticks = (timeout_ms == PLATFORM_OS_WAIT_FOREVER)
                        ? portMAX_DELAY
                        : pdMS_TO_TICKS(timeout_ms);

    count = (uint32_t)ulTaskNotifyTake(pdTRUE, timeout_ticks);

    if (notify_count != NULL)
    {
        *notify_count = count;
    }

    return (count > 0u) ? PLATFORM_ERR_OK : PLATFORM_ERR_TIMEOUT;
}

platform_os_task_t platform_os_task_get_current(void)
{
    TaskHandle_t current = xTaskGetCurrentTaskHandle();
    uint32_t i;

    for (i = 0u; i < IMPL_OS_MAX_TASKS; i++)
    {
        if ((s_task_pool[i].used == PLATFORM_TRUE) &&
            (s_task_pool[i].handle == current))
        {
            return &s_task_pool[i];
        }
    }

    return NULL;
}

void platform_os_task_yield(void)
{
    taskYIELD();
}

uint32_t platform_os_tick_get(void)
{
    return (uint32_t)xTaskGetTickCount();
}

uint32_t platform_os_ms_to_ticks(uint32_t milliseconds)
{
    return (uint32_t)pdMS_TO_TICKS(milliseconds);
}

uint32_t platform_os_ticks_to_ms(uint32_t ticks)
{
    /* Split the division so ticks * 1000 cannot overflow uint32. */
    return (uint32_t)(((ticks / (uint32_t)configTICK_RATE_HZ) * 1000u) +
                      (((ticks % (uint32_t)configTICK_RATE_HZ) * 1000u) /
                       (uint32_t)configTICK_RATE_HZ));
}

platform_bool_t platform_os_is_in_isr(void)
{
    return (xPortIsInsideInterrupt() != pdFALSE) ? PLATFORM_TRUE
                                                 : PLATFORM_FALSE;
}

platform_err_t platform_os_kernel_init(void)
{
    /* The CubeMX startup already called osKernelInitialize(). */
    return PLATFORM_ERR_OK;
}

platform_err_t platform_os_kernel_start(void)
{
    vTaskStartScheduler();

    /* Reaching this line means the idle task could not be allocated. */
    return PLATFORM_ERR_NO_RESOURCE;
}

platform_os_critical_state_t platform_os_critical_enter(void)
{
    /*
     * BASEPRI mask, save/restore. Task context only: clearing BASEPRI inside
     * an ISR would re-enable interrupts in the middle of that ISR.
     */
    return (platform_os_critical_state_t)portSET_INTERRUPT_MASK_FROM_ISR();
}

void platform_os_critical_exit(platform_os_critical_state_t state)
{
    portCLEAR_INTERRUPT_MASK_FROM_ISR((UBaseType_t)state);
}

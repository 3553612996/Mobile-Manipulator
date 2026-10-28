/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file platform_os_queue.h
 *
 * @par dependencies
 * - platform_os_common.h
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the platform message queue interface.
 *
 * Items are copied by value. item_size_bytes is the size of one message, not
 * the total queue size. 04_Impl converts these values to the kernel's unit.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __PLATFORM_OS_QUEUE_H__
#define __PLATFORM_OS_QUEUE_H__

//******************************** Includes *********************************//

#include "platform_os_common.h"

//******************************** Declaring ********************************//

typedef struct platform_os_queue *platform_os_queue_t;

platform_err_t platform_os_queue_create(const char *name,
                                        uint32_t item_size_bytes,
                                        uint32_t item_count,
                                        platform_os_queue_t *out_queue);

/*
 * Send returns PLATFORM_ERR_TIMEOUT when the queue stays full for timeout_ms.
 * Receive returns PLATFORM_ERR_TIMEOUT when the queue stays empty.
 * The *_from_isr variants return PLATFORM_ERR_BUSY instead of waiting.
 */
platform_err_t platform_os_queue_send(platform_os_queue_t queue,
                                      const void *item,
                                      uint32_t timeout_ms);
platform_err_t platform_os_queue_send_from_isr(platform_os_queue_t queue,
                                               const void *item);
platform_err_t platform_os_queue_receive(platform_os_queue_t queue,
                                         void *item,
                                         uint32_t timeout_ms);
platform_err_t platform_os_queue_receive_from_isr(platform_os_queue_t queue,
                                                  void *item);
platform_err_t platform_os_queue_peek(platform_os_queue_t queue,
                                      void *item,
                                      uint32_t timeout_ms);

uint32_t platform_os_queue_count(platform_os_queue_t queue);
uint32_t platform_os_queue_free(platform_os_queue_t queue);
platform_err_t platform_os_queue_reset(platform_os_queue_t queue);
platform_err_t platform_os_queue_destroy(platform_os_queue_t queue);

#endif /* __PLATFORM_OS_QUEUE_H__ */

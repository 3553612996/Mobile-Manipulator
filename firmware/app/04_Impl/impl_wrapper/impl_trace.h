/******************************************************************************
 * Copyright (C) 2026 wuying. All Rights Reserved.
 *
 * @file impl_trace.h
 *
 * @par dependencies
 * - none
 *
 * @author wuying <3553612993@qq.com>
 *
 * @brief Provide the trace hooks used by the Impl wrapper layer.
 *
 * The default build compiles every hook to nothing. To enable tracing, define
 * IMPL_TRACE_ENABLE and provide impl_trace_backend.h in this directory. The
 * backend must define all four macros below and must be safe in interrupt
 * context for the *_ISR variants.
 *
 * @version V1.0 2026-10-28
 *
 * @note 1 tab == 4 spaces.
 *
 *****************************************************************************/

#ifndef __IMPL_TRACE_H__
#define __IMPL_TRACE_H__

#ifdef IMPL_TRACE_ENABLE

#include "impl_trace_backend.h"

#else

#define IMPL_TRACE_ENTER(name)              ((void)0)
#define IMPL_TRACE_EXIT(name, err)          ((void)0)

#define IMPL_TRACE_ISR_ENTER(name)          ((void)0)
#define IMPL_TRACE_ISR_EXIT(name, err)      ((void)0)

#endif /* IMPL_TRACE_ENABLE */

#endif /* __IMPL_TRACE_H__ */

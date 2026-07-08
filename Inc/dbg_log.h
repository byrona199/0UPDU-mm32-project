/**
 * @file dbg_log.h
 * @brief UART1 (PA1/PA0) debug log switch.
 *
 * Set ENABLE_DBG_LOG to 1 to enable output on UART1.
 */
#ifndef __DBG_LOG_H
#define __DBG_LOG_H

#define ENABLE_DBG_LOG 0

#if ENABLE_DBG_LOG
void dbg_log(const char *fmt, ...);
#else
#define dbg_log(...) ((void)0)
#endif

#endif /* __DBG_LOG_H */

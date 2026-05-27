/**
 * @file dbg_log.h
 * @brief UART2 debug log switch.
 *
 * Set ENABLE_DBG_LOG to 0 to compile out all dbg_log() calls
 * (saves ~256 B SRAM for the static buffer, ~28 B for the mutex CB,
 * plus per-thread stack savings since vsnprintf consumes ~300 B
 * of caller stack). Linker will also strip printfa.o (~2.3 KB Flash)
 * and all log format-string literals when no caller remains.
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

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/*
 * Minimal stubs to satisfy SEGGER RTL libc file I/O references
 * We do not use stdio streams; route all prints via hal_trace.
 */

/* Provide a dummy stdout symbol expected by fileops.o */
// Match SEGGER Embedded Studio RTL hooks to avoid pulling in file I/O.
#include <__SEGGER_RTL.h>

// Provide a dummy stdout symbol of the correct type expected by SEGGER RTL
__SEGGER_RTL_FILE* stdout = 0;

/* Provide no-op file operations required by the SEGGER runtime */
// No-op file operation hooks; returning -1 indicates unsupported operation
int __SEGGER_RTL_X_file_write(__SEGGER_RTL_FILE* __stream, const char* __s, unsigned __len) {
    (void)__stream;
    (void)__s;
    (void)__len;
    return -1;
}

int __SEGGER_RTL_X_file_stat(__SEGGER_RTL_FILE* __stream) {
    (void)__stream;
    return -1;
}

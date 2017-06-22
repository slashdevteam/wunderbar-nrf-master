#include "debug.h"

#if defined SEGGER_RTT_LOG
#include <stdarg.h>
#include "SEGGER_RTT.h"

extern int SEGGER_RTT_vprintf(unsigned BufferIndex, const char * sFormat, va_list * pParamList);

int RTT_printf(const char * sFormat, ...) {
  int r;
  va_list ParamList;

  va_start(ParamList, sFormat);
  r = SEGGER_RTT_vprintf(0, sFormat, &ParamList);
  va_end(ParamList);
  return r;
}

void debug_init(void)
{
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
}
#endif

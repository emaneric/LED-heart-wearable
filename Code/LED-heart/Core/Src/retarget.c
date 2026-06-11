#include "SEGGER_RTT.h"

int __io_putchar(int ch) {
    SEGGER_RTT_PutChar(0, (char)ch);
    return ch;
}

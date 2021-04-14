#include "util.h"

void print_string(const char *string) {
    mRs232_Uart4WriteString((Int8 *)string);
};
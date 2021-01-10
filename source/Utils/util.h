#ifndef UTIL_H_
#define UTIL_H_

#include "Modules/mRS232.h"

void print_string(char *string){
     mRs232_Uart4WriteString((Int8 *)string);
}

#endif
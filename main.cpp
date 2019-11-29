#include "NXPLogic.h"

int main(void) {
    NXPLogic nxp = NXPLogic();

    while (nxp.step());
    
    return 0;
}
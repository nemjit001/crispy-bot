#include "NXPLogic.h"

Serial terminal(USBTX, USBRX);

int main(void) {
    NXPLogic nxp = NXPLogic();

    while (nxp.step()) {}

    return 0;
}
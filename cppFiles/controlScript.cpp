#include "mbed.h"
#include <cstdlib>

/*
 * TODO:
 * based on velocity calc time needed before turning?
 * write camera function
 * write driving function
 *
 */

int main() {
	try {
		while (true) {
			/* write drive chain */
		}
	}
	catch (std::runtime_error& excpt) {
		std::cout << "Runtime Error: " << excpt.what() << std::endl;
	}
    return 0;
}

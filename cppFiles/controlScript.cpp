#include "mbed.h"
#include <cstdlib>

int main() {
	try {
	// write some code
	}
	catch (std::runtime_error& excpt) {
		std::cout << "Runtime Error: " << excpt.what() << std::endl;
	}
    return 0;
}

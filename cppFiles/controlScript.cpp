#include <iostream>
#include <cstdlib>
#include "physcom.h"

int main () {
	try {
		// write code mah dudes
	}
	catch (std::runtime_error& excpt) {
		std::cout << excpt.what();
	}
	return 0;
}

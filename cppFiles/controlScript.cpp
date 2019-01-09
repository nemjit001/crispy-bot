#include <iostream>
#include <cstdlib>

int main () {
	try {
		// write code mah dudes
	}
	catch (std::runtime_error& excpt) {
		std::cout << excpt.what();
	}
	return 0;
}

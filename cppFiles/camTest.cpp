#include "mbed.h"

int main () {
	vector<vector<int>> cam(512, (512, 0));
	for (int i = 0; i < 100) {
		for (int j = 0; j < cam.size(); ++j) {
			if (j == horiRes) {
				std::cout << cam.at(j) << std::endl;
			}
			else {
				std::cout << cam.at(j);
			}
		}
	}
	return 0;
}

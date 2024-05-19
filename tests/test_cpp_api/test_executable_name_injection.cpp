#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
	std::filesystem::path path{ CURAN_BINARY_LOCATION "/mimic_child_proc" CURAN_BINARY_SUFFIX };
	if (std::filesystem::exists(path))
		std::cout << " exists\n";
	else
		std::cout << " does not exist\n";
	return 0;
}
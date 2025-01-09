#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files


#ifdef _WIN32
#include <windows.h>
#endif
#include <asio.hpp>
#include <variant>
#include <utility>
#include <map>
#include <optional>


int main() {
	unsigned short port = 50000;
	asio::io_context io_context;
	asio::ip::tcp::socket socket{io_context};
	return 0;
}


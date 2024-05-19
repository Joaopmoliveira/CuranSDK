#include <string>
#include <iostream>
#include <vector>
#include <array>
#include <cassert>
#include <charconv>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string_view>
#include <system_error>

template<typename... Args>
void received(Args ... arg) {
    constexpr size_t size = sizeof ...(Args);
    const char* loc[size] ={ arg...};
    std::vector<std::string> arguments;
    for(const auto& val : loc) {
        arguments.emplace_back(val);
    }
    std::vector<char*> vector_of_strings;

    for (auto& s : arguments) {
        std::cout << s << std::endl;
        vector_of_strings.push_back(s.data());
    }
    vector_of_strings.push_back(NULL);
}

void received(int n_arguments,char* arguments[]) {
    if (n_arguments <= 0)
        throw std::runtime_error("number of arguments is incorrect");

    if (n_arguments == 1)
        return;

    if (n_arguments == 2) {
        int port{};
        std::string str{ arguments[1]};
        auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), port);

        if (ec != std::errc())
            throw std::runtime_error("failure to read the port from the command line");             
    }

    if (n_arguments > 2) {
        throw std::runtime_error("too many input arguments");
    }
    
}

int main() {
	received("My name is:", "The master of all trades", "and the jack of none");
	return 0;
}
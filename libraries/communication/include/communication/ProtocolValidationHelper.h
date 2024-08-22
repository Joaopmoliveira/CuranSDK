#ifndef PROCOTOL_VALIDATION_HELPER_HEADER
#define PROCOTOL_VALIDATION_HELPER_HEADER

#include <functional>
#include <type_traits>


namespace curan {
namespace communication {

template<typename, typename = void>
constexpr bool is_type_complete_v = false;

template<typename T>
constexpr bool is_type_complete_v
    <T, std::void_t<decltype(sizeof(T))>> = true;

}
}

#endif
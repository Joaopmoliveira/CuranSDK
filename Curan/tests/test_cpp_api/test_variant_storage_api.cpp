#include <variant>
#include <functional>

struct A{

};

struct B {

};

struct C {

};

using D = std::variant<A,B,C>;

template<typename S>
struct H {
using callback_A = std::function<void(const S&,A)>;
using callback_B = std::function<void(const S&,B)>;
using callback_C = std::function<void(const S&,C)>;

std::list<callback_A> callbacks_A;
std::list<callback_B> callbacks_B;
std::list<callback_C> callbacks_C;

void add_A_callback();
void add_B_callback();
void add_C_callback();

};

struct H1 : H<H1> {

};

struct H2:  H<H2> {

};

struct H3:  H<H3> {

};

struct H4:  H<H4> {

};
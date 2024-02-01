#include <variant>
#include <functional>
#include <iostream>
#include <list>
#include <list>
#include <memory>

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

struct A{
    int a;
};

struct B {
    int b;
};

struct C {
    int c;
};

using D = std::variant<A,B,C>;

template<typename S>
struct H {
using callback_A = std::function<void(S*,A)>;
using callback_B = std::function<void(S*,B)>;
using callback_C = std::function<void(S*,C)>;

std::list<callback_A> callbacks_A;
std::list<callback_B> callbacks_B;
std::list<callback_C> callbacks_C;

void add_A_callback(callback_A&& call){
    callbacks_A.emplace_back(std::move(call));
}
void add_B_callback(callback_B&& call){
    callbacks_B.emplace_back(std::move(call));
}
void add_C_callback(callback_C&& call){
    callbacks_C.emplace_back(std::move(call));
}

};

struct H1 : H<H1> {
    void printtype(){
        std::printf("Type: H1\n");
    }

    void call(D d){
        std::visit(overloaded{
            [&](A a){
                for(const auto& localcall : callbacks_A)
                    localcall(this,a);
            },
            [&](B b){
                for(const auto& localcall : callbacks_B)
                    localcall(this,b);
            },
            [&](C c){
                for(const auto& localcall : callbacks_C)
                    localcall(this,c);
            },
        },
        d
        );
    }
};

struct H2:  H<H2> {
    void printtype(){
        std::printf("Type: H2\n");
    }

    void call(D d){
        std::visit(overloaded{
            [&](A a){
                for(const auto& localcall : callbacks_A)
                    localcall(this,a);
            },
            [&](B b){
                for(const auto& localcall : callbacks_B)
                    localcall(this,b);
            },
            [&](C c){
                for(const auto& localcall : callbacks_C)
                    localcall(this,c);
            },
        },
        d
        );
    }
};

struct H3:  H<H3> {
    void printtype(){
        std::printf("Type: H3\n");
    }

    void call(D d){
        std::visit(overloaded{
            [&](A a){
                for(const auto& localcall : callbacks_A)
                    localcall(this,a);
            },
            [&](B b){
                for(const auto& localcall : callbacks_B)
                    localcall(this,b);
            },
            [&](C c){
                for(const auto& localcall : callbacks_C)
                    localcall(this,c);
            },
        },
        d
        );
    }
};

struct H4:  H<H4> {
    void printtype(){
        std::printf("Type: H4\n");
    }

    void call(D d){
        std::visit(overloaded{
            [&](A a){
                for(const auto& localcall : callbacks_A)
                    localcall(this,a);
            },
            [&](B b){
                for(const auto& localcall : callbacks_B)
                    localcall(this,b);
            },
            [&](C c){
                for(const auto& localcall : callbacks_C)
                    localcall(this,c);
            },
        },
        d
        );
    }
};

int function123(){
    // first we will try to allocate each signal with distinct values
    A a{10};
    B b{20};
    C c{30};

    H1 h1;
    h1.add_A_callback([](H1* widget,A a){widget->printtype();std::printf("the A callback is called with value %d\n",a.a);});
    h1.add_A_callback([](H1* widget,A a){widget->printtype();std::printf("the A second callback is called with value %d\n",a.a);});
    h1.add_B_callback([](H1* widget,B a){widget->printtype();std::printf("the B callback is called with value %d\n",a.b);});
    h1.add_C_callback([](H1* widget,C a){widget->printtype();std::printf("the C callback is called with value %d\n",a.c);});
    h1.add_C_callback([](H1* widget,C a){widget->printtype();std::printf("the C second callback is called with value %d\n",a.c);});

    H2 h2;
    h2.add_A_callback([](H2* widget,A a){widget->printtype();std::printf("the A callback is called with value %d\n",a.a);});
    h2.add_A_callback([](H2* widget,A a){widget->printtype();std::printf("the A second callback is called with value %d\n",a.a);});
    h2.add_B_callback([](H2* widget,B a){widget->printtype();std::printf("the B callback is called with value %d\n",a.b);});
    h2.add_C_callback([](H2* widget,C a){widget->printtype();std::printf("the C callback is called with value %d\n",a.c);});
    h2.add_C_callback([](H2* widget,C a){widget->printtype();std::printf("the C second callback is called with value %d\n",a.c);});

    H3 h3;
    h3.add_A_callback([](H3* widget,A a){widget->printtype();std::printf("the A callback is called with value %d\n",a.a);});
    h3.add_A_callback([](H3* widget,A a){widget->printtype();std::printf("the A second callback is called with value %d\n",a.a);});
    h3.add_B_callback([](H3* widget,B a){widget->printtype();std::printf("the B callback is called with value %d\n",a.b);});
    h3.add_C_callback([](H3* widget,C a){widget->printtype();std::printf("the C callback is called with value %d\n",a.c);});
    h3.add_C_callback([](H3* widget,C a){widget->printtype();std::printf("the C second callback is called with value %d\n",a.c);});

    H4 h4;
    h4.add_A_callback([](H4* widget,A a){widget->printtype();std::printf("the A callback is called with value %d\n",a.a);});
    h4.add_A_callback([](H4* widget,A a){widget->printtype();std::printf("the A second callback is called with value %d\n",a.a);});
    h4.add_B_callback([](H4* widget,B a){widget->printtype();std::printf("the B callback is called with value %d\n",a.b);});
    h4.add_C_callback([](H4* widget,C a){widget->printtype();std::printf("the C callback is called with value %d\n",a.c);});
    h4.add_C_callback([](H4* widget,C a){widget->printtype();std::printf("the C second callback is called with value %d\n",a.c);});

    h1.call(a);
    h1.call(b);
    h1.call(c);

    h2.call(a);
    h2.call(b);
    h2.call(c);

    h3.call(a);
    h3.call(b);
    h3.call(c);

    h4.call(a);
    h4.call(b);
    h4.call(c);
    return 0;
}

int main(){
    std::list<std::unique_ptr<double>> values;
    for(int i = 0; i< 10; ++i)
        values.emplace_back(std::make_unique<double>((double)i));
    
    std::printf("values are:\n");
    for(const auto& val : values)
        std::printf(" %f ",*val);
    std::printf("\n");

    values.remove_if([](std::unique_ptr<double> & val){return *val == 1.0;});

    std::printf("modified values are:\n");
    for(const auto& val : values)
        std::printf(" %f ",*val);
    std::printf("\n");
    return 0;
}
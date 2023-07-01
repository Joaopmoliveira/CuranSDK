#include <variant>
#include <vector>
#include <iostream>
#include "utils/Overloading.h"

struct A{
    void foo(){
        std::printf("printing foo\n");
    }
};

struct B : public A{
    int custom_val = 100;
    int size_ = 10;

    B(){};
    B(B& other) = delete;
    B(B&& other) : custom_val{other.custom_val}, size_{other.size_}{}

    void bar(){
        std::printf("printing bar from B\n");
    };

    B& update(int val){
        custom_val = val;
        return *(this);
    };

    B& size(int insize){
        size_ = insize;
        return *(this);
    };
};

struct C : public A{
    int custom_val = 100;
    int size_ = 10;

    C(){};
    C(C& other) = delete;
    C(C&& other) : custom_val{other.custom_val}, size_{other.size_}{}

    void bar(){
        std::printf("printing bar from C\n");
    }

    C& update(int val){
        custom_val = val;
        return *(this);
    }

    C& size(int insize){
        size_ = insize;
        return *(this);
    }
};

using widget = std::variant<B,C>;

struct D{
    std::vector<widget> widgetcontainer;
    std::vector<double> subdividions;
    bool subdivisions_external = false;

    D& operator<<(widget&& wid){
        if(subdivisions_external)
            throw std::runtime_error("you cannot add a widget to the container after specifiying the custom subdivisions");
        widgetcontainer.emplace_back(std::move(wid));
        return *(this);
    }

    D& divisions(std::initializer_list<double> init){
    subdivisions_external = true;
    if(widgetcontainer.size() != init.size())
        throw std::runtime_error("the supplied size of the divisions between widgets is not of a correct size");
    }
};

int main(){
    B valb;
    valb.update(111).size(10);
    C valc;
    valc.update(222).size(20);
    D container;
    container << std::move(valb) << std::move(valc);
    for(auto& v : container.widgetcontainer)
        std::visit(curan::utilities::overloaded{
			[](B& arg) {
                arg.foo();
                arg.bar();
			},
            [](C& arg) {
                arg.foo();
                arg.bar();
			}},v);
    return 0;
}
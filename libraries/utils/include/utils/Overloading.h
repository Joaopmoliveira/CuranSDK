#ifndef CURAN_OVERLOADING_HEADER_FILE_
#define CURAN_OVERLOADING_HEADER_FILE_

namespace curan {
namespace utilities {

/*
Visit the page https://en.cppreference.com/w/cpp/utility/variant/visit for further information
We have variants, which are a safe version of unions from c, but to switch between the type 
contained in the variant we need the overloaded operator, which is only defined in the cpp standard of 2020

A typical use case of this class is 

using Contained = std::variant<int,double,float,std::string>;

void print(Contained cont){
	std::visit(utilities::overloaded{
		[](int arg) { std::cout "contains int with value: " << arg << std::endl; },
		[](double arg) { std::cout "contains double with value: " << arg << std::endl; },
		[](float arg) { std::cout "contains float with value: " << arg << std::endl; },
		[](std::string arg) { std::cout "contains string with value: " << arg << std::endl; },
}

int main(){
	Contained union_of_values = 0.0; //underlying type is double
	print(union_of_values);
	union_of_values = 4.0f; // underlying type is float
	print(union_of_values);
	union_of_values = "str"; // underlying type is string
	print(union_of_values);
	union_of_values = 4; // underlying type is int
	print(union_of_values);
	return 0;
}

which prints the following 

contains double with value: 0.0 
contains float with value: 4.0
contains string with value: str
contains int with value: 4
*/

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

}
}

#endif
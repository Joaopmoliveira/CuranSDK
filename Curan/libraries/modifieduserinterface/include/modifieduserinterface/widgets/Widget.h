#ifndef CURAN_WIDGET_HEADER_FILE_
#define CURAN_WIDGET_HEADER_FILE_

#include <variant>

namespace curan{
namespace ui{

class Container;
class Button;
class Slider;

using Widget = std::variant<std::unique_ptr<Button>,std::unique_ptr<Container>,std::unique_ptr<Slider>>; 

} 
}

#endif
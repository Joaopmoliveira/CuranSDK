#ifndef CURAN_CONFIG_DRAW_HEADER_FILE_
#define CURAN_CONFIG_DRAW_HEADER_FILE_

namespace curan {
namespace ui {

class Page;

/*
The ConfigDraw object currently contains 
only a pointer to the page to which all 
the widgets belong to. 

Whenever callbacks are called, a pointer to this object is 
passed around. In the future, we can add 
more entries to this object,
without breaking any code, whilst at the 
same time having the flexibility of adding 
stuff without concerns.
*/

struct ConfigDraw {
	Page* stack_page;
	ConfigDraw(Page* in_stack_page) : stack_page{ in_stack_page } {};
	ConfigDraw() : stack_page{ nullptr } {};
};

}
}
#endif
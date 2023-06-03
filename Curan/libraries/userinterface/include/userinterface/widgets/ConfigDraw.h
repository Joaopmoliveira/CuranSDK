#ifndef CURAN_CONFIG_DRAW_HEADER_FILE_
#define CURAN_CONFIG_DRAW_HEADER_FILE_

#include "Page.h"

namespace curan {
	namespace ui {
		struct ConfigDraw {
			Page* stack_page;
			ConfigDraw(Page* in_stack_page) : stack_page{ in_stack_page } {};
			ConfigDraw() : stack_page{ nullptr } {};
		};
	}
}
#endif
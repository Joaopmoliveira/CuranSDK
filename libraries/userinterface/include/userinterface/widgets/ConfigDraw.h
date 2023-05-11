#ifndef CURAN_CONFIG_DRAW_HEADER_FILE_
#define CURAN_CONFIG_DRAW_HEADER_FILE_

#include <stack>
#include "Page.h"

namespace curan {
	namespace ui {
		struct ConfigDraw {
			std::stack<std::shared_ptr<Page>> stack_page;
		};
	}
}
#endif
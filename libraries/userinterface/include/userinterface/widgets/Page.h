#ifndef CURAN_PAGE_HEADER_FILE_
#define CURAN_PAGE_HEADER_FILE_

#include <list>
#include <any>

namespace curan {
	namespace ui {
		class Page {
			std::list<std::any> contained_widgets;
		};
	}
}

#endif
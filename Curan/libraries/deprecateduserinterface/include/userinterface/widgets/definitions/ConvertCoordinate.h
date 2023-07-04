#ifndef CURAN_BUTTON_HEADER_FILE_
#define CURAN_BUTTON_HEADER_FILE_

#include "UIdefinitions.h"

namespace curan {
	namespace ui {
		void convertScreenToSkia(SkCanvas* canvas, GLFWwindow* window, double& x, double& y);
	}
}

#endif
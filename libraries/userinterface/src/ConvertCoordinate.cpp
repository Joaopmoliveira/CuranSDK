#include "userinterface/widgets/definitions/ConvertCoordinate.h"

namespace curan {
	namespace ui {
		void convertScreenToSkia(SkCanvas* canvas, GLFWwindow* window, double& x, double& y) {
			int lx = -1;
			int ly = -1;
			glfwGetWindowSize(window, &lx, &ly);
			canvas->getSurface()->width();
			canvas->getSurface()->height();
			double xtmp = x * ((canvas->getSurface()->width()) / ((double)lx));
			double ytmp = y * ((canvas->getSurface()->height()) / ((double)ly));
		}
	}
}
#include "userinterface/BoxPose.h"

namespace curan {
	namespace ui {
		std::shared_ptr<BoxPose> BoxPose::make_shared_box(int origin_x, int origin_y, int width, int height) {
			return std::shared_ptr<BoxPose>(new BoxPose{ origin_x ,origin_y,width ,height });
		}
	}
}
#ifndef CURAN_BOXPOSE_HEADER_FILE_
#define CURAN_BOXPOSE_HEADER_FILE_

#include <memory>

namespace curan {
	namespace ui {

		struct BoxPose {
			int origin_x;
			int origin_y;
			int width;
			int height;

			static std::shared_ptr<BoxPose> make_shared_box(int origin_x, int origin_y, int width, int height);
		private:
			BoxPose(int origin_x, int origin_y, int width, int height) : origin_x{ origin_x }, origin_y{ origin_y }, width{ width }, height{ height } {};
		};
	}
}

#endif
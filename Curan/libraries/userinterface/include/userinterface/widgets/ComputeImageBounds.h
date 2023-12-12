#ifndef CURAN_COMPUTE_IMAGE_BOUNDS_HEADER_FILE_
#define CURAN_COMPUTE_IMAGE_BOUNDS_HEADER_FILE_

#include "definitions/UIdefinitions.h"

namespace curan {
	namespace ui {
        inline SkRect compute_bounded_rectangle(SkRect position,float image_width, float image_height){
            float current_selected_width = position.width();
	        float current_selected_height = position.height();
	        float scale_factor = std::min(current_selected_width * 0.9f / image_width, current_selected_height * 0.95f / image_height);
	        float init_x = (current_selected_width - image_width * scale_factor) / 2.0f + position.x();
	        float init_y = (current_selected_height - image_height * scale_factor) / 2.0f + position.y();
            return SkRect::MakeXYWH(init_x, init_y, scale_factor * image_width, scale_factor * image_height);
        }

    }
}


#endif
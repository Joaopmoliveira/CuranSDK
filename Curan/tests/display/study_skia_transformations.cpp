#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include <thread>

int main(){
    SkRect src = SkRect::MakeXYWH(0,0,1000,800);
    SkRect dst = SkRect::MakeXYWH(300,300,400,400);
    SkMatrix matrix;
    matrix.setAll(-1, -1, -1, -1, -1, -1, -1, -1, -1);
    if(!matrix.setRectToRect(src, dst, SkMatrix::kFill_ScaleToFit)){
        return 1;
    }
    std::cout << "\n";
    matrix.dump();
    std::cout << "\n";

    SkMatrix inverse_mat;
    SkVector vec_other_way;
    if(matrix.invert(&inverse_mat)){
        vec_other_way = inverse_mat.mapPoint(SkPoint::Make(300,300));
        std::printf("vector is: %f %f\n",vec_other_way.fX,vec_other_way.fY);
        vec_other_way = matrix.mapPoint(SkPoint::Make(50,50));
        std::printf("vector is: %f %f\n",vec_other_way.fX,vec_other_way.fY);
    }

    

    return 0;
}

/*
int notmain() {
	try {
		using namespace curan::ui;
		IconResources resources{ CURAN_COPIED_RESOURCE_PATH"/images" };
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),1200,800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		SkColor colbuton = { SK_ColorRED };

		SkPaint paint_square;
		paint_square.setStyle(SkPaint::kStroke_Style);
		paint_square.setAntiAlias(true);
		paint_square.setStrokeWidth(4);
		paint_square.setColor(colbuton);

        SkMatrix mat;
        mat.Translate(SkVector::Make(200.0,200.0));
        mat.Scale(1.0/400.0,1.0/400.0);

        SkVector vec = mat.mapVector(300,300);

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			canvas->drawColor(SK_ColorWHITE);

            canvas->drawRect(SkRect::MakeXYWH(200,200,300,300),paint_square);

			SkPoint point{ 250,250 };
			canvas->drawCircle(point,10.0, paint_square);
            


			glfwPollEvents();
			auto signals = viewer->process_pending_signals();
			

			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		return 0;
	}
	catch (...) {
		std::cout << "Failed";
		return 1;
	}
}
*/
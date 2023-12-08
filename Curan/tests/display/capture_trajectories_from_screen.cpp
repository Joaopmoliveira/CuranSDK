#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "utils/Overloading.h"
#include "userinterface/widgets/Drawable.h"
#include "utils/Lockable.h"
#include "userinterface/widgets/SignalProcessor.h"
#include <iostream>
#include <thread>

constexpr int default_window_size = 100;

struct ZoomIn{
	bool is_active = false;
	SkPoint current_active_point;
	SkRect window_size;
	SkSamplingOptions options;
	SkPaint rectangle_paint;
	double scale_factor = 2.0;

	SkPoint get_coordinates(){
		return current_active_point;
	}

	ZoomIn(){
		options = SkSamplingOptions(SkFilterMode::kLinear,SkMipmapMode::kLinear);
		rectangle_paint.setStyle(SkPaint::kStroke_Style);
		rectangle_paint.setAntiAlias(true);
		rectangle_paint.setStrokeWidth(3);
		rectangle_paint.setColor(SK_ColorGREEN);
	}

	operator bool(){
		return is_active;
	};

	void operator ()(){
		 is_active = false;
	};	

	void activate(){
		is_active = true;
	}

	void deactivate(){
		is_active = false;
	}

	void store_position(SkPoint new_active_point,SkRect new_window_size){
		current_active_point = new_active_point;
		window_size = new_window_size;
	}

	SkIRect get_interger_region(){
		auto left = current_active_point.x()-default_window_size;
		if(left<0)
			left = 0;
		auto top = current_active_point.y()-default_window_size;
		if(top<0)
			top = 0;
		auto right = default_window_size*2+left;
		if(right>window_size.fRight)
			right = window_size.fRight;
		auto bottom = default_window_size*2+top;
		if(bottom>window_size.fBottom)
			bottom = window_size.fBottom;
		return SkIRect::MakeLTRB(left,top,right,bottom);
	}

	SkRect get_float_region(){
		auto left = current_active_point.x()-default_window_size;
		if(left<0)
			left = 0;
		auto top = current_active_point.y()-default_window_size;
		if(top<0)
			top = 0;
		auto right = default_window_size*2+left;
		if(right>window_size.fRight)
			right = window_size.fRight;
		auto bottom = default_window_size*2+top;
		if(bottom>window_size.fBottom)
			bottom = window_size.fBottom;
		return SkRect::MakeLTRB(left,top,right,bottom);
	}

	void draw(SkCanvas* canvas){
		if(canvas){
			auto region = get_interger_region();
			auto float_region = get_float_region();
			auto image = canvas->getSurface()->makeImageSnapshot(region);
			SkAutoCanvasRestore restore{canvas,true};
			canvas->scale(scale_factor,scale_factor);
			canvas->drawRect(SkRect::MakeXYWH(1.0/scale_factor*(float_region.fLeft-default_window_size),1.0/scale_factor*(float_region.fTop-default_window_size),float_region.width(),float_region.height()),rectangle_paint);
			canvas->drawImage(image,1.0/scale_factor*(float_region.fLeft-default_window_size),1.0/scale_factor*(float_region.fTop-default_window_size), options);
			
		}
	}
};

struct Stroke{
	SkPath rendered_path;
	std::vector<SkPoint> recorded_points;

	Stroke(std::vector<SkPoint> in_recorded_points){
		if(in_recorded_points.size()==0){
			return;
		}
		recorded_points = in_recorded_points;
		rendered_path.moveTo(recorded_points.front());
		for(const auto & point : recorded_points){
			rendered_path.lineTo(point);
		}
	}

	double distance(SkPoint point){
		double minimum = std::numeric_limits<double>::max();
		for(const auto& r : recorded_points){
			float local = (r-point).distanceToOrigin();
			if(minimum>local)
				minimum = local;
		}
		return minimum;
	}

};

class Panel : public curan::ui::Drawable, public curan::utilities::Lockable, public curan::ui::SignalProcessor<Panel>
{
private:
	SkColor hover_color;
	SkColor waiting_color;
	SkPaint paint;
	bool compiled = false;
	std::list<Stroke> strokes;
	bool magnifying_glass = false;

	Panel(){

	}

public:
	static std::unique_ptr<Panel> make(){

	}

	void compile() override{

	}

	~Panel(){

	}

	curan::ui::drawablefunction draw() override{

	}

	curan::ui::callablefunction call() override{

	}


};


int main()
{
	try
	{
		using namespace curan::ui;
		IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();

		DisplayParams param{std::move(context), 2200, 1800};
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
		viewer->get_size();

		SkColor colbuton = {SK_ColorRED};

		SkPaint paint_square;
		paint_square.setStyle(SkPaint::kFill_Style);
		paint_square.setAntiAlias(true);
		paint_square.setStrokeWidth(4);
		paint_square.setColor(SK_ColorRED);

		SkPaint paint_stroke;
		paint_stroke.setStyle(SkPaint::kStroke_Style);
		paint_stroke.setAntiAlias(true);
		paint_stroke.setStrokeWidth(8);
		paint_stroke.setColor(SK_ColorBLUE);

		std::list<Stroke> strokes;
		std::vector<SkPoint> current_stroke;
		current_stroke.reserve(1000);

		bool is_pressed = false;
		bool is_highlighting = false;
		ZoomIn zoom_in;

		while (!glfwWindowShouldClose(viewer->window))
		{
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface *pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas *canvas = pointer_to_surface->getCanvas();
			canvas->drawColor(SK_ColorWHITE);

			canvas->drawPoints(SkCanvas::PointMode::kPoints_PointMode,current_stroke.size(),current_stroke.data(),paint_square);


			if(is_highlighting){
					double minimum = std::numeric_limits<double>::max();
					auto minimum_index = strokes.end();
					for(auto begin = strokes.begin(); begin != strokes.end(); ++begin){
						double local = begin->distance(zoom_in.get_coordinates());
						if(minimum>local){
							minimum = local;
							minimum_index = begin;
						}
						canvas->drawPath((*begin).rendered_path,paint_stroke);						
					}
					if(minimum_index!=strokes.end() && minimum< 20.0f){
						paint_stroke.setStrokeWidth(12);
						paint_stroke.setColor(SK_ColorDKGRAY);
						canvas->drawPath((*minimum_index).rendered_path,paint_stroke);
						paint_stroke.setStrokeWidth(8);
						paint_stroke.setColor(SK_ColorBLUE);
					}

				} else {
					for(auto begin = strokes.begin(); begin != strokes.end(); ++begin){
						canvas->drawPath((*begin).rendered_path,paint_stroke);						
					}
				}

			if(zoom_in){
				zoom_in.draw(canvas);
			}

			glfwPollEvents();
			auto signals = viewer->process_pending_signals();

			for (const auto &sig : signals)
			{
				std::visit(curan::utilities::overloaded{
					[](Empty arg) {

					},
					[&](Move arg) {
						if(!is_highlighting){
							if(is_pressed){
								current_stroke.push_back(SkPoint::Make((float)arg.xpos,(float)arg.ypos));
							} else if(!current_stroke.empty()){
								strokes.emplace_back(current_stroke);
								current_stroke.clear();
							}
						}
						zoom_in.store_position(SkPoint::Make((float)arg.xpos,(float)arg.ypos),viewer->get_size());
					},
					[&](Press arg) {
						is_pressed = true;
					},
					[](Scroll arg){							
					},
					[&](Unpress arg) {
						is_pressed = false;
						if(!current_stroke.empty()){
							strokes.emplace_back(current_stroke);
							current_stroke.clear();
						}
					},
					[&](Key arg) {
						if (arg.key == GLFW_KEY_A && arg.action == GLFW_PRESS){
							if(zoom_in)
								zoom_in.deactivate();
							else
								zoom_in.activate();
							return;
						}
						if (arg.key == GLFW_KEY_S && arg.action == GLFW_PRESS){
							is_highlighting = !is_highlighting;
							if(!current_stroke.empty()){
								strokes.emplace_back(current_stroke);
								current_stroke.clear();
							}
							return;
						}
					},
					[](ItemDropped arg){

					}},
					sig);
			}
			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		return 0;
	}
	catch (...)
	{
		std::cout << "Failed";
		return 1;
	}
}
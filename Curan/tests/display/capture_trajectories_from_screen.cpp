#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "utils/Overloading.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Drawable.h"
#include "userinterface/widgets/Overlay.h"
#include "utils/Lockable.h"
#include "userinterface/widgets/SignalProcessor.h"
#include <iostream>
#include <thread>
#include <unordered_map>

constexpr int default_window_size = 50;
constexpr int default_padding = 50;

struct ZoomIn
{
	bool is_active = false;
	SkPoint current_active_point;
	SkRect window_size;
	SkSamplingOptions options;
	SkPaint rectangle_paint;
	double scale_factor = 2.0;

	SkPoint get_coordinates()
	{
		return current_active_point;
	}

	ZoomIn()
	{
		options = SkSamplingOptions(SkFilterMode::kLinear, SkMipmapMode::kLinear);
		rectangle_paint.setStyle(SkPaint::kStroke_Style);
		rectangle_paint.setAntiAlias(true);
		rectangle_paint.setStrokeWidth(3);
		rectangle_paint.setColor(SK_ColorGREEN);
	}

	operator bool()
	{
		return is_active;
	};

	void operator()()
	{
		is_active = false;
	};

	void activate()
	{
		is_active = true;
	}

	void deactivate()
	{
		is_active = false;
	}

	void store_position(SkPoint new_active_point, SkRect new_window_size)
	{
		current_active_point = new_active_point;
		window_size = new_window_size;
	}

	SkIRect get_interger_region()
	{
		auto left = current_active_point.x() - default_window_size;
		if (left < 0)
			left = 0;
		auto top = current_active_point.y() - default_window_size;
		if (top < 0)
			top = 0;
		auto right = default_window_size * 2 + left;
		if (right > window_size.fRight)
			right = window_size.fRight;
		auto bottom = default_window_size * 2 + top;
		if (bottom > window_size.fBottom)
			bottom = window_size.fBottom;
		return SkIRect::MakeLTRB(left, top, right, bottom);
	}

	SkRect get_float_region()
	{
		auto left = current_active_point.x() - default_window_size;
		if (left < 0)
			left = 0;
		auto top = current_active_point.y() - default_window_size;
		if (top < 0)
			top = 0;
		auto right = default_window_size * 2 + left;
		if (right > window_size.fRight)
			right = window_size.fRight;
		auto bottom = default_window_size * 2 + top;
		if (bottom > window_size.fBottom)
			bottom = window_size.fBottom;
		return SkRect::MakeLTRB(left, top, right, bottom);
	}

	void draw(SkCanvas *canvas)
	{
		if (canvas)
		{
			auto region = get_interger_region();
			auto float_region = get_float_region();
			auto image = canvas->getSurface()->makeImageSnapshot(region);
			SkAutoCanvasRestore restore{canvas, true};
			canvas->scale(scale_factor, scale_factor);
			canvas->drawRect(SkRect::MakeXYWH(1.0 / scale_factor * (float_region.fLeft - default_window_size), 1.0 / scale_factor * (float_region.fTop - default_window_size), float_region.width(), float_region.height()), rectangle_paint);
			canvas->drawImage(image, 1.0 / scale_factor * (float_region.fLeft - default_window_size), 1.0 / scale_factor * (float_region.fTop - default_window_size), options);
		}
	}
};

struct Stroke
{
	SkPath rendered_path;
	std::vector<SkPoint> recorded_points;

	Stroke(std::vector<SkPoint> in_recorded_points)
	{
		if (in_recorded_points.size() == 0)
		{
			return;
		}
		recorded_points = in_recorded_points;
		rendered_path.moveTo(recorded_points.front());
		for (const auto &point : recorded_points)
		{
			rendered_path.lineTo(point);
		}
	}

	double distance(SkPoint point)
	{
		double minimum = std::numeric_limits<double>::max();
		for (const auto &r : recorded_points)
		{
			float local = (r - point).distanceToOrigin();
			if (minimum > local)
				minimum = local;
		}
		return minimum;
	}
};

struct Point {

};

class Panel : public curan::ui::Drawable, public curan::utilities::Lockable, public curan::ui::SignalProcessor<Panel>
{
private:

	size_t counter = 0;

	SkColor colbuton = {SK_ColorRED};
	SkPaint paint_square;
	SkPaint bluring_paint;
	SkPaint paint_stroke;
	SkPaint background_paint;
	std::unordered_map<size_t,Stroke> strokes;
	std::vector<SkPoint> current_stroke;
	SkMatrix panel_matrix_transform;
	curan::ui::IconResources& system_icons;
	SkFont text_font;

	std::array<float, 3> color_phase_offset;

	bool is_pressed = false;
	bool is_highlighting = false;
	ZoomIn zoom_in;
	std::unique_ptr<curan::ui::LightWeightPage> options_overlay;

	SkSamplingOptions options;
	sk_sp<SkImageFilter> imgfilter;

	void insert_in_map(const std::vector<SkPoint>& future_stroke){
		strokes.try_emplace(counter,future_stroke);
		++counter;
	}

	Panel(curan::ui::IconResources& other) : system_icons{other} 
	{		
		paint_square.setStyle(SkPaint::kFill_Style);
		paint_square.setAntiAlias(true);
		paint_square.setStrokeWidth(4);
		paint_square.setColor(SK_ColorRED);

		background_paint.setStyle(SkPaint::kFill_Style);
		background_paint.setAntiAlias(true);
		background_paint.setStrokeWidth(4);
		background_paint.setColor(SK_ColorBLACK);

		paint_stroke.setStyle(SkPaint::kStroke_Style);
		paint_stroke.setAntiAlias(true);
		paint_stroke.setStrokeWidth(8);
		paint_stroke.setColor(SK_ColorGRAY);

		current_stroke.reserve(1000);

		imgfilter = SkImageFilters::Blur(10, 10, nullptr);
		bluring_paint.setImageFilter(imgfilter);
		options = SkSamplingOptions();

		const char* fontFamily = nullptr;
		SkFontStyle fontStyle;
		sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
		auto typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

		text_font = SkFont(typeface, 20, 1.0f, 0.0f);
		text_font.setEdging(SkFont::Edging::kAntiAlias);

	}

public:
	static std::unique_ptr<Panel> make(curan::ui::IconResources& other){
		std::unique_ptr<Panel> button = std::unique_ptr<Panel>(new Panel{other});
		return button;
	}

	~Panel()
	{
	}

	void compile() override{

	}

	void framebuffer_resize() override {
		auto pos = get_position();
		if(options_overlay){
			auto min_size = options_overlay->minimum_size();
			SkRect centered_minimum_position = SkRect::MakeXYWH(pos.centerX()-min_size.width()/2.0f,pos.centerY()-min_size.height()/2.0f,default_padding+min_size.width(),default_padding+min_size.height());
			options_overlay->propagate_size_change(centered_minimum_position);
		}
		set_size(pos);
    	return ;
	}

	std::unique_ptr<curan::ui::Overlay> create_options_overlay()
	{
		using namespace curan::ui;

		auto button = Button::make("zoom", system_icons);
		button->set_click_color(SK_ColorGRAY).set_hover_color(SkColorSetARGB(255, 30, 144, 255)).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(100, 80));
		button->add_press_call([this](Button *button, Press press, ConfigDraw *config){
			if (zoom_in)
				zoom_in.deactivate();
			else
				zoom_in.activate(); }
		);

		auto button2 = Button::make("highlight", system_icons);
		button2->set_click_color(SK_ColorGRAY).set_hover_color(SkColorSetARGB(255, 30, 144, 255)).set_waiting_color(SK_ColorDKGRAY).set_size(SkRect::MakeWH(100, 80));
		button2->add_press_call([this](Button *button, Press press, ConfigDraw *config){
		is_highlighting = !is_highlighting;
			if (!current_stroke.empty()){
				insert_in_map(current_stroke);
				current_stroke.clear();
			} 
		});

		auto viwers_container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
		*viwers_container << std::move(button) << std::move(button2);
		viwers_container->set_divisions({0.0, 0.5, 1.0});
		viwers_container->set_color(SK_ColorTRANSPARENT);

    	auto post_sig = [this](Signal sig, bool page_interaction, ConfigDraw* config) {
		std::visit(curan::utilities::overloaded{
		[](Empty arg) {

			},
		[](Move arg) {

			},
		[this,page_interaction](Press arg) {
				if (!page_interaction && options_overlay){
					options_overlay = nullptr;
					return;
				}
			},
		[](Scroll arg) {;

			},
		[](Unpress arg) {

			},
		[](Key arg) {

			},
		[](ItemDropped arg) {;

		} },
		sig);
	};

		return Overlay::make(std::move(viwers_container),post_sig,SkColorSetARGB(10, 125, 125, 125));
	}

	curan::ui::drawablefunction draw() override
	{
		auto lamb = [this](SkCanvas *canvas)
		{
			auto widget_rect = get_position();
			SkAutoCanvasRestore restore{canvas, true};
			canvas->drawRect(widget_rect,background_paint);
			canvas->drawPoints(SkCanvas::PointMode::kPoints_PointMode, current_stroke.size(), current_stroke.data(), paint_square);
			if (is_highlighting)
			{
				double minimum = std::numeric_limits<double>::max();
				auto minimum_index = strokes.end();
				for (auto begin = strokes.begin(); begin != strokes.end(); ++begin)
				{
					double local = begin->second.distance(zoom_in.get_coordinates());
					if (minimum > local)
					{
						minimum = local;
						minimum_index = begin;
					}
					canvas->drawPath(begin->second.rendered_path, paint_stroke);
				}

				for (auto begin = strokes.begin(); begin != strokes.end(); ++begin)
				{
					auto point = begin->second.recorded_points[0];
					paint_square.setColor(SkColorSetARGB(255,0,0,0));
					canvas->drawCircle(SkPoint::Make(point.fX+10,point.fY+10),20,paint_square);
					paint_square.setColor(SK_ColorRED);
					std::string indentifier = "s"+std::to_string(begin->first);
					paint_stroke.setStrokeWidth(0.5f);
					canvas->drawSimpleText(indentifier.data(),indentifier.size(),SkTextEncoding::kUTF8 ,point.fX+10,point.fY+10,text_font, paint_stroke);
					paint_stroke.setStrokeWidth(8);
				}			

				if (minimum_index != strokes.end() && minimum < 20.0f)
				{
					paint_stroke.setStrokeWidth(12);
					paint_stroke.setColor(SK_ColorDKGRAY);
					canvas->drawPath(minimum_index->second.rendered_path, paint_stroke);
					paint_stroke.setStrokeWidth(8);
					paint_stroke.setColor(SK_ColorBLUE);
				}
			}
			else
			{
				for (auto begin = strokes.begin(); begin != strokes.end(); ++begin){
					canvas->drawPath(begin->second.rendered_path, paint_stroke);
					auto point = begin->second.recorded_points[0];
					paint_square.setColor(SkColorSetARGB(125,0,0,0));
					canvas->drawCircle(SkPoint::Make(point.fX+15,point.fY+10),20,paint_square);
					paint_square.setColor(SK_ColorRED);
					std::string indentifier = "s"+std::to_string(begin->first);
					paint_stroke.setStrokeWidth(0.5f);
					canvas->drawSimpleText(indentifier.data(),indentifier.size(),SkTextEncoding::kUTF8 ,point.fX+10,point.fY+10,text_font, paint_stroke);
					paint_stroke.setStrokeWidth(8);
				}
			}

			if (zoom_in)
			{
				zoom_in.draw(canvas);
			}

			if(options_overlay){
				auto image = canvas->getSurface()->makeImageSnapshot();
				canvas->drawImage(image, 0, 0, options, &bluring_paint);
				options_overlay->draw(canvas);
			}


		};
		return lamb;
	}

	curan::ui::callablefunction call() override
	{
		auto lamb = [this](curan::ui::Signal sig, curan::ui::ConfigDraw *config){
			bool interacted = false;
			if(options_overlay)
				return options_overlay->propagate_signal(sig,config);
			
			std::visit(curan::utilities::overloaded{
				[](curan::ui::Empty arg) {

				},
				[&](curan::ui::Move arg){
					zoom_in.store_position(SkPoint::Make((float)arg.xpos, (float)arg.ypos), get_size());
					if(!interacts(arg.xpos,arg.ypos)){
						return;
					}
					if (!is_highlighting){
						if (is_pressed){
							current_stroke.push_back(SkPoint::Make((float)arg.xpos, (float)arg.ypos));
						} else if (!current_stroke.empty()){
							insert_in_map(current_stroke);
							current_stroke.clear();
						}
						interacted = true;
					}
					
				},
				[&](curan::ui::Press arg){
					if(!interacts(arg.xpos,arg.ypos))
						return;
					is_pressed = true;
					interacted = true;
				},
				[](curan::ui::Scroll arg) {
				
				},
				[&](curan::ui::Unpress arg){
					is_pressed = false;
					if (!current_stroke.empty()){
						insert_in_map(current_stroke);
						current_stroke.clear();
					}
					interacted = true;
				},
				[&](curan::ui::Key arg){
					if (arg.key == GLFW_KEY_A && arg.action == GLFW_PRESS){
						if(options_overlay){
							options_overlay = nullptr;
							return;
						}
						auto overlay = create_options_overlay();
						options_overlay = std::move(overlay->take_ownership());
						framebuffer_resize();
					}
				},
				[](curan::ui::ItemDropped arg) {

				}},sig);
				return interacted;
		};
		return lamb;
	};
	
};

int main() {
	try {
		using namespace curan::ui;
		IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),2200,1200 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		auto panel = Panel::make(resources);
		SkRect rect = SkRect::MakeXYWH(0, 0, 2200, 1200);
		panel->set_position(rect);
		panel->compile();
		panel->framebuffer_resize();

		auto caldraw = panel->draw();
		auto calsignal = panel->call();

		ConfigDraw config_draw;

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			canvas->drawColor(SK_ColorWHITE);
			caldraw(canvas);
			glfwPollEvents();
			auto signals = viewer->process_pending_signals();
			if (!signals.empty())
				calsignal(signals.back(),&config_draw);
			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		return 0;
	}
	catch(const std::exception& e){
		std::cout << "Exception thrown:" << e.what() << "\n";
	}
	catch (...) {
		std::cout << "Failed to create window for unknown reason\n";
		return 1;
	}
}

/*
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

		SkPaint paint_options;
		paint_options.setStyle(SkPaint::kStroke_Style);
		paint_options.setAntiAlias(true);
		paint_options.setStrokeWidth(3);
		paint_options.setColor(SK_ColorGRAY);

		std::list<Stroke> strokes;
		std::vector<SkPoint> current_stroke;
		current_stroke.reserve(1000);

		bool is_pressed = false;
		bool is_highlighting = false;
		bool options = false;
		ZoomIn zoom_in;

		while (!glfwWindowShouldClose(viewer->window))
		{
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface *pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas *canvas = pointer_to_surface->getCanvas();
			canvas->drawColor(SK_ColorBLACK);

			canvas->drawPoints(SkCanvas::PointMode::kPoints_PointMode, current_stroke.size(), current_stroke.data(), paint_square);

			if (is_highlighting)
			{
				double minimum = std::numeric_limits<double>::max();
				auto minimum_index = strokes.end();
				for (auto begin = strokes.begin(); begin != strokes.end(); ++begin)
				{
					double local = begin->distance(zoom_in.get_coordinates());
					if (minimum > local)
					{
						minimum = local;
						minimum_index = begin;
					}
					canvas->drawPath((*begin).rendered_path, paint_stroke);
				}
				if (minimum_index != strokes.end() && minimum < 20.0f)
				{
					paint_stroke.setStrokeWidth(12);
					paint_stroke.setColor(SK_ColorDKGRAY);
					canvas->drawPath((*minimum_index).rendered_path, paint_stroke);
					paint_stroke.setStrokeWidth(8);
					paint_stroke.setColor(SK_ColorBLUE);
				}
			}
			else
			{
				for (auto begin = strokes.begin(); begin != strokes.end(); ++begin)
				{
					canvas->drawPath((*begin).rendered_path, paint_stroke);
				}
			}

			if (zoom_in)
			{
				zoom_in.draw(canvas);
			}

			if(options){
				canvas->drawCircle(SkPoint::Make(30,30),30,paint_options);
				for(float angle = 0 ;angle<3.141562*2;angle += 3.141562*2/(3)){
					SkPoint point = SkPoint::Make(30*(std::cos(angle)+1),30*(std::sin(angle)+1));
					canvas->drawLine(SkPoint::Make(30,30),point,paint_options);
				}
			}

			glfwPollEvents();
			auto signals = viewer->process_pending_signals();

			for (const auto &sig : signals)
			{
				std::visit(curan::utilities::overloaded{[](Empty arg) {

														},
														[&](Move arg)
														{
															if (!is_highlighting)
															{
																if (is_pressed)
																{
																	current_stroke.push_back(SkPoint::Make((float)arg.xpos, (float)arg.ypos));
																}
																else if (!current_stroke.empty())
																{
																	strokes.emplace_back(current_stroke);
																	current_stroke.clear();
																}
															}
															zoom_in.store_position(SkPoint::Make((float)arg.xpos, (float)arg.ypos), viewer->get_size());
														},
														[&](Press arg)
														{
															is_pressed = true;
														},
														[](Scroll arg) {
														},
														[&](Unpress arg)
														{
															is_pressed = false;
															if (!current_stroke.empty())
															{
																strokes.emplace_back(current_stroke);
																current_stroke.clear();
															}
														},
														[&](Key arg)
														{
															if (arg.key == GLFW_KEY_A && arg.action == GLFW_PRESS)
															{
																if (zoom_in)
																	zoom_in.deactivate();
																else
																	zoom_in.activate();
																return;
															}
															if (arg.key == GLFW_KEY_S && arg.action == GLFW_PRESS)
															{
																is_highlighting = !is_highlighting;
																options = !options;
																if (!current_stroke.empty())
																{
																	strokes.emplace_back(current_stroke);
																	current_stroke.clear();
																}
																return;
															}
														},
														[](ItemDropped arg) {

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
*/
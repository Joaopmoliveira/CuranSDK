#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include "utils/CircularBuffer.h"
#include <thread>

#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Container.h"

#include "userinterface/widgets/Drawable.h"
#include "userinterface/widgets/definitions/UIdefinitions.h"
#include "utils/Lockable.h"
#include "utils/Overloading.h"
#include <optional>
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/SignalProcessor.h"

#include <iostream>
#include <thread>

#include <vector>
#include <string>

constexpr double minimum_spacing = 20;

constexpr std::array<SkColor,6> colors = {SK_ColorRED,SK_ColorGREEN,SK_ColorBLUE,SK_ColorYELLOW,SK_ColorCYAN,SK_ColorMAGENTA};

class Plotter : public  curan::ui::Drawable , public curan::utilities::Lockable, public curan::ui::SignalProcessor<Plotter> {

    std::vector<curan::utilities::CircularBuffer<SkPoint>> buffers;
    std::vector<uint8_t> verbs;
    SkPath path;
    SkColor background = SK_ColorWHITE;
    

public:

inline SkColor get_background_color() {
	std::lock_guard<std::mutex> g{ get_mutex() };
	return background;
}

inline Plotter& set_background_color(SkColor color) {
	std::lock_guard<std::mutex> g{ get_mutex() };
	background = color;
    return *(this);
}


static std::unique_ptr<Plotter> make(const size_t& plotter_size, const size_t& number_of_buffers){
	std::unique_ptr<Plotter> button = std::unique_ptr<Plotter>(new Plotter{plotter_size,number_of_buffers});
	return button;
}

void compile() override {
    compiled = true;
}

~Plotter(){

}

void append(const SkPoint& in,const size_t& index){
    assert(index < buffers.size());
    buffers[index].put(SkPoint{in});
}

curan::ui::drawablefunction draw() override{
    if(!compiled)
	    throw std::runtime_error("must compile the button before drawing operations");
    
    auto lamb = [this](SkCanvas* canvas) {
        SkAutoCanvasRestore restore{canvas,true};
        auto widget_rect = get_position();

        SkPaint paint;
        paint.setAntiAlias(true);
        paint.setStyle(SkPaint::kFill_Style);
        paint.setColor(get_background_color());
        canvas->drawRect(widget_rect,paint);

        paint.setStyle(SkPaint::kStroke_Style);
        paint.setColor(SK_ColorBLACK);
        paint.setStrokeWidth(1.0);

        const std::string x_label{"t"} ;
        canvas->drawLine(SkPoint::Make(widget_rect.fLeft,widget_rect.fTop+widget_rect.height()),SkPoint::Make(widget_rect.fLeft+widget_rect.width(),widget_rect.fTop+widget_rect.height()),paint); // x 
        canvas->drawSimpleText(x_label.data(),x_label.size(),SkTextEncoding::kUTF8,widget_rect.fLeft+widget_rect.width()-10.0f,widget_rect.fTop+widget_rect.height()-10.0f,SkFont{nullptr},paint); // draw variable name in middle of line

        const std::string y_label{"y"} ;
        canvas->drawLine(SkPoint::Make(widget_rect.fLeft,widget_rect.fTop+widget_rect.height()),SkPoint::Make(widget_rect.fLeft,widget_rect.fTop),paint); // y 
        canvas->drawSimpleText(y_label.data(),y_label.size(),SkTextEncoding::kUTF8,widget_rect.fLeft+10.0f,widget_rect.fTop+10.0f,SkFont{nullptr},paint); // draw variable name in midle of line

        double max_y = -100000.0;
        double max_x = -100000.0;
        double min_y =  100000.0;
        double min_x =  100000.0;

        for(auto & buff : buffers)
            buff.operate([&](SkPoint &in){
                if(in.fX> max_x)
                    max_x = in.fX;
                if(in.fX < min_x)
                    min_x = in.fX;
                if(in.fY> max_y)
                    max_y = in.fY;
                if(in.fY < min_y)
                    min_y = in.fY;
                });

        size_t color_index = 0;
        for(auto & buff : buffers){
            paint.setColor(colors[color_index]);
            auto view = buff.linear_view([&](SkPoint& in){ in.fX = widget_rect.fLeft+widget_rect.width()*(in.fX-min_x)/(max_x-min_x); in.fY = widget_rect.fTop+widget_rect.height()*((max_y-in.fY)/(max_y-min_y));});
            assert(verbs.size()>=view.size());
            if(view.size()>0){
                path = SkPath::Make(view.data(),view.size(),verbs.data(),view.size(),nullptr,0,SkPathFillType::kEvenOdd,true);
                canvas->drawPath(path,paint);
            }
            ++color_index;
        }
	};
    return lamb;
}

curan::ui::callablefunction call() override{
    if(!compiled)
	    throw std::runtime_error("must compile the button before drawing operations");
    auto lamb = [this](curan::ui::Signal sig, curan::ui::ConfigDraw* config) {
		bool interacted = false;
		std::visit(curan::utilities::overloaded{
			[this,config](curan::ui::Empty arg) {

			},
			[this,&interacted,config](curan::ui::Move arg) {
				
			},
			[this,&interacted,config](curan::ui::Press arg) {
				
			},
			[this,config](curan::ui::Scroll arg) {;

			},
			[this,&interacted,config](curan::ui::Unpress arg) {
				
			},
			[this](curan::ui::Key arg) {

			},
			[this](curan::ui::ItemDropped arg) {;

			}},
			sig);
			return interacted;
		};
	return lamb;
}

private:

	Plotter(const size_t& buffer_size, const size_t& number_of_buffers) : buffers{number_of_buffers,curan::utilities::CircularBuffer<SkPoint>{buffer_size}}{
        assert(buffer_size>0);
        assert(number_of_buffers>0);
        assert(colors.size()>=number_of_buffers);
        verbs.resize(buffer_size);
        for(auto& verb : verbs)
            verb = SkPath::kLine_Verb;
        verbs[0] = SkPath::kMove_Verb;
        for(auto & buff : buffers)
            buff.operate([&](SkPoint &in){
                in.fX = 0.0;
                in.fY = 0.0;
            });
    }

	Plotter(const Plotter& other) = delete;

	bool compiled = false;
};

int main()
{
    try
    {
        using namespace curan::ui;
        IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
        std::unique_ptr<Context> context = std::make_unique<Context>();
        DisplayParams param{std::move(context), 1200, 800};
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

        auto plotter = Plotter::make(500,2);
		SkRect rect = SkRect::MakeXYWH(450, 450, 300, 200);
		plotter->set_position(rect);
        plotter->set_background_color(SK_ColorCYAN);
		plotter->compile();

		auto caldraw = plotter->draw();
		auto calsignal = plotter->call();

		ConfigDraw config_draw;

        float time = 0.0;

        while (!glfwWindowShouldClose(viewer->window))
        {
            auto start = std::chrono::high_resolution_clock::now();
            SkSurface *surface = viewer->getBackbufferSurface();
            SkCanvas *canvas = surface->getCanvas();

            canvas->drawColor(SK_ColorWHITE);
			caldraw(canvas);

            time += 0.016f;
            plotter->append(SkPoint::Make(time,std::cos(10.0f*time)),0);
            plotter->append(SkPoint::Make(time,std::sin(10.0f*time)),1);
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
    catch (const std::exception &e)
    {
        std::cout << "\nException thrown:\n" << e.what() << "\n";
    }
    catch (...)
    {
        std::cout << "Failed to create window for unknown reason\n";
        return 1;
    }
}
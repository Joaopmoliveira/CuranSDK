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

class Plotter : public  curan::ui::Drawable , public curan::utilities::Lockable, public curan::ui::SignalProcessor<Plotter> {

    curan::utilities::CircularBuffer<SkPoint> buffer;
    std::vector<uint8_t> verbs;
    SkPath path;
public:

static std::unique_ptr<Plotter> make(const size_t& plotter_size){
	std::unique_ptr<Plotter> button = std::unique_ptr<Plotter>(new Plotter{plotter_size});
	return button;
}

void compile() override {
    compiled = true;
}

~Plotter(){

}

void add(const SkPoint& in){
    buffer.put(SkPoint{in});
}

curan::ui::drawablefunction draw() override{
    if(!compiled)
	    throw std::runtime_error("must compile the button before drawing operations");
    
    auto lamb = [this](SkCanvas* canvas) {
        SkAutoCanvasRestore restore{canvas,true};
        auto widget_rect = get_position();

        canvas->translate(widget_rect.fLeft,widget_rect.fTop);
        
        SkPaint paint;
        paint.setAntiAlias(true);
        paint.setStyle(SkPaint::kStroke_Style);
        paint.setColor(SK_ColorBLACK);
        paint.setStrokeWidth(1.0);

        const std::string x_label{"t"} ;
        canvas->drawLine(SkPoint::Make(widget_rect.fLeft,widget_rect.height()),SkPoint::Make(widget_rect.fLeft+widget_rect.width(),widget_rect.fTop+widget_rect.height()),paint); // x 
        canvas->drawSimpleText(x_label.data(),x_label.size(),SkTextEncoding::kUTF8,widget_rect.fLeft+widget_rect.width()-10.0f,widget_rect.fTop+widget_rect.height()-10.0f,SkFont{nullptr},paint); // draw variable name in middle of line

        const std::string y_label{"y"} ;
        canvas->drawLine(SkPoint::Make(widget_rect.fLeft,widget_rect.height()),SkPoint::Make(widget_rect.fLeft,widget_rect.fLeft),paint); // y 
        canvas->drawSimpleText(y_label.data(),y_label.size(),SkTextEncoding::kUTF8,10.0f,10.0f,SkFont{nullptr},paint); // draw variable name in midle of line

        canvas->translate(10,10);

        double max_y = -100000.0;
        double max_x = -100000.0;
        double min_y =  100000.0;
        double min_x =  100000.0;
        buffer.operate([&](SkPoint &in){
            if(in.fX> max_x)
                max_x = in.fX;
            if(in.fX < min_x)
                min_x = in.fX;
            if(in.fY> max_y)
                max_y = in.fY;
            if(in.fY < min_y)
                min_y = in.fY;
        });
        auto view = buffer.linear_view([&](SkPoint& in){ in.fX = widget_rect.width()*(in.fX-min_x)/(max_x-min_x); in.fY = widget_rect.height()*((max_y-in.fY)/(max_y-min_y));});
        assert(verbs.size()>=view.size());
        if(view.size()>0){
            path = SkPath::Make(view.data(),view.size(),verbs.data(),view.size(),nullptr,0,SkPathFillType::kEvenOdd,true);
            canvas->drawPath(path,paint);
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

	Plotter(const size_t& buffer_size) : buffer{buffer_size}{
        assert(buffer_size>0);
        verbs.resize(buffer_size);
        for(auto& verb : verbs)
            verb = SkPath::kLine_Verb;
        verbs[0] = SkPath::kMove_Verb;
        buffer.operate([&](SkPoint &in){
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

        auto plotter = Plotter::make(500);
		SkRect rect = SkRect::MakeXYWH(450, 450, 300, 200);
		plotter->set_position(rect);
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
            plotter->add(SkPoint::Make(time,std::cos(10.0f*time)));
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
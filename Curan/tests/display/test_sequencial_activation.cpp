#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"

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


class SequencialSteps : public  curan::ui::Drawable , public curan::utilities::Lockable, public curan::ui::SignalProcessor<SequencialSteps> {
public:

    enum Status{
        EXECUTED,
        UNEXECUTED
    };

    enum Connection{
        LINEAR_HORIZONTAl,
        LINEAR_VERTICAl
    };

    enum Envelopment{
        CIRCULAR,
        RECTANGLE
    };

    struct Value{
        Status state = Status::UNEXECUTED;
        sk_sp<SkTextBlob> compiled_description;
    };

private:

    sk_sp<SkTypeface> typeface;
	size_t font_size = 15;
    std::map<std::string,Value> values;
	SkPaint paint;
	SkPaint paint_text;
    SkColor executed_color;
    SkColor unexecuted_color;
    const Envelopment envel;
    const Connection connect;

public:

template<typename... Args>
static std::unique_ptr<SequencialSteps> make(const Envelopment& e, const Connection& c, Args ... arg){
	std::unique_ptr<SequencialSteps> button = std::unique_ptr<SequencialSteps>(new SequencialSteps{e,c});
	return button;
}

void compile() override{

}

~SequencialSteps(){

}

curan::ui::drawablefunction draw() override{
    if(!compiled)
	    throw std::runtime_error("must compile the button before drawing operations");

    if(envel == Envelopment::CIRCULAR && connect == Connection::LINEAR_HORIZONTAl){
        auto lamb = [this](SkCanvas* canvas) {
		    auto widget_rect = get_position();
	    };  
        return lamb;
    }

    if(envel == Envelopment::CIRCULAR && connect == Connection::LINEAR_VERTICAl){
        auto lamb = [this](SkCanvas* canvas) {
		    auto widget_rect = get_position();
	    };  
        return lamb;
    }

    if(envel == Envelopment::RECTANGLE && connect == Connection::LINEAR_HORIZONTAl){
        auto lamb = [this](SkCanvas* canvas) {
		    auto widget_rect = get_position();
	    };  
        return lamb;
    }

    if(envel == Envelopment::RECTANGLE && connect == Connection::LINEAR_HORIZONTAl){
        auto lamb = [this](SkCanvas* canvas) {
		    auto widget_rect = get_position();
	    };  
        return lamb;
    }

    auto lamb = [](SkCanvas* canvas) {

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

SequencialSteps& set_executed_color(SkColor color){
    executed_color = color;
    return *this;
}

SkColor get_executed_color() {
    return executed_color;
}

SequencialSteps& set_unexecuted_color(SkColor color){
    unexecuted_color = color;
    return *this;
}

SkColor get_unexecuted_color() {
    return unexecuted_color;
}

private:

    template<typename... Args>
	SequencialSteps(const Envelopment& e, const Connection& c, Args ... arg) : envel{e}, connect{c} {
	    constexpr size_t size = sizeof ...(Args);
	    const char* loc[size] = { arg... };
        for(size_t ind = 0; ind < size; ++ind){
            std::string arg_ind{loc[ind]};
            Value to_add;
            values.emplace(arg_ind,to_add);
        }
    }

	SequencialSteps(const SequencialSteps& other) = delete;

	bool compiled = false;
};

int main(){
	try {
		using namespace curan::ui;
		IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),1200,800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));


		auto button = SequencialSteps::make(SequencialSteps::Envelopment::CIRCULAR,SequencialSteps::Connection::LINEAR_HORIZONTAl);
		SkRect rect = SkRect::MakeXYWH(50, 100, 300, 200);
		button->set_position(rect);
		button->compile();

		auto caldraw = button->draw();
		auto calsignal = button->call();

		ConfigDraw config_draw;

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			canvas->drawColor(SK_ColorWHITE);
			
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
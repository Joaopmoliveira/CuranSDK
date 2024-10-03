#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/Overlay.h"
#include "userinterface/widgets/TextBlob.h"

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
#include <functional>

constexpr double minimum_spacing = 20;

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
        SkRect bounds;
        SkRect current_pose;
        std::function<std::unique_ptr<curan::ui::Overlay>(void)> create_overlay;
    };

private:

    std::optional<curan::ui::Press> clicked;
    sk_sp<SkTypeface> typeface;
	size_t font_size = 15;
    SkFont font;
    std::map<std::string,Value*> values;
    std::list<Value> sequential_values;
	SkPaint paint;
	SkPaint paint_text;
    SkColor executed_color = SK_ColorGREEN;
    SkColor unexecuted_color = SK_ColorRED; 
    SkColor background_color = SK_ColorBLUE;
    const Envelopment envel;
    const Connection connect;

public:

void overlaycreator(const std::string& s, std::function<std::unique_ptr<curan::ui::Overlay>(void)> create_overlay){
    std::lock_guard<std::mutex> g{get_mutex()};
    auto search = values.find(s);
    if ( search != values.end())
        search->second->create_overlay = create_overlay;
}

void on(const std::string& s){
    std::lock_guard<std::mutex> g{get_mutex()};
    auto search = values.find(s);
    if ( search != values.end())
        search->second->state = Status::EXECUTED;
}

void off(const std::string& s){
    std::lock_guard<std::mutex> g{get_mutex()};
    auto search = values.find(s);
    if ( search != values.end())
        search->second->state = Status::UNEXECUTED;
}

template<typename... Args>
static std::unique_ptr<SequencialSteps> make(const Envelopment& e, const Connection& c, Args ... arg){
	std::unique_ptr<SequencialSteps> button = std::unique_ptr<SequencialSteps>(new SequencialSteps{e,c,(arg)...});
	return button;
}

void compile() override {
    font = SkFont(typeface, font_size, 1.0f, 0.0f);
    
    double max_width = minimum_spacing;
    double max_height = minimum_spacing;

    switch(connect){
    case Connection::LINEAR_HORIZONTAl:
        for(auto& v : values){
            v.second->compiled_description = SkTextBlob::MakeFromString(v.first.c_str(), font);
            font.measureText(v.first.c_str(), v.first.size(), SkTextEncoding::kUTF8, &v.second->bounds);
            max_width += v.second->bounds.width() + minimum_spacing;
            if(max_height-2*minimum_spacing<v.second->bounds.height())
                max_height = v.second->bounds.height()+2*minimum_spacing;
        }
    break;
    case Connection::LINEAR_VERTICAl:
        for(auto& v : values){
            v.second->compiled_description = SkTextBlob::MakeFromString(v.first.c_str(), font);
            font.measureText(v.first.c_str(), v.first.size(), SkTextEncoding::kUTF8, &v.second->bounds);
            max_height += v.second->bounds.height() + minimum_spacing;
            if(max_width-2*minimum_spacing<v.second->bounds.width())
                max_width = v.second->bounds.width()+2*minimum_spacing;
        }
    break;
    }
    set_size(SkRect::MakeWH(max_width,max_height));
    compiled = true;
}

~SequencialSteps(){

}

curan::ui::drawablefunction draw() override{
    if(!compiled)
	    throw std::runtime_error("must compile the button before drawing operations");

    if(envel == Envelopment::CIRCULAR && connect == Connection::LINEAR_HORIZONTAl){
        auto lamb = [this](SkCanvas* canvas) {
		    auto widget_rect = get_position();
            paint.setColor(get_background_color());
            auto size = get_size();

            SkRect drawable = SkRect::MakeXYWH(widget_rect.centerX()-widget_rect.width()/2.0,widget_rect.centerY()-size.height()/2.0,widget_rect.width(),size.height());
            canvas->drawRect(drawable,paint);

            SkRect connective_line = SkRect::MakeXYWH(widget_rect.centerX()-widget_rect.width()/2.0f+20.0f,widget_rect.centerY()-10.0f/2.0f,widget_rect.width()-40.0f,10.0f);
            paint.setColor(get_unexecuted_color());
            canvas->drawRect(connective_line,paint);

            double extra_increments = 0.0;
            if(sequential_values.size()>1)
                extra_increments = (drawable.width()-size.width())/(sequential_values.size()-1);
            if(extra_increments<0.0) extra_increments = 0.0;

            double y_offset = widget_rect.centerY() ; 
            double x_offset = widget_rect.centerX() - widget_rect.width()/2.0 + minimum_spacing;
            for(auto& v : sequential_values){
                if(v.state == Status::UNEXECUTED)
                    paint.setColor(get_unexecuted_color());
                else
                    paint.setColor(get_executed_color());
                SkRect work_position = SkRect::MakeXYWH(x_offset-5,y_offset-(size.height()-20)/2.0,v.bounds.width()+10,size.height()-20);
                {
                    std::lock_guard<std::mutex> g{get_mutex()};
                    v.current_pose = work_position;
                }
                canvas->drawOval(work_position,paint);
                paint.setColor(SK_ColorBLACK);
                canvas->drawTextBlob(v.compiled_description,x_offset,y_offset+v.bounds.height()/2.0,paint);
                x_offset += v.bounds.width() + minimum_spacing+extra_increments;
            }
	    };  
        return lamb;
    }

    if(envel == Envelopment::CIRCULAR && connect == Connection::LINEAR_VERTICAl){
        auto lamb = [this](SkCanvas* canvas) {
		    auto widget_rect = get_position();
            paint.setColor(get_background_color());
            auto size = get_size();

            SkRect drawable = SkRect::MakeXYWH(widget_rect.centerX()-size.width()/2.0,widget_rect.centerY()-size.height()/2.0,size.width(),size.height());
            canvas->drawRect(drawable,paint);

            double y_offset = widget_rect.centerY() - size.height()/2.0;
            double x_offset = widget_rect.centerX() - size.width()/2.0;
            for(auto& v : sequential_values){
                if(v.state == Status::UNEXECUTED)
                    paint.setColor(get_unexecuted_color());
                else
                    paint.setColor(get_executed_color());
                canvas->drawTextBlob(v.compiled_description,x_offset,y_offset,paint);
                y_offset += v.bounds.height() + minimum_spacing;
            }
	    };  
        return lamb;
    }

    if(envel == Envelopment::RECTANGLE && connect == Connection::LINEAR_HORIZONTAl){
        auto lamb = [this](SkCanvas* canvas) {
		    auto widget_rect = get_position();
            paint.setColor(get_background_color());
            auto size = get_size();

            SkRect drawable = SkRect::MakeXYWH(widget_rect.centerX()-widget_rect.width()/2.0,widget_rect.centerY()-size.height()/2.0,widget_rect.width(),size.height());
            paint.setColor(get_background_color());
            canvas->drawRect(drawable,paint);

            SkRect connective_line = SkRect::MakeXYWH(widget_rect.centerX()-widget_rect.width()/2.0f+10.0f,widget_rect.centerY()-10.0/2.0,widget_rect.width()-40.0f,10.f);
            paint.setColor(get_unexecuted_color());
            canvas->drawRect(connective_line,paint);
            double extra_increments = 0.0;
            if(sequential_values.size()>1)
                extra_increments = (drawable.width()-size.width())/(sequential_values.size()-1);
            if(extra_increments<0.0) extra_increments = 0.0;

            double y_offset = widget_rect.centerY() ; 
            double x_offset = widget_rect.centerX() - widget_rect.width()/2.0 + minimum_spacing;
            for(auto& v : sequential_values){
                if(v.state == Status::UNEXECUTED)
                    paint.setColor(get_unexecuted_color());
                else
                    paint.setColor(get_executed_color());
                SkRect work_position = SkRect::MakeXYWH(x_offset-5,y_offset-(size.height()-20)/2.0,v.bounds.width()+10,size.height()-20);
                {
                    std::lock_guard<std::mutex> g{get_mutex()};
                    v.current_pose = work_position;
                }
                canvas->drawRect(work_position,paint);
                paint.setColor(SK_ColorBLACK);
                canvas->drawTextBlob(v.compiled_description,x_offset,y_offset+v.bounds.height()/2.0,paint);
                x_offset += v.bounds.width() + minimum_spacing+extra_increments;
            }
	    };  
        return lamb;
    }

    if(envel == Envelopment::RECTANGLE && connect == Connection::LINEAR_VERTICAl){
        auto lamb = [this](SkCanvas* canvas) {
		    auto widget_rect = get_position();
            paint.setColor(get_background_color());
            auto size = get_size();
            
            SkRect drawable = SkRect::MakeXYWH(widget_rect.centerX()-size.width()/2.0,widget_rect.centerY()-size.height()/2.0,size.width(),size.height());
            canvas->drawRect(drawable,paint);

            double y_offset = widget_rect.centerY() - size.height()/2.0;
            double x_offset = widget_rect.centerX() - size.width()/2.0 + minimum_spacing;
            for(auto& v : sequential_values){  
                {
                    std::lock_guard<std::mutex> g{get_mutex()};
                    if(v.state == Status::UNEXECUTED)
                        paint.setColor(get_unexecuted_color());
                    else
                        paint.setColor(get_executed_color());
                }
                canvas->drawTextBlob(v.compiled_description,x_offset,y_offset,paint);
                y_offset += v.bounds.height() + minimum_spacing;
            }
	    };  
        return lamb;
    }

    //this should never happen
    
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
                for(auto& v : sequential_values){
                    std::lock_guard<std::mutex> g{get_mutex()};
                    if(v.create_overlay && v.current_pose.contains((float)arg.xpos,(float)arg.ypos))
                        config->stack_page->stack(v.create_overlay());
                }
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
    std::lock_guard<std::mutex> g{get_mutex()};
    executed_color = color;
    return *this;
}

SkColor get_executed_color() {
    std::lock_guard<std::mutex> g{get_mutex()};
    return executed_color;
}

SequencialSteps& set_unexecuted_color(SkColor color){
    std::lock_guard<std::mutex> g{get_mutex()};
    unexecuted_color = color;
    return *this;
}

SkColor get_unexecuted_color() {
    std::lock_guard<std::mutex> g{get_mutex()};
    return unexecuted_color;
}

SkColor get_background_color(){
    std::lock_guard<std::mutex> g{get_mutex()};
    return background_color;
}

SequencialSteps& set_background_color(SkColor color){
    std::lock_guard<std::mutex> g{get_mutex()};
    background_color = color;
    return *this;
};

private:

    template<typename... Args>
	SequencialSteps(const Envelopment& e, const Connection& c, Args ... arg) : envel{e}, connect{c} {
	    constexpr size_t size = sizeof ...(Args);
	    const char* loc[size] = { arg... };
        for(size_t ind = 0; ind < size; ++ind){
            std::string arg_ind{loc[ind]};
            Value to_add;
            sequential_values.push_back(to_add);
            values.emplace(arg_ind,&sequential_values.back());
        }

        paint.setStyle(SkPaint::kFill_Style);
	    paint.setAntiAlias(true);
	    paint.setStrokeWidth(4);
	    paint.setColor(SK_ColorDKGRAY);

    }

	SequencialSteps(const SequencialSteps& other) = delete;

	bool compiled = false;
};

int main(){
    try {
		using namespace curan::ui;
		IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();
		DisplayParams param{ std::move(context),1200,800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

		auto sequential = SequencialSteps::make(SequencialSteps::Envelopment::CIRCULAR,
                                                SequencialSteps::Connection::LINEAR_HORIZONTAl,
                                                "One","Two","Three");
        sequential->set_unexecuted_color(SkColorSetARGB(0xFF, 0xE5, 0xD9, 0x5C))
                   .set_unexecuted_color(SkColorSetARGB(0xFF, 0x5C, 0xC1, 0xE1))
                   .set_background_color(SkColorSetARGB(0xFF, 0x99, 0xDE, 0xBA));
        sequential->overlaycreator("One",[](){
            auto text = curan::ui::TextBlob::make("Clicked One");
            text->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	        auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	        *container <<  std::move(text);
	        container->set_color(SK_ColorTRANSPARENT);
	        return Overlay::make(std::move(container),SK_ColorTRANSPARENT,true);
        });
        sequential->overlaycreator("Two",[](){
            auto text = curan::ui::TextBlob::make("Clicked Two");
            text->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	        auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	        *container <<  std::move(text);
	        container->set_color(SK_ColorTRANSPARENT);
	        return Overlay::make(std::move(container),SK_ColorTRANSPARENT,true);
        });
        sequential->overlaycreator("Three",[](){
            auto text = curan::ui::TextBlob::make("Clicked Three");
            text->set_text_color(SK_ColorWHITE).set_background_color(SK_ColorBLACK).set_size(SkRect::MakeWH(200, 80));
	        auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	        *container <<  std::move(text);
	        container->set_color(SK_ColorTRANSPARENT);
	        return Overlay::make(std::move(container),SK_ColorTRANSPARENT,true);
        });
        SequencialSteps* sequential_ptr = sequential.get();

		auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
		*container << std::move(sequential);

        std::atomic<bool> continue_shifting = true;
        std::thread th{[&](){
            static size_t counter = 0;
            bool value = true;
            while(continue_shifting){
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                if(counter % 10 == 0){
                    if(value) sequential_ptr->off("Two");
                    else  sequential_ptr->on("Two");
                    value = ! value;
                }
                ++counter;
            }
        }};

		auto page = Page{std::move(container),SK_ColorBLACK};
		page.update_page(viewer.get());

        viewer->set_minimum_size(page.minimum_size());

		ConfigDraw config_draw{ &page };

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();

			if (viewer->was_updated()) {
		    	page.update_page(viewer.get());
				viewer->update_processed();
			}
			page.draw(canvas);
			auto signals = viewer->process_pending_signals();

			if (!signals.empty())
				page.propagate_signal(signals.back(), &config_draw);

            glfwPollEvents();

			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
        continue_shifting = false;
        th.join();
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
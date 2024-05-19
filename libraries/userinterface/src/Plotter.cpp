#include "userinterface/widgets/Plotter.h"
#include "utils/Overloading.h"

namespace curan {
namespace ui {

constexpr std::array<SkColor,6> colors = {SK_ColorRED,SK_ColorGREEN,SK_ColorBLUE,SK_ColorYELLOW,SK_ColorCYAN,SK_ColorMAGENTA};


std::unique_ptr<Plotter> Plotter::make(const size_t& plotter_size, const size_t& number_of_buffers){
	std::unique_ptr<Plotter> button = std::unique_ptr<Plotter>(new Plotter{plotter_size,number_of_buffers});
	return button;
}

void Plotter::compile() {
    compiled = true;
}

Plotter::~Plotter(){

}

curan::ui::drawablefunction Plotter::draw(){
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

        for(auto & buff : buffers){
            std::lock_guard<std::mutex> g{get_mutex()};
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
        }

        size_t color_index = 0;
        for(auto & buff : buffers){
            paint.setColor(colors[color_index]);
            std::vector<SkPoint> view;
            {
                std::lock_guard<std::mutex> g{get_mutex()};
                view = view = buff.linear_view([&](SkPoint& in){ in.fX = widget_rect.fLeft+widget_rect.width()*(in.fX-min_x)/(max_x-min_x); in.fY = widget_rect.fTop+widget_rect.height()*((max_y-in.fY)/(max_y-min_y));});
            }
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

curan::ui::callablefunction Plotter::call(){
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

Plotter::Plotter(const size_t& buffer_size, const size_t& number_of_buffers) : buffers{number_of_buffers,curan::utilities::CircularBuffer<SkPoint>{buffer_size}}{
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

}
}

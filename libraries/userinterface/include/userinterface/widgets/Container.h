#ifndef CURAN_CONTAINER_HEADER_FILE_
#define CURAN_CONTAINER_HEADER_FILE_

#include "Drawable.h"
#include "utils/Lockable.h"
#include "ImageWrapper.h"
#include "RuntimeEffect.h"
#include <vector>
#include <memory>
#include <optional>

/*
The Container class is the glue that allows us to 
create pages with multiple widgets. Basically all 
things that can be drawn, derive from the drawable 
class, which contains two methods, the draw method 
and the call method. 

The container is responsible to position widgets on 
different regions of the page.

By default the container is linear, implying that 
the space that the container ocupies will be divided 
equaly between the widgets contained inside it. The 
container can be vertical or horizontal. If its 
vertical the container divides the space allocated 
to it with a uniform vertical space, and if horizontal 
it divides the space equaly horizontaly. 

Its also possible to specify a variable container size, 
in which case, you can specify the relative coordinates 
of the widgets contained inside the container. 

You can specify the background color of the container, 
or if you need more customized behavior you can specify 
a background texture to be used, or even customize the 
behavior of a shader which fills the background area. 

An example of using containers is the following 

int main(){
	
	% We allocate two image display widgets, where we want 
	% the first image display ocupying the left region of 
	% the container and the second image display ocupying the
	% right region of the container. They should ocupy the same
	% relative horizontal area of the container

	auto first_image_display = ImageDisplay::make();
	auto second_image_display = ImageDisplay::make();

	% Given the previous specification we create a linear 
	% container with an horizontal layout and when we put 
	% the widgets inside the container the first one ocupies 
	% the left region and the second ocupies the right 
	% region, i.e., its always left to right and top to bottom

	auto displaycontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*displaycontainer << std::move(first_image_display) << std::move(second_image_display); 

	% now we want three buttons to control the display functions
	% a button to connect a socket, a button to collect data and 
	% an button to show additional options

	auto connect = Button::make("Connect",resources);
	connect->set_click_color(SK_ColorGRAY)
					 .set_hover_color(SK_ColorDKGRAY)
					 .set_waiting_color(SK_ColorBLACK)
					 .set_size(SkRect::MakeWH(100, 80));

	auto collect = Button::make("Data Collection",resources);
	collect->set_click_color(SK_ColorGRAY)
							.set_hover_color(SK_ColorDKGRAY)
							.set_waiting_color(SK_ColorBLACK)
							.set_size(SkRect::MakeWH(200, 80));

	auto options = Button::make("Options",resources);
	options->set_click_color(SK_ColorGRAY)
				   .set_hover_color(SK_ColorDKGRAY)
				   .set_waiting_color(SK_ColorBLACK)
				   .set_size(SkRect::MakeWH(200, 80));

	% we can compose the buttons in an horizontal display
	% with an equal spacing between them 

	auto buttoncontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::HORIZONTAL);
	*buttoncontainer << std::move(connect) << std::move(collect) << std::move(options);

	% Now that we have both containers we can join them in a container
	% but we don't want the buttons to have the same space as the image
	% displays. Because the images are more important we want them to ocupy 
	% 90% of the vertical spacing, thus we can write

	auto widgetcontainer = Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	*widgetcontainer << std::move(buttoncontainer) << std::move(displaycontainer);
	widgetcontainer->set_divisions({ 0.0 , 0.1 , 1.0 });

	% where set_divisions specifying the vertical spacing that the widgetcontainer 
	% will allocate for the button container and the display container

}

*/

namespace curan {
namespace ui {

class Container : public Drawable , utilities::Lockable{
public:

enum class ContainerType{
    LINEAR_CONTAINER,
    VARIABLE_CONTAINER
};

enum class Arrangement {
	VERTICAL,
	HORIZONTAL,
	UNDEFINED
};

~Container();

drawablefunction draw() override;
callablefunction call() override;

SkRect minimum_size() override;

inline Container& set_color(SkColor color){
	std::lock_guard<std::mutex> g{ get_mutex() };
	layout_color = color;
	return *(this);
}

inline SkColor get_color(){
	std::lock_guard<std::mutex> g{ get_mutex() };
	return layout_color;
}

inline Container& set_divisions(const std::vector<SkScalar>& indivision){
	std::lock_guard<std::mutex> g{ get_mutex() };
	if(type==ContainerType::VARIABLE_CONTAINER)
		throw std::runtime_error("Trying to set divisions on a variable container");
	divisions = indivision;
	return *(this);
}

inline Container& set_shader_colors(std::array<SkColor,2> in_shader_colors){
	std::lock_guard<std::mutex> g{ get_mutex() };
	shader_colors = in_shader_colors;
	return *(this);
}

inline Container& set_variable_layout(const std::vector<SkRect>& indivision){
	std::lock_guard<std::mutex> g{ get_mutex() };
	if(type==ContainerType::LINEAR_CONTAINER)
		throw std::runtime_error("Trying to set variable layout on linear container");
	rectangles_of_contained_layouts = indivision;
	return *(this);
}

inline bool is_leaf() override {
	return false;
}

void compile() override;

void framebuffer_resize(const SkRect& new_page_size) override;

Container& linearize_container(std::vector<drawablefunction>& callable_draw, std::vector<callablefunction>& callable_signal);

inline std::vector<SkRect> get_positioning() {
	if(!compiled)
		throw std::runtime_error("cannot query positions while container not compiled");
	return rectangles_of_contained_layouts;
};

Container& operator<<(std::unique_ptr<Drawable> value);

static std::unique_ptr<Container> make(const ContainerType& type, const Arrangement& arragement);
static std::unique_ptr<Container> make(const ContainerType& type, const Arrangement& arragement, ImageWrapper image_wrapper);
static std::unique_ptr<Container> make(const ContainerType& type, const Arrangement& arragement, RuntimeEffect image_wrapper);

private:

Container(const ContainerType& type,const Arrangement& arragement);
Container(const ContainerType& type,const Arrangement& arragement, ImageWrapper image_wrapper);
Container(const ContainerType& type,const Arrangement& arragement, RuntimeEffect image_wrapper);
			
std::optional<ImageWrapper> background_image;
std::optional<RuntimeEffect> background_effect;
std::optional<std::array<SkColor,2>> shader_colors;
SkPaint paint_layout;
std::vector<SkScalar> divisions;
std::vector<std::unique_ptr<Drawable>> contained_layouts;
std::vector<SkRect> rectangles_of_contained_layouts;
bool horizontaly_fixed = false;
bool vertically_fixed = false;
ContainerType type;
Arrangement arragement;
SkColor layout_color;
bool compiled = false;
};

}
}
#endif
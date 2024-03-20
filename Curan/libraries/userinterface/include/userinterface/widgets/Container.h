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
The Container class is the glue that allows us to create 
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
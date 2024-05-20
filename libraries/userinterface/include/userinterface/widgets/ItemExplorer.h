#ifndef CURAN_ICON_EXPLORER_HEADER_FILE_
#define CURAN_ICON_EXPLORER_HEADER_FILE_

#include "Drawable.h"
#include "utils/Lockable.h"
#include "definitions/UIdefinitions.h"
#include <optional>
#include "IconResources.h"
#include "SignalProcessor.h"
#include <queue>
#include "ImageWrapper.h"

namespace curan {
namespace ui {

/*

*/

struct Item {
	sk_sp<SkImage> image;
	std::optional<curan::ui::ImageWrapper> wrapper;
	SkRect current_pos;			
	bool is_selected = false;
	bool is_highlighted = false;
	std::string text;
	SkRect cached_text_size;
	size_t identifier = 0;

	template<typename ...T>
	explicit Item(size_t in_identifier, std::string in_text,T ...u) : identifier{in_identifier},text{in_text},wrapper{std::in_place,std::forward<T>(u)...}{if(wrapper) image=(*wrapper).image;
	}
	
	explicit Item(size_t identifier, std::string text);
};

class ItemExplorer : public  Drawable , public utilities::Lockable, public SignalProcessor<ItemExplorer> {
public:

	static std::unique_ptr<ItemExplorer> make(const std::string& default_icon_name,IconResources& system_icons,bool is_exclusive = true);
	void compile() override;

	~ItemExplorer();

	std::list<size_t> highlighted();

	void add(Item&& item_to_add);

	void remove(size_t identifier);

	drawablefunction draw() override;
	callablefunction call() override;

	inline ItemExplorer& set_font_size(const double & in_size) {
		std::lock_guard<std::mutex> g{ get_mutex() };
		font_size = in_size;
        return *(this);
	}

	inline ItemExplorer& set_font_source(sk_sp<SkTypeface> in_typeface) {
		std::lock_guard<std::mutex> g{ get_mutex() };
		typeface = in_typeface;
        return *(this);
	}

	inline SkColor get_background_color() {
		std::lock_guard<std::mutex> g{ get_mutex() };
		return color_background;
	}

	inline ItemExplorer& set_background_color(SkColor color) {
		std::lock_guard<std::mutex> g{ get_mutex() };
		color_background = color;
        return *(this);
	}

	inline SkColor get_text_color() {
		std::lock_guard<std::mutex> g{ get_mutex() };
		return color_text;
	}

	inline ItemExplorer& set_text_color(SkColor color) {
		std::lock_guard<std::mutex> g{ get_mutex() };
		color_text = color;
        return *(this);
	}

	inline SkColor get_waiting_color() {
		std::lock_guard<std::mutex> g{ get_mutex() };
		return color_waiting;
	}

	inline ItemExplorer& set_waiting_color(SkColor new_waiting_color) {
		std::lock_guard<std::mutex> g{ get_mutex() };
		color_waiting = new_waiting_color;
        return *(this);
	}

	inline ItemExplorer& set_hover_color(SkColor new_click_color) {
		std::lock_guard<std::mutex> g{ get_mutex() };
		color_hover = new_click_color;
        return *(this);
	}

	inline SkColor get_hover_color() {
		std::lock_guard<std::mutex> g{ get_mutex() };
		return color_hover;
	}

	inline ItemExplorer& set_selected_color(SkColor new_click_color) {
		std::lock_guard<std::mutex> g{ get_mutex() };
		color_selected = new_click_color;
        return *(this);
	}

	inline SkColor get_selected_color() {
		std::lock_guard<std::mutex> g{ get_mutex() };
		return color_selected;
	}

private:

	ItemExplorer(const std::string& default_icon_name,IconResources& system_icons,bool exclusive);

	int selected_image_identifier = 0;
	SkRect preview_rectangle;
	bool is_exclusive = true;

	SkPaint paint_image_background;
	SkPaint paint_background;
	SkPaint text_paint;
	SkFont font;
	sk_sp<SkTypeface> typeface;

	sk_sp<SkImage> default_item;

	double font_size = DEFAULT_TEXT_SIZE;

	SkColor color_background;
	SkColor color_waiting;
	SkColor color_hover;
	SkColor color_text;
	SkColor color_selected;

	SkScalar vertical_scroll = 0.0;

	std::atomic<float> maximum_height{ 0 };
	std::atomic<float> current_height_offset{ 0 };
	bool compiled = false;
	std::list<Item> item_list;
	std::list<size_t> to_remove;
	std::list<Item> to_add;

};
	
}
}

#endif
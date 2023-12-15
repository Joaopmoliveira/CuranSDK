#ifndef CURAN_SLIDER_PANEL_HEADER_FILE_
#define CURAN_SLIDER_PANEL_HEADER_FILE_

#include <functional>
#include "userinterface/widgets/definitions/Interactive.h"
#include "userinterface/widgets/ImageWrapper.h"
#include <unordered_map>
#include "userinterface/widgets/Drawable.h"
#include "utils/Lockable.h"
#include "userinterface/widgets/SignalProcessor.h"
#include "itkExtractImageFilter.h"
#include "itkImage.h"
#include "userinterface/widgets/IconResources.h"

namespace curan {
namespace ui {

using stroke_added_callback = std::function<curan::ui::Stroke(void)>;
using sliding_panel_callback = std::function<std::optional<curan::ui::ImageWrapper>(size_t slider_value)>;

enum MaskUsed
{
	CLEAN,
	DIRTY
};

enum Direction
{
	X = 0,
	Y = 1,
	Z = 2
};

class Mask
{
	MaskUsed _mask_flag = MaskUsed::CLEAN;
	std::unordered_map<size_t, curan::ui::Stroke> recorded_strokes;

public:
	Mask(){}
	Mask(const Mask &m) = delete;
	Mask &operator=(const Mask &m) = delete;

	template <typename... T>
	void try_emplace(T&&... u)
	{
		recorded_strokes.try_emplace(std::forward<T>(u)...);
		_mask_flag = MaskUsed::DIRTY;
	}

	void container_resized(const SkMatrix &inverse_homogenenous_transformation);

	void draw(SkCanvas *canvas,const SkMatrix& homogenenous_transformation,const SkPoint& point, bool is_highlighting,SkPaint& paint_stroke,SkPaint& paint_square,const SkFont& text_font);
};

constexpr size_t size_of_slider_in_height = 30;
constexpr unsigned int Dimension = 3;

class SlidingPanel : public curan::ui::Drawable, public curan::utilities::Lockable, public curan::ui::SignalProcessor<SlidingPanel>
{
public:
	enum class SliderStates
	{
		WAITING,
		PRESSED,
		HOVER,
		SCROLL,
	};

	struct image_info
	{
		std::optional<curan::ui::ImageWrapper> image;
		double width_spacing = 1;
		double height_spacing = 1;
	};

private:
	using PixelType = unsigned char;
	
	using ImageType = itk::Image<PixelType, Dimension>;
	using ExtractFilterType = itk::ExtractImageFilter<ImageType, ImageType>;

	ImageType::Pointer contained_volume;
	ExtractFilterType::Pointer extract_filter;

	SkRect reserved_slider_space;
	SkRect reserved_drawing_space;
	size_t counter = 0;

	SkColor colbuton = {SK_ColorRED};
	SkPaint paint_square;
	SkPaint paint_stroke;
	SkPaint background_paint;
	SkPaint paint_points;

	std::vector<Mask> masks;
	curan::ui::PointCollection current_stroke;

	SkRect background_rect;
	SkMatrix homogenenous_transformation;
	SkMatrix inverse_homogenenous_transformation;

	curan::ui::IconResources &system_icons;
	SkFont text_font;

	image_info background;

	bool is_pressed = false;
	bool is_highlighting = false;
	curan::ui::ZoomIn zoom_in;

	size_t _current_index = 0;
	float current_value = 0.5;
	float value_pressed = 0.5;
	float dragable_percent_size = 0.01f;

	SkSamplingOptions options;
	sk_sp<SkImageFilter> imgfilter;

	SkColor hover_color = SK_ColorLTGRAY;
	SkColor waiting_color = SK_ColorCYAN;
	SkColor click_color = SK_ColorGRAY;
	SkColor slider_color = SK_ColorGRAY;

	SliderStates current_state = SliderStates::WAITING;
	Direction direction = Direction::X;
	SkPaint slider_paint;

	void query_if_required();

	Mask &current_mask();

	image_info extract_slice_from_volume(size_t index);

	SlidingPanel(curan::ui::IconResources &other, ImageType::Pointer in_contained_volume, Direction in_direction);
	
	void insert_in_map(const curan::ui::PointCollection &future_stroke);

public:
	static std::unique_ptr<SlidingPanel> make(curan::ui::IconResources &other, ImageType::Pointer in_contained_volume, Direction in_direction);

	~SlidingPanel();

	void compile() override;

	void update_volume(ImageType::Pointer in_contained_volume, Direction in_direction);

	void framebuffer_resize(const SkRect &new_page_size) override;

	inline SlidingPanel &trigger(float in_current_value)
	{
		value_pressed = in_current_value;
		return *(this);
	}

	inline float read_trigger()
	{
		return value_pressed;
	}

	inline SlidingPanel &set_current_value(float in_current_value)
	{
		std::lock_guard<std::mutex> g{get_mutex()};
		if (in_current_value < 0.0)
			in_current_value = 0.0;
		if (in_current_value > 1.0)
			in_current_value = 1.0;
		current_value = in_current_value;
		query_if_required();
		return *(this);
	}

	inline float get_current_value()
	{
		std::lock_guard<std::mutex> g{get_mutex()};
		return current_value;
	}

	inline SkColor get_hover_color()
	{
		std::lock_guard<std::mutex> g{get_mutex()};
		return hover_color;
	}

	inline SlidingPanel &set_hover_color(SkColor color)
	{
		std::lock_guard<std::mutex> g{get_mutex()};
		hover_color = color;
		return *(this);
	}

	inline SkColor get_waiting_color()
	{
		std::lock_guard<std::mutex> g{get_mutex()};
		return waiting_color;
	}

	inline SlidingPanel &set_waiting_color(SkColor new_waiting_color)
	{
		std::lock_guard<std::mutex> g{get_mutex()};
		waiting_color = new_waiting_color;
		return *(this);
	}

	inline SkColor get_click_color()
	{
		std::lock_guard<std::mutex> g{get_mutex()};
		return click_color;
	}

	inline SlidingPanel &set_click_color(SkColor new_click_color)
	{
		std::lock_guard<std::mutex> g{get_mutex()};
		click_color = new_click_color;
		return *(this);
	}

	inline SkColor get_slider_color()
	{
		std::lock_guard<std::mutex> g{get_mutex()};
		return slider_color;
	}

	inline SlidingPanel &set_slider_color(SkColor new_slider_color)
	{
		std::lock_guard<std::mutex> g{get_mutex()};
		slider_color = new_slider_color;
		return *(this);
	}

	inline SliderStates get_current_state()
	{
		std::lock_guard<std::mutex> g{get_mutex()};
		return current_state;
	}

	inline SlidingPanel &set_current_state(SliderStates state)
	{
		std::lock_guard<std::mutex> g{get_mutex()};
		current_state = state;
		return *(this);
	}

	curan::ui::drawablefunction draw() override;

	curan::ui::callablefunction call() override;
};

}
}

#endif